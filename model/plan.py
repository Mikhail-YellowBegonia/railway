"""调度计划的数据类型（计划层 P1 / P3a / P7b）。

规格：`docs/plan_layer_roadmap.md` §2（Q23-1 定稿）。本模块**只定义数据结构与
自洽校验**；运行消费点位于 ``model.plan_dispatch``，本模块不执行列车行为。

**模型要点**（逐条对应已拍板结论）：

- **计划 = 有序条目 + 指令指针**（Q15）；**指针在控制车（车厢）上**（Q14）。
  本模块的 `Plan` 只是这个数据；**指针推进策略**（哪些命令自然步进、哪些只由事件
  推进、单一步进者、到达事件幂等、一次到达事件最多走一圈）见 roadmap §2.3，
  **实现在 P3**，本模块的 `Plan.advance()` 只做"移动指针 + 回绕"这一个纯位移动作。
- **条目只有一类，携带逻辑；"路径限定"是它的属性**（Q21-2）：`PlanItem.anchors`
  是可选的有序锚点序列（硬约束，Q22-3），解析之后**只是路径**——它不是第二种条目。
- **锚点 = 图上已有的节点**；**控制点 = 锚点的一个字段**（"在道岔 N 走哪条出边"，
  Q22-5）⇒ 不需要独立的控制点概念，`Anchor.exit_edge_id` 给定时即控制点。
- **终点沿用现有 `TrainEntity.goal` 的三元组** `(edge_id, t, direction)`，
  允许停在边中途（站台停靠不需要新实体）。
- **demo 命令集合 = `goto` / `goto_couple` / `wait_couple` / `decouple` /
  `reverse`，无跳转命令**（Q23-8）：循环只靠"走完回绕第一项"。
- **引用一律落在物理要素上**（Q22-2）：锚点 → `node_id`、控制点 → `edge_id`、
  终点 → `(edge_id, t)`；demo 新连挂目标 → `CoupleSelector(edge_id, direction)`，运行时按
  冻结路线进入方向选择、认领并锁定第一个可达暴露端。旧式
  `TrainRef(wagon_id,end)` 仅保留兼容。
  **"段（simple_segment）"不参与引用**（`model/segments.py` 只作派生视图）。
- **失效语义的判据由本模块提供**（Q23-2 的"永久失效 = 引用对象不存在"）：
  `validate(...)` 在给出 network 时检查节点/边是否仍存在、控制点出口是否可达。

⚠ **只读纪律（Q3）**：计划属于**车厢的域数据**，因此它将来挂在 **`Wagon`**
（运行时对象）上，**不要加到 `WagonConfig`**（那是只读配置；Q1 的归属规则：
域数据归车厢、运行期派生归编组）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # 只为类型标注，避免与将来的 wagon.py → plan.py 形成环
    from model.rail_network import RailNetwork


#: 终点：`(edge_id, t, direction)`——与 `TrainEntity.goal` 同形，`direction ∈ {+1, -1}`。
Goal = tuple[int, float, int]

#: 有向边：`(edge_id, direction)`，direction ∈ {+1, -1}。
DirectedEdge = tuple[int, int]

#: 目标端头：`+1` = 前端车钩，`-1` = 后端车钩。
END_FRONT = 1
END_REAR = -1


class PlanCommand(Enum):
    """计划条目的命令（无跳转命令，循环靠计划自然回绕）。"""

    GOTO = "goto"                # 前往：走到目标点并停稳（自然步进）
    GOTO_COUPLE = "goto_couple"  # 前往连挂：开到指定车厢的指定端头并完成连挂（事件步进）
    WAIT_COUPLE = "wait_couple"  # 等待连挂：原地等待别的列车来连挂（只由事件推进）
    DECOUPLE = "decouple"        # 解挂：从逻辑车头后第 n 位切开（动作完成即步进）
    REVERSE = "reverse"          # 折返：在同一 simple_segment 内切换逻辑方向


#: 命令的中文名（GUI 展示 / 失效提示用；Q23-3 要求失效条目"可见 + 带原因"）。
COMMAND_LABELS: dict[PlanCommand, str] = {
    PlanCommand.GOTO: "前往",
    PlanCommand.GOTO_COUPLE: "前往连挂",
    PlanCommand.WAIT_COUPLE: "等待连挂",
    PlanCommand.DECOUPLE: "解挂",
    PlanCommand.REVERSE: "折返",
}


@dataclass(frozen=True)
class FixedRoute:
    """确认后的固定行车路径（P3a）。

    `edges` 是从构造起点到终点的完整有向边序列；`start_offset` / `end_offset`
    沿用 `ResolvedPath` 的弧长语义。它是计划执行的权威路径：P3b 只能投影/消费它，
    不能重新寻路。终点仍属于带逻辑的 `PlanItem`，以避免维护两份可漂移的数据。
    """

    edges: tuple[DirectedEdge, ...]
    start_offset: float
    end_offset: float

    def used_edge_ids(self) -> tuple[int, ...]:
        return tuple(edge_id for edge_id, _direction in self.edges)

    def remaining_to_goal(self, network: RailNetwork) -> float:
        total = sum(network.edges[edge_id].length for edge_id, _ in self.edges)
        return max(0.0, total - self.start_offset - self.end_offset)

    def conflicts_with_signal(self, signal: DirectedEdge) -> bool:
        """该单向信号是否会从背面封死固定路线的一段。"""
        edge_id, direction = signal
        return (edge_id, -direction) in self.edges

    def validate(
        self, network: RailNetwork | None = None, *, goal: Goal | None = None,
    ) -> list[str]:
        """校验固定路径结构、拓扑连续性与可选终点的一致性。"""
        problems: list[str] = []
        if not self.edges:
            return ["固定路径为空"]
        if self.start_offset < 0.0 or self.end_offset < 0.0:
            problems.append("固定路径偏移不得为负")

        for i, (edge_id, direction) in enumerate(self.edges):
            if direction not in (1, -1):
                problems.append(f"固定路径第 {i} 条边 direction={direction} 非法（应为 ±1）")
            if network is not None and edge_id not in network.edges:
                problems.append(f"固定路径第 {i} 条边 {edge_id} 不存在（引用永久失效）")

        if network is None or problems:
            return problems

        first_length = network.edges[self.edges[0][0]].length
        last_length = network.edges[self.edges[-1][0]].length
        total_length = sum(network.edges[edge_id].length for edge_id, _ in self.edges)
        if self.start_offset > first_length:
            problems.append("固定路径起点偏移超出首边长度")
        if self.end_offset > last_length:
            problems.append("固定路径终点偏移超出末边长度")
        if self.start_offset + self.end_offset > total_length:
            problems.append("固定路径起终点偏移超过总长度")

        for i, (current, following) in enumerate(zip(self.edges, self.edges[1:])):
            current_edge_id, current_direction = current
            following_edge_id, following_direction = following
            current_edge = network.edges[current_edge_id]
            following_edge = network.edges[following_edge_id]
            head = (current_edge.node_b_id if current_direction > 0
                    else current_edge.node_a_id)
            tail = (following_edge.node_a_id if following_direction > 0
                    else following_edge.node_b_id)
            if head != tail:
                problems.append(f"固定路径第 {i}→{i + 1} 条边不连续")
                continue
            if current_edge_id == following_edge_id and current_direction == -following_direction:
                if network.nodes[head].connection_count() != 1:
                    problems.append(f"固定路径第 {i}→{i + 1} 条边在非死端折返")
            elif not network.turn_allowed(head, current_edge_id, following_edge_id):
                problems.append(f"固定路径第 {i}→{i + 1} 条边转向不许可")

        if goal is not None:
            goal_edge_id, _goal_t, goal_direction = goal
            if self.edges[-1] != (goal_edge_id, goal_direction):
                problems.append("固定路径末边与条目终点边/方向不一致")
        return problems


@dataclass(frozen=True)
class Anchor:
    """路径限定里的一个**硬约束途经点**（Q22-3：必须按序经过）。

    - `node_id`：**图上已有的节点**（道岔或二度节点）——因此解析**永不切边**。
    - `exit_edge_id`：给定时为**控制点**——"经过该节点时**从这条边离开**"
      （≈ 现实里的进路 / 道岔定反位，Q22-5）；为 `None` 时只要求经过该节点。
    """

    node_id: int
    exit_edge_id: int | None = None

    @property
    def is_control_point(self) -> bool:
        """是否指定了出口（= 控制点形态）。"""
        return self.exit_edge_id is not None


@dataclass(frozen=True)
class TrainRef:
    """连挂目标（Q23-7 demo 最简形态：指向具体车厢的端头，**不做全图搜索**）。

    用**目标车厢的 `wagon_id`** 而不是"列车对象引用"来指认目标：车厢是稳定主体，
    连挂/解挂只重组编组、**不销毁车厢**（Q1/Q16），因此解析时可以用"该车厢现在
    在哪列车上"重新定位目标；目标车厢本身不存在了才算**永久失效**（Q23-2）。
    """

    wagon_id: str
    end: int = END_FRONT

    def validate(self) -> list[str]:
        problems: list[str] = []
        if not self.wagon_id:
            problems.append("连挂目标缺少 wagon_id")
        if self.end not in (END_FRONT, END_REAR):
            problems.append(f"连挂目标端头非法：{self.end}（应为 {END_FRONT} 或 {END_REAR}）")
        return problems


@dataclass(frozen=True)
class CoupleSelector:
    """声明式连挂目标（P7c 首版：固定 edge + 执行列车驶入方向）。"""

    edge_id: int
    direction: int

    def validate(self, network: RailNetwork | None = None) -> list[str]:
        if self.direction not in (1, -1):
            return [f"固定连挂 edge direction 非法：{self.direction}（应为 ±1）"]
        if self.edge_id < 0:
            return [f"固定连挂边 id 非法：{self.edge_id}"]
        if network is not None and self.edge_id not in network.edges:
            return [f"固定连挂边 {self.edge_id} 不存在（引用永久失效）"]
        return []


@dataclass(frozen=True)
class PlanItem:
    """计划里的一条条目：命令、编辑元数据与确认后的固定路线。"""

    command: PlanCommand
    goal: Goal | None = None
    train_ref: TrainRef | None = None
    couple_selector: CoupleSelector | None = None
    decouple_after: int | None = None
    anchors: tuple[Anchor, ...] = ()
    fixed_route: FixedRoute | None = None

    # ── 构造捷径（让调用点自解释）───────────────────────────────────────
    @classmethod
    def goto(
        cls,
        goal: Goal,
        anchors: tuple[Anchor, ...] = (),
        fixed_route: FixedRoute | None = None,
    ) -> PlanItem:
        return cls(
            command=PlanCommand.GOTO,
            goal=goal,
            anchors=anchors,
            fixed_route=fixed_route,
        )

    @classmethod
    def goto_couple(
        cls,
        train_ref: TrainRef | None = None,
        anchors: tuple[Anchor, ...] = (),
        fixed_route: FixedRoute | None = None,
        *,
        edge_id: int | None = None,
        direction: int | None = None,
    ) -> PlanItem:
        return cls(
            command=PlanCommand.GOTO_COUPLE,
            train_ref=train_ref,
            couple_selector=(
                CoupleSelector(edge_id, direction if direction is not None else 0)
                if edge_id is not None else None
            ),
            anchors=anchors,
            fixed_route=fixed_route,
        )

    @classmethod
    def wait_couple(cls) -> PlanItem:
        return cls(command=PlanCommand.WAIT_COUPLE)

    @classmethod
    def decouple(cls, after: int) -> PlanItem:
        return cls(command=PlanCommand.DECOUPLE, decouple_after=after)

    @classmethod
    def reverse(cls) -> PlanItem:
        return cls(command=PlanCommand.REVERSE)

    @property
    def label(self) -> str:
        return COMMAND_LABELS.get(self.command, self.command.value)

    @property
    def display_label(self) -> str:
        """面向玩家的简短描述；selector 必须显式展示 edge 与方向。"""
        if self.command is PlanCommand.GOTO_COUPLE and self.couple_selector is not None:
            selector = self.couple_selector
            return f"{self.label} edge {selector.edge_id} dir {selector.direction:+d}"
        return self.label

    def validate(
        self,
        network: RailNetwork | None = None,
        *,
        require_fixed_route: bool = False,
    ) -> list[str]:
        """条目自洽 + 引用存在性检查，返回问题描述列表（空 = 通过）。

        给出 `network` 时额外检查**引用是否仍然存在**（节点/边/控制点出口）——
        这正是失效语义里"**永久失效 = 引用对象不存在**"的判据（Q23-2）。
        """
        problems: list[str] = []
        label = self.label

        # ① 命令与载荷必须匹配（三种命令各有确定的载荷形态）
        if self.command is PlanCommand.GOTO:
            if self.goal is None:
                problems.append(f"{label}：缺少终点 goal")
            if self.train_ref is not None:
                problems.append(f"{label}：不应带连挂目标 train_ref")
            if self.couple_selector is not None:
                problems.append(f"{label}：不应带连挂 selector")
        elif self.command is PlanCommand.GOTO_COUPLE:
            if self.train_ref is None and self.couple_selector is None:
                problems.append(f"{label}：缺少连挂目标（固定 edge 或兼容车厢引用）")
            if self.train_ref is not None and self.couple_selector is not None:
                problems.append(f"{label}：不能同时指定车厢和固定 edge")
            if self.goal is not None:
                problems.append(f"{label}：不应带终点 goal（目标由运行时端头给出）")
        elif self.command is PlanCommand.WAIT_COUPLE:
            if (self.goal is not None or self.train_ref is not None
                    or self.couple_selector is not None):
                problems.append(f"{label}：不应带终点或连挂目标")
            if self.anchors:
                problems.append(f"{label}：不应带路径限定（等待条目不产生路径）")
            if self.fixed_route is not None:
                problems.append(f"{label}：不应带固定路径")
        elif self.command is PlanCommand.DECOUPLE:
            if self.decouple_after is None or self.decouple_after < 1:
                problems.append(f"{label}：车头后解挂位次必须 >= 1")
            if (self.goal is not None or self.train_ref is not None
                    or self.couple_selector is not None or self.anchors
                    or self.fixed_route is not None):
                problems.append(f"{label}：不应带路径或连挂目标")
        elif self.command is PlanCommand.REVERSE:
            if (self.goal is not None or self.train_ref is not None
                    or self.couple_selector is not None or self.decouple_after is not None
                    or self.anchors or self.fixed_route is not None):
                problems.append(f"{label}：不应带其它载荷")

        if require_fixed_route and self.command in (
            PlanCommand.GOTO, PlanCommand.GOTO_COUPLE,
        ) and self.fixed_route is None:
            problems.append(f"{label}：尚未冻结固定路径")

        # ② 终点三元组自洽
        if self.goal is not None:
            edge_id, t, direction = self.goal
            if not (0.0 <= t <= 1.0):
                problems.append(f"{label}：终点 t={t} 越界（应在 [0, 1]）")
            if direction not in (1, -1):
                problems.append(f"{label}：终点 direction={direction} 非法（应为 ±1）")
            if network is not None and edge_id not in network.edges:
                problems.append(f"{label}：终点边 {edge_id} 不存在（引用永久失效）")

        # ③ 连挂目标自洽
        if self.train_ref is not None:
            problems.extend(f"{label}：{p}" for p in self.train_ref.validate())
        if self.couple_selector is not None:
            problems.extend(
                f"{label}：{p}" for p in self.couple_selector.validate(network)
            )

        # ④ 固定路径。P3a 允许未确认的草稿继续通过普通 validate；P3b 会传
        # require_fixed_route=True，确保执行链路没有回退到 Dijkstra 的机会。
        if self.fixed_route is not None:
            route_goal = self.goal if self.command is PlanCommand.GOTO else None
            problems.extend(
                f"{label}：{p}" for p in self.fixed_route.validate(network, goal=route_goal)
            )
            if (self.command is PlanCommand.GOTO_COUPLE
                    and self.couple_selector is not None
                    and self.fixed_route.edges
                    and self.fixed_route.edges[-1] != (
                        self.couple_selector.edge_id, self.couple_selector.direction
                    )):
                problems.append(f"{label}：固定路径末段不是 selector 指定的 edge + direction")

        # ⑤ 路径限定（硬约束锚点）自洽 + 引用存在性
        for i, anchor in enumerate(self.anchors):
            if i > 0 and anchor.node_id == self.anchors[i - 1].node_id:
                problems.append(
                    f"{label}：第 {i} 个锚点与上一个重复（node {anchor.node_id}）"
                )
            if network is None:
                continue
            node = network.nodes.get(anchor.node_id)
            if node is None:
                problems.append(
                    f"{label}：锚点 {i} 的节点 {anchor.node_id} 不存在（引用永久失效）"
                )
                continue
            exit_edge_id = anchor.exit_edge_id
            if exit_edge_id is None:
                continue
            if exit_edge_id not in network.edges:
                problems.append(
                    f"{label}：锚点 {i} 的出口边 {exit_edge_id} 不存在（引用永久失效）"
                )
                continue
            if exit_edge_id not in node.incident_edge_ids:
                problems.append(
                    f"{label}：锚点 {i} 的出口边 {exit_edge_id} 不接在节点 "
                    f"{anchor.node_id} 上"
                )
                continue
            # 出口必须真的能被某条入射边转到（否则是"死出口"⇒ 硬约束必然解析失败）
            enterable = [
                e for e in sorted(node.incident_edge_ids)
                if e != exit_edge_id and network.turn_allowed(anchor.node_id, e, exit_edge_id)
            ]
            if not enterable:
                problems.append(
                    f"{label}：锚点 {i} 的出口边 {exit_edge_id} 在节点 "
                    f"{anchor.node_id} 上是死出口（没有任何入射边能转到它）"
                )

        return problems


@dataclass
class Plan:
    """有序条目 + 指令指针（指针落在控制车上；Q14/Q15）。

    空计划 = **列车停车等待**（Q15）。计划可循环，也保留兼容的一次性模式；后者
    走完后指针停在尾后（``current() is None``）。`advance()`
    只做指针移动；**步进策略不在这里**（roadmap §2.3 / P3）。
    """

    items: list[PlanItem] = field(default_factory=list)
    pointer: int = 0
    # 第一条条目的 fixed_route 是“建表位置 → 第一目标”的首次启动路线；
    # 首次走完整份计划后，必须改用“末目标 → 第一目标”的循环接缝。
    loop_route: FixedRoute | None = None
    has_wrapped: bool = False
    requires_closed_cycle: bool = False
    repeat: bool = True

    # ── 指针 ──────────────────────────────────────────────────────────
    def __len__(self) -> int:
        return len(self.items)

    @property
    def is_empty(self) -> bool:
        return not self.items

    def current(self) -> PlanItem | None:
        """当前条目；空计划返回 None（⇒ 停车等待）。"""
        if not self.items:
            return None
        pointer = self._clamped_pointer()
        if not self.repeat and pointer == len(self.items):
            return None
        return self.items[pointer]

    @property
    def is_complete(self) -> bool:
        """一次性计划是否已消费完全部条目。"""
        return not self.repeat and bool(self.items) and self.pointer == len(self.items)

    def move_to(self, index: int) -> None:
        """把指针移到指定条目（越界抛 IndexError；空计划只允许 0）。"""
        if not self.items:
            if index != 0:
                raise IndexError(f"空计划只能把指针放在 0，收到 {index}")
            self.pointer = 0
            self.has_wrapped = False
            return
        if not (0 <= index < len(self.items)):
            raise IndexError(f"指针越界：{index}（共 {len(self.items)} 条）")
        self.pointer = index
        self.has_wrapped = False

    def advance(self) -> int:
        """指针前进一条；循环计划回绕，一次性计划停在尾后，返回新指针。

        ⚠ 只移动指针：不判断命令类型、不触发任何动作、不做"整圈无可执行命令"
        的兜底——那些属于步进策略（roadmap §2.3，**P3 实现**；其中
        "**一次到达事件最多走一圈**"是硬性正确性要求）。
        """
        if not self.items:
            self.pointer = 0
            self.has_wrapped = False
            return self.pointer
        old_pointer = self._clamped_pointer()
        if old_pointer == len(self.items) - 1:
            if self.repeat:
                self.pointer = 0
                self.has_wrapped = True
            else:
                self.pointer = len(self.items)
                self.has_wrapped = False
        else:
            self.pointer = old_pointer + 1
        return self.pointer

    def _clamped_pointer(self) -> int:
        if not self.items:
            return 0
        maximum = len(self.items) if not self.repeat else len(self.items) - 1
        if not (0 <= self.pointer <= maximum):
            self.pointer = min(max(self.pointer, 0), maximum)
        return self.pointer

    # ── 编辑（Q23-5：仅停放列车可编辑；这里只提供纯数据操作）──────────
    def append(self, item: PlanItem) -> None:
        self.items.append(item)
        self.loop_route = None
        self.has_wrapped = False
        self._clamped_pointer()

    def insert(self, index: int, item: PlanItem) -> None:
        """在 `index` 处插入；**插入点若在指针之前，指针后移一位**，以保持
        "当前条目"不变。"""
        index = max(0, min(index, len(self.items)))
        if self.items and index < self.pointer:
            self.pointer += 1
        self.items.insert(index, item)
        self.loop_route = None
        self.has_wrapped = False
        self._clamped_pointer()

    def remove_at(self, index: int) -> PlanItem:
        """删除一条；指针调整规则：删指针之前的条目 ⇒ 指针 -1；
        删的正是当前条目 ⇒ 指针停在原位（即原来的下一条）；删空 ⇒ 归 0。"""
        if not (0 <= index < len(self.items)):
            raise IndexError(f"删除越界：{index}（共 {len(self.items)} 条）")
        if index < self.pointer:
            self.pointer -= 1
        removed = self.items.pop(index)
        self.loop_route = None
        self.has_wrapped = False
        if not self.items:
            self.pointer = 0
        else:
            maximum = len(self.items) if not self.repeat else len(self.items) - 1
            self.pointer = min(max(self.pointer, 0), maximum)
        return removed

    # ── 校验 ──────────────────────────────────────────────────────────
    def validate(
        self,
        network: RailNetwork | None = None,
        *,
        require_fixed_route: bool = False,
    ) -> list[str]:
        """整份计划自查（指针范围 + 逐条 validate），返回问题描述列表。"""
        problems: list[str] = []
        if self.items:
            maximum = len(self.items) if not self.repeat else len(self.items) - 1
            if not (0 <= self.pointer <= maximum):
                problems.append(
                    f"指针 {self.pointer} 越界（共 {len(self.items)} 条）"
                )
        elif self.pointer != 0:
            problems.append(f"空计划的指针必须为 0，实际 {self.pointer}")
        for i, item in enumerate(self.items):
            problems.extend(
                f"第 {i} 条：{p}"
                for p in item.validate(network, require_fixed_route=require_fixed_route)
            )
        if self.loop_route is not None:
            first_goal = self.items[0].goal if self.items else None
            problems.extend(
                f"循环接缝：{p}"
                for p in self.loop_route.validate(network, goal=first_goal)
            )
        elif require_fixed_route and self.has_wrapped and self.items:
            problems.append("循环接缝：计划已经回绕但尚未冻结末目标到第一目标的路径")
        return problems

    def validate_cycle(self, network: RailNetwork | None = None) -> list[str]:
        """校验纯 goto 计划的 n→1 固定路线；它是计划完整性条件。"""
        if not self.requires_closed_cycle:
            return []
        if not self.items or any(item.command is not PlanCommand.GOTO for item in self.items):
            return []
        first = self.items[0]
        last = self.items[-1]
        if first.goal is None or last.goal is None:
            return ["普通前往计划的首条或末条缺少终点，无法闭环"]
        if self.loop_route is None:
            return ["普通前往计划缺少末条→第一条的固定闭环路径"]
        problems = self.loop_route.validate(network, goal=first.goal)
        if network is not None and not problems:
            start_edge_id, start_t, start_direction = last.goal
            start_edge = network.edges.get(start_edge_id)
            if start_edge is None:
                problems.append("末条终点边不存在，无法校验闭环起点")
                return [f"循环接缝：{problem}" for problem in problems]
            expected_directed = (start_edge_id, start_direction)
            if self.loop_route.edges[0] != expected_directed:
                problems.append(
                    "闭环路径首边与末条终点边/方向不一致"
                )
            else:
                edge_length = start_edge.length
                expected_offset = (
                    start_t * edge_length if start_direction > 0
                    else (1.0 - start_t) * edge_length
                )
                if abs(self.loop_route.start_offset - expected_offset) > 1e-6:
                    problems.append("闭环路径起点偏移与末条终点不一致")
        return [f"循环接缝：{problem}" for problem in problems]
