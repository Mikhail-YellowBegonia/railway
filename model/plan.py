"""调度计划的数据类型（计划层 P1：**纯数据类型，无消费点**）。

规格：`docs/plan_layer_roadmap.md` §2（Q23-1 定稿）。本模块**只定义数据结构与
自洽校验**，不产生任何行为、不被任何运行链路引用（P1 阶段的纪律；消费点从 P3 起
才接）。

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
- **demo 命令集合 = `goto` / `goto_couple` / `wait_couple`，无跳转命令**（Q23-8）：
  循环只靠"走完回绕第一项"。
- **引用一律落在物理要素上**（Q22-2）：锚点 → `node_id`、控制点 → `edge_id`、
  终点 → `(edge_id, t)`；连挂目标 → **目标车厢的 `wagon_id`**（见 `TrainRef`：
  车厢是稳定主体，连挂/解挂只重组编组、不销毁车厢，故它比"列车对象引用"稳）。
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

#: 目标端头：`+1` = 前端车钩，`-1` = 后端车钩。
END_FRONT = 1
END_REAR = -1


class PlanCommand(Enum):
    """计划条目的命令（demo 三条，**无跳转命令**，Q23-7/Q23-8）。"""

    GOTO = "goto"                # 前往：走到目标点并停稳（自然步进）
    GOTO_COUPLE = "goto_couple"  # 前往连挂：开到指定车厢的指定端头并完成连挂（事件步进）
    WAIT_COUPLE = "wait_couple"  # 等待连挂：原地等待别的列车来连挂（只由事件推进）


#: 命令的中文名（GUI 展示 / 失效提示用；Q23-3 要求失效条目"可见 + 带原因"）。
COMMAND_LABELS: dict[PlanCommand, str] = {
    PlanCommand.GOTO: "前往",
    PlanCommand.GOTO_COUPLE: "前往连挂",
    PlanCommand.WAIT_COUPLE: "等待连挂",
}


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
class PlanItem:
    """计划里的一条条目：**命令（逻辑）+ 可选路径限定（锚点）**。"""

    command: PlanCommand
    goal: Goal | None = None
    train_ref: TrainRef | None = None
    anchors: tuple[Anchor, ...] = ()

    # ── 构造捷径（让调用点自解释）───────────────────────────────────────
    @classmethod
    def goto(cls, goal: Goal, anchors: tuple[Anchor, ...] = ()) -> PlanItem:
        return cls(command=PlanCommand.GOTO, goal=goal, anchors=anchors)

    @classmethod
    def goto_couple(
        cls, train_ref: TrainRef, anchors: tuple[Anchor, ...] = ()
    ) -> PlanItem:
        return cls(
            command=PlanCommand.GOTO_COUPLE, train_ref=train_ref, anchors=anchors
        )

    @classmethod
    def wait_couple(cls) -> PlanItem:
        return cls(command=PlanCommand.WAIT_COUPLE)

    @property
    def label(self) -> str:
        return COMMAND_LABELS.get(self.command, self.command.value)

    def validate(self, network: RailNetwork | None = None) -> list[str]:
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
        elif self.command is PlanCommand.GOTO_COUPLE:
            if self.train_ref is None:
                problems.append(f"{label}：缺少连挂目标 train_ref")
            if self.goal is not None:
                problems.append(f"{label}：不应带终点 goal（目标由 train_ref 给出）")
        elif self.command is PlanCommand.WAIT_COUPLE:
            if self.goal is not None or self.train_ref is not None:
                problems.append(f"{label}：不应带终点或连挂目标")
            if self.anchors:
                problems.append(f"{label}：不应带路径限定（等待条目不产生路径）")

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

        # ④ 路径限定（硬约束锚点）自洽 + 引用存在性
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

    空计划 = **列车停车等待**（Q15）。`advance()` 只做"指针前进 + 走完回绕到 0"
    这一个纯位移动作；**步进策略不在这里**（roadmap §2.3 / P3）。
    """

    items: list[PlanItem] = field(default_factory=list)
    pointer: int = 0

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
        return self.items[self._clamped_pointer()]

    def move_to(self, index: int) -> None:
        """把指针移到指定条目（越界抛 IndexError；空计划只允许 0）。"""
        if not self.items:
            if index != 0:
                raise IndexError(f"空计划只能把指针放在 0，收到 {index}")
            self.pointer = 0
            return
        if not (0 <= index < len(self.items)):
            raise IndexError(f"指针越界：{index}（共 {len(self.items)} 条）")
        self.pointer = index

    def advance(self) -> int:
        """指针前进一条，**走完回绕到第一项**（Q15），返回新指针。

        ⚠ 只移动指针：不判断命令类型、不触发任何动作、不做"整圈无可执行命令"
        的兜底——那些属于步进策略（roadmap §2.3，**P3 实现**；其中
        "**一次到达事件最多走一圈**"是硬性正确性要求）。
        """
        if not self.items:
            self.pointer = 0
            return self.pointer
        self.pointer = (self._clamped_pointer() + 1) % len(self.items)
        return self.pointer

    def _clamped_pointer(self) -> int:
        if not self.items:
            return 0
        if not (0 <= self.pointer < len(self.items)):
            self.pointer = min(max(self.pointer, 0), len(self.items) - 1)
        return self.pointer

    # ── 编辑（Q23-5：仅停放列车可编辑；这里只提供纯数据操作）──────────
    def append(self, item: PlanItem) -> None:
        self.items.append(item)
        self._clamped_pointer()

    def insert(self, index: int, item: PlanItem) -> None:
        """在 `index` 处插入；**插入点若在指针之前，指针后移一位**，以保持
        "当前条目"不变。"""
        index = max(0, min(index, len(self.items)))
        if self.items and index < self.pointer:
            self.pointer += 1
        self.items.insert(index, item)
        self._clamped_pointer()

    def remove_at(self, index: int) -> PlanItem:
        """删除一条；指针调整规则：删指针之前的条目 ⇒ 指针 -1；
        删的正是当前条目 ⇒ 指针停在原位（即原来的下一条）；删空 ⇒ 归 0。"""
        if not (0 <= index < len(self.items)):
            raise IndexError(f"删除越界：{index}（共 {len(self.items)} 条）")
        if index < self.pointer:
            self.pointer -= 1
        removed = self.items.pop(index)
        if not self.items:
            self.pointer = 0
        else:
            self.pointer = min(max(self.pointer, 0), len(self.items) - 1)
        return removed

    # ── 校验 ──────────────────────────────────────────────────────────
    def validate(self, network: RailNetwork | None = None) -> list[str]:
        """整份计划自查（指针范围 + 逐条 validate），返回问题描述列表。"""
        problems: list[str] = []
        if self.items:
            if not (0 <= self.pointer < len(self.items)):
                problems.append(
                    f"指针 {self.pointer} 越界（共 {len(self.items)} 条）"
                )
        elif self.pointer != 0:
            problems.append(f"空计划的指针必须为 0，实际 {self.pointer}")
        for i, item in enumerate(self.items):
            problems.extend(f"第 {i} 条：{p}" for p in item.validate(network))
        return problems
