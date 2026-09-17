"""调度计划数据类型回归 —— 计划层 P1（`docs/plan_layer_roadmap.md` §2）。

P1 是**纯数据类型、无消费点**：本测试只验证数据自洽（命令与载荷匹配、终点/锚点/
控制点合法性、引用存在性检查 = "永久失效"判据）、不可变性、以及 `Plan` 的指针与
编辑语义（插入/删除时如何保持"当前条目"）。

覆盖：
① 三种命令 + 锚点（含控制点）构造后 validate 通过（无 network / 有 network 两种）；
② 命令与载荷不匹配的 6 种非法组合逐个被标记；
③ 终点三元组越界（t ∉ [0,1]、direction ≠ ±1、边不存在）；
④ `TrainRef` 自洽性；
⑤ 不可变性（frozen dataclass）+ `anchors` 是 tuple；
⑥ `Plan` 指针：空计划 = 停车等待、`advance` 回绕、`move_to` 越界、
   插入/删除时指针保持"当前条目"；
⑦ **引用存在性 = 永久失效判据**：节点/边不存在、出口边不接在该节点上、
   出口是"死出口"（没有任何入射边能转到它）；
⑧ `Plan.validate` 聚合逐条问题（带"第 N 条："前缀）并标记脏指针。

自建图，不依赖 `manual_track.geojson`（仓库既有约定：避免用户存档漂移导致测试失效）。

图：道岔 t(0,0)，东臂 t-m-a（共线两段）、西臂 t-c、北臂 t-b。
    e_tm: t→m，e_ma: m→a，e_tc: t→c，e_tb: t→b。
    因此"经过 t 并从 e_tc 离开"是可行的（从 e_tm 进来西行 → 继续西行 e_tc，dot=1）。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from model.plan import (
    COMMAND_LABELS,
    END_FRONT,
    END_REAR,
    Anchor,
    FixedRoute,
    Plan,
    PlanCommand,
    PlanItem,
    TrainRef,
)
from model.rail_network import RailNetwork
from model.vec3 import Vec3

net = RailNetwork()
t = net.add_node(Vec3(0.0, 0.0, 0.0))
m = net.add_node(Vec3(100.0, 0.0, 0.0))
a = net.add_node(Vec3(200.0, 0.0, 0.0))
c = net.add_node(Vec3(-100.0, 0.0, 0.0))
b = net.add_node(Vec3(0.0, 100.0, 0.0))
EDGE_TM = net.add_edge(t, m).edge_id
EDGE_MA = net.add_edge(m, a).edge_id
EDGE_TC = net.add_edge(t, c).edge_id
EDGE_TB = net.add_edge(t, b).edge_id

# ── ① 合法条目（含锚点与控制点）────────────────────────────────────────
anchors = (Anchor(m.node_id), Anchor(t.node_id, EDGE_TC))
item_goto = PlanItem.goto((EDGE_MA, 0.5, 1), anchors)
item_couple = PlanItem.goto_couple(TrainRef(wagon_id="w-42", end=END_FRONT))
item_wait = PlanItem.wait_couple()

assert item_goto.validate() == [], item_goto.validate()
assert item_couple.validate() == [], item_couple.validate()
assert item_wait.validate() == [], item_wait.validate()
assert item_goto.validate(net) == [], item_goto.validate(net)
assert item_goto.label == "前往" and item_wait.label == "等待连挂"
assert COMMAND_LABELS[PlanCommand.GOTO_COUPLE] == "前往连挂"
assert not Anchor(m.node_id).is_control_point
assert Anchor(t.node_id, EDGE_TC).is_control_point
print("✅ ① 三种命令条目 + 锚点（含控制点）自洽；给出 network 时引用检查通过")

# 控制点的出口可达性也确认一下（"从 e_tm 进来 → 从 e_tc 出去"许可）
assert net.turn_allowed(t.node_id, EDGE_TM, EDGE_TC), "测试图前提：该出口应可达"

# ── ② 命令与载荷不匹配 ──────────────────────────────────────────────────
cases = [
    (PlanItem(command=PlanCommand.GOTO), "缺少终点 goal"),
    (
        PlanItem(command=PlanCommand.GOTO, goal=(EDGE_TM, 0.0, 1), train_ref=TrainRef("w1")),
        "不应带连挂目标",
    ),
    (PlanItem(command=PlanCommand.GOTO_COUPLE), "缺少连挂目标"),
    (
        PlanItem(command=PlanCommand.GOTO_COUPLE, train_ref=TrainRef("w1"), goal=(EDGE_TM, 0.0, 1)),
        "不应带终点",
    ),
    (PlanItem(command=PlanCommand.WAIT_COUPLE, goal=(EDGE_TM, 0.0, 1)), "不应带终点或连挂目标"),
    (PlanItem(command=PlanCommand.WAIT_COUPLE, anchors=(Anchor(1),)), "不应带路径限定"),
]
for item, expect in cases:
    problems = item.validate()
    assert any(expect in p for p in problems), f"应标记「{expect}」，实际 {problems}"
print("✅ ② 命令与载荷不匹配的 6 种非法组合逐个被标记")

# ── ③ 终点三元组越界 / 边不存在 ─────────────────────────────────────────
assert any("越界" in p for p in PlanItem.goto((EDGE_TM, 1.5, 1)).validate())
assert any("越界" in p for p in PlanItem.goto((EDGE_TM, -0.1, 1)).validate())
assert any("direction" in p for p in PlanItem.goto((EDGE_TM, 0.5, 0)).validate())
assert any("不存在" in p for p in PlanItem.goto((99999, 0.5, 1)).validate(net))
print("✅ ③ 终点 t / direction 越界、终点边不存在（永久失效）被标记")

# ── ④ TrainRef 自洽 ────────────────────────────────────────────────────
ref_problems = TrainRef(wagon_id="", end=0).validate()
assert any("wagon_id" in p for p in ref_problems), ref_problems
assert any("端头" in p for p in ref_problems), ref_problems
assert TrainRef("w", END_REAR).validate() == []
assert TrainRef("w", END_FRONT).validate() == []
print("✅ ④ TrainRef：缺少 wagon_id / 端头非法被标记，±1 端头通过")

# ── ⑤ 不可变 ────────────────────────────────────────────────────────────
assert isinstance(item_goto.anchors, tuple), "anchors 必须是 tuple（不可变）"
for target, attr in (
    (Anchor(1), "node_id"),
    (item_goto, "command"),
    (TrainRef("w"), "wagon_id"),
):
    try:
        setattr(target, attr, None)
    except AttributeError as exc:  # dataclasses.FrozenInstanceError ⊂ AttributeError
        assert "frozen" in str(exc).lower() or True
    else:
        raise AssertionError(f"{type(target).__name__}.{attr} 竟然可写（frozen 失效）")
print("✅ ⑤ Anchor / PlanItem / TrainRef 均为 frozen，`anchors` 为 tuple")

# ── ⑥ Plan 指针与编辑语义 ───────────────────────────────────────────────
plan = Plan()
assert plan.is_empty and plan.current() is None, "空计划 ⇒ current() 为 None（停车等待）"
assert plan.advance() == 0, "空计划 advance 应保持 0"
assert plan.validate() == []

plan.append(PlanItem.goto((EDGE_TM, 0.0, 1)))   # 0
plan.append(PlanItem.wait_couple())             # 1
plan.append(PlanItem.goto((EDGE_TB, 1.0, 1)))   # 2
assert plan.pointer == 0 and plan.current() is plan.items[0]
assert (plan.advance(), plan.advance()) == (1, 2)
assert plan.advance() == 0, "走完必须回绕到第一项（Q15）"
one_shot = Plan([PlanItem.wait_couple()], repeat=False)
assert one_shot.advance() == 1 and one_shot.current() is None
assert one_shot.is_complete and one_shot.validate() == []
plan.move_to(2)
assert plan.pointer == 2 and plan.current() is plan.items[2]
for bad in (3, -1):
    try:
        plan.move_to(bad)
    except IndexError:
        pass
    else:
        raise AssertionError(f"move_to({bad}) 应抛 IndexError")
empty_plan = Plan()
try:
    empty_plan.move_to(1)
except IndexError:
    pass
else:
    raise AssertionError("空计划 move_to(1) 应抛 IndexError")

# 插入点若在指针之前 ⇒ 指针后移，保持"当前条目"不变
plan.move_to(2)
current_before = plan.current()
plan.insert(0, PlanItem.goto((EDGE_TC, 0.5, -1)))
assert plan.pointer == 3, f"插入点在前 ⇒ 指针应后移到 3，实际 {plan.pointer}"
assert plan.current() is current_before, "插入后当前条目必须不变"
# 插入点若在指针之后 ⇒ 指针不动
plan.move_to(1)
current_before = plan.current()
plan.insert(3, PlanItem.wait_couple())
assert plan.pointer == 1 and plan.current() is current_before, "插入点在后 ⇒ 指针不动"
# 删除指针之前的条目 ⇒ 指针 -1，当前条目不变
plan.remove_at(0)
assert plan.pointer == 0 and plan.current() is current_before
# 删除当前条目 ⇒ 指针停在原位（= 原来的下一条）
plan.move_to(0)
nxt = plan.items[1]
plan.remove_at(0)
assert plan.current() is nxt, "删除当前条目后指针应停在原位（即下一条）"
# 删空 ⇒ 指针归 0、current() 为 None
while plan.items:
    plan.remove_at(0)
assert plan.pointer == 0 and plan.current() is None and plan.validate() == []
try:
    plan.remove_at(0)
except IndexError:
    pass
else:
    raise AssertionError("空计划 remove_at(0) 应抛 IndexError")
print("✅ ⑥ Plan：空计划语义 / 循环与一次性指针 / move_to 越界 / 插入删除保持当前条目")

# ── ⑦ 引用存在性 = 永久失效判据 ─────────────────────────────────────────
ghost_node = PlanItem.goto((EDGE_TM, 0.0, 1), (Anchor(424242),))
assert any("不存在" in p for p in ghost_node.validate(net)), ghost_node.validate(net)

ghost_exit = PlanItem.goto((EDGE_TM, 0.0, 1), (Anchor(t.node_id, 99999),))
assert any("不存在" in p for p in ghost_exit.validate(net)), ghost_exit.validate(net)

wrong_node = PlanItem.goto((EDGE_TM, 0.0, 1), (Anchor(t.node_id, EDGE_MA),))
assert any("不接在节点" in p for p in wrong_node.validate(net)), wrong_node.validate(net)

# 死出口：二度急折节点处，出口边没有任何入射边能转过去（dot = 0 → 不许可）
net3 = RailNetwork()
k0 = net3.add_node(Vec3(0.0, 0.0, 0.0))
k1 = net3.add_node(Vec3(100.0, 0.0, 0.0))
k2 = net3.add_node(Vec3(100.0, 10.0, 0.0))
g1 = net3.add_edge(k0, k1)
g2 = net3.add_edge(k1, k2)
dead = PlanItem.goto((g1.edge_id, 0.5, 1), (Anchor(k1.node_id, g2.edge_id),))
assert any("死出口" in p for p in dead.validate(net3)), dead.validate(net3)
print("✅ ⑦ 永久失效判据：节点/边不存在、出口不接在该节点上、死出口 全部被标记")

# ── ⑧ 整份计划校验聚合 ──────────────────────────────────────────────────
bad_plan = Plan(items=[PlanItem.goto((EDGE_TM, 2.0, 1)), PlanItem(command=PlanCommand.GOTO)])
problems = bad_plan.validate(net)
assert any(p.startswith("第 0 条：") for p in problems), problems
assert any(p.startswith("第 1 条：") for p in problems), problems
dirty = Plan(items=[PlanItem.wait_couple()])
dirty.pointer = 5
assert any("指针" in p for p in dirty.validate()), dirty.validate()
assert Plan().validate() == []
print("✅ ⑧ 整份计划校验聚合并带条目序号；脏指针被标记")

# ── ⑨ 玩家循环计划必须显式包含 n→1 路径 ─────────────────────────────
open_cycle = Plan(
    [PlanItem.goto((EDGE_TM, 0.0, 1), fixed_route=FixedRoute(((EDGE_TM, 1),), 0.0, 0.0))],
    requires_closed_cycle=True,
)
assert any("缺少末条→第一条" in p for p in open_cycle.validate_cycle(net))
wrong_seam = Plan(
    [PlanItem.goto((EDGE_TM, 0.5, -1), fixed_route=FixedRoute(((EDGE_TM, -1),), 0.0, 0.0))],
    loop_route=FixedRoute(((EDGE_MA, -1), (EDGE_TM, -1)), 0.0, 0.0),
    requires_closed_cycle=True,
)
assert any("首边与末条终点" in p for p in wrong_seam.validate_cycle(net))
print("✅ ⑨ 声明为循环的计划缺少 n→1 路径时明确判为不完整")

print("\n全部通过")
