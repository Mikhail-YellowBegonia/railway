"""P9-A3 consist builder operations, kept independent from pygame_gui."""
from controller.consist_builder import ConsistBuilder, WagonPreset


builder = ConsistBuilder.start("depot-1", "North Depot", 100.0)
assert len(builder.consist.wagons) == 1
assert builder.consist.wagons[0].have_control
assert builder.consist.wagons[0].is_powered

builder.select_preset(WagonPreset.COACH)
builder.add_wagon()
builder.add_wagon()
assert len(builder.consist.wagons) == 3
assert not builder.consist.wagons[-1].have_control
assert not builder.consist.wagons[-1].is_powered
assert builder.can_complete
print("✅ 增加车厢只使用动力控制车 / 普通车厢两种预设")

selected = builder.selected_wagon
assert selected is builder.consist.wagons[2]
assert builder.move_selected(-1)
assert builder.selected_wagon_index == 1
assert builder.consist.wagons[1] is selected
assert builder.move_selected(-1)
assert not builder.move_selected(-1)
assert builder.consist.wagons[0] is selected
print("✅ 点按箭头移动所选车厢，越界移动安全拒绝")

old_order = list(builder.consist.wagons)
old_direction = builder.direction
builder.reverse()
assert builder.consist.wagons == list(reversed(old_order))
assert builder.direction == -old_direction
assert all(wagon.orientation == -1 for wagon in builder.consist.wagons)
print("✅ 反转编组同步反转成员顺序、车厢逻辑朝向和 Depot 生成方向")

assert builder.delete_selected()
assert len(builder.consist.wagons) == 2
assert builder.delete_selected()
assert len(builder.consist.wagons) == 1
assert not builder.delete_selected()
print("✅ 删除车厢保留至少一节的编组下限")

short_depot = ConsistBuilder.start("short", "Short Depot", 19.0)
assert not short_depot.can_complete
print("✅ 编组超过 Depot 长度时完成条件关闭")

print("\n全部通过")
