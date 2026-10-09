"""A0.3 drag ownership regression."""
from controller.drag_state import DragOwner, DragSession


drag = DragSession()
assert drag.begin(DragOwner.SCREEN_WIDGET, "schedule.reorder", (10, 20))
assert not drag.begin(DragOwner.CAMERA, "camera.pan", (10, 20))
assert not drag.update(DragOwner.WORLD_TOOL, (30, 40))
assert drag.update(DragOwner.SCREEN_WIDGET, (30, 40))
assert drag.release(DragOwner.CAMERA, (50, 60)) is None
assert drag.release(DragOwner.SCREEN_WIDGET, (50, 60)) == "schedule.reorder"
assert not drag.active

assert drag.begin(DragOwner.CAMERA, "camera.pan", (0, 0))
assert not drag.cancel(DragOwner.SCREEN_WIDGET)
assert drag.cancel(DragOwner.CAMERA)
assert not drag.active

print("✅ 拖拽所有权：start/update/release/cancel 单一 owner 规则通过")
