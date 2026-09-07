"""BrakingController 回归测试：制动闩锁 + 连续制动曲线停车。

背景（见 model/train_controller.py 顶部注释，三轮方案演进记录）：

1. safety_margin > 1 会让 "remaining <= d_stop" 判据在全力制动过程中
   双向翻转（d_stop 比 remaining 掉得更快），若不加闩锁会持续震荡直到
   停车前；闩锁之后车会在 remaining 还有余量时就已经速度归零。

2. 早期实现把"速度降到某个阈值以下"直接当成"到达终点"，但
   safety_margin 高估了制动距离，车通常会在离目标还有几米时就已经
   减速到阈值——早期逻辑会在这里直接判定到达，导致车停在目标前方一段
   距离（提前停车 bug，用户实测截图报告）。曾经改成"降到阈值后转入
   固定低速爬行，直到 remaining 真正归零"，但这个固定低速本身在高速
   巡航场景下要爬很久（20m/s 巡航时爬行段能到 22m，要爬 44 秒），
   体感是"过早减速+缓慢爬行"，用户第二次实测报告确认这个问题。

3. 最终方案：连续制动曲线（真实 ATO 做法）——目标速度不是固定值，是
   `v_brake_target = sqrt(2·a_brake·remaining/margin)`，随剩余距离
   平滑衰减到 0，制动阶段用 bang-bang 硬控制追这条曲线（不用 PID，
   PID 追动态曲线会有响应滞后，实测直接冲过终点）。这个方案同时消除
   了前两轮的问题：不存在固定阈值可以双向翻转，也不存在"先砸到固定
   低速再匀速爬"的两段式手感。

**关于 brake_toggles 的判据**：连续曲线用 bang-bang 控制，逐帧判断
"v 是否高于当前这一帧的目标曲线值"，在离散时间步下必然会有高频的
throttle/brake 切换（车速贴着连续下降的目标曲线来回跨越，每次跨越
都会切一次输出）——这是良性的微调，车辆加速度本身没有跳变（固定的
a_max/a_brake），不会造成体感顿挫。这跟早期版本"完全跳出制动模式回
巡航、全力回充油门、再重新判定进制动"的**恶性宏观震荡**（`_braking`
标志本身反复翻转）性质不同，不能用同一个计数器区分。本测试改为
统计 `_braking` 标志的翻转次数（进入制动后应该只翻一次，不会退回
巡航再重新进入），而不是统计 throttle/brake 输出的翻转次数。
"""
from model.train_controller import BrakingController
from model.train_physics import SimplePhysics
from model.wagon import create_simple_wagon, Consist

physics = SimplePhysics(a_max=1.0, b_max=2.0, v_max=30.0)
consist = Consist(wagons=[create_simple_wagon(length=20.0, mass=50.0)])
dt = 1.0 / 60.0


def simulate(v_cruise: float, total_length: float, max_steps: int = 20000):
    ctrl = BrakingController(physics, consist)
    v, s = 0.0, 0.0
    braking_flag_toggles = 0
    prev_braking = None
    for _ in range(max_steps):
        throttle, brake = ctrl.update(v, s, total_length, v_cruise, dt)
        # 到达终点那一帧 update() 会把 _braking 主动清回 False 作为收尾
        # 复位（不是"退回巡航重新判定"的震荡，是停车动作本身的一部分），
        # 不计入统计——真正要抓的是"制动途中"发生的反复翻转。
        if not ctrl.stopped:
            if prev_braking is not None and ctrl._braking != prev_braking:
                braking_flag_toggles += 1
            prev_braking = ctrl._braking
        a = physics.compute_acceleration(v, throttle, brake, consist)
        v = max(0.0, v + a * dt)
        s += v * dt
        if ctrl.stopped:
            return ctrl, v, s, braking_flag_toggles
    return ctrl, v, s, braking_flag_toggles


for v_cruise in (5.0, 10.0, 15.0, 20.0):
    d_stop_est = (v_cruise ** 2) / (2 * physics.b_max) * 1.3
    total_length = d_stop_est * 1.5 + 50.0

    ctrl, v_final, s_final, braking_toggles = simulate(v_cruise, total_length)

    assert ctrl.stopped, f"v_cruise={v_cruise}: 未能在步数上限内停车"
    remaining_final = total_length - s_final
    assert remaining_final <= BrakingController.STOP_EPSILON + 1e-6, \
        (f"v_cruise={v_cruise}: 停车时距目标还有 {remaining_final:.4f}m，"
         f"超出 STOP_EPSILON——这正是要修复的提前停车 bug")
    # _braking 标志只应从 False 翻到 True 一次（进入制动曲线阶段后不会
    # 退回巡航），不允许反复翻转（回归前实测 v_cruise=15 时反复震荡
    # 161 次）。
    assert braking_toggles <= 1, \
        (f"v_cruise={v_cruise}: _braking 标志翻转 {braking_toggles} 次，"
         f"怀疑制动区宏观震荡回归（不是良性的 bang-bang 高频微调）")

print("✅ BrakingController 连续制动曲线停车回归测试通过")
