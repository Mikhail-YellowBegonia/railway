"""A0.4 train-control ownership contract."""
from model.train_control import TrainControlAuthority, authority_label


assert TrainControlAuthority.PLAN.value == "plan"
assert authority_label(TrainControlAuthority.MANUAL_TAKEOVER) == "临时手动接管"
assert set(TrainControlAuthority) == {
    TrainControlAuthority.NONE,
    TrainControlAuthority.PLAN,
    TrainControlAuthority.PLAN_PAUSED,
    TrainControlAuthority.MANUAL_TAKEOVER,
}
print("✅ 列车控制权：计划/暂停/临时接管/无指令四态定义通过")
