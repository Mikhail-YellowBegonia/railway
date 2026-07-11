from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import uuid


@dataclass
class WagonDataPacket:
    """车厢数据包：与车厢 ID 绑定，托管于列车编组。

    内容（payload）目前为空，未来可存放调度命令、状态标记等。
    排序键（order_key）默认为创建时间戳序，列表融合/拆分时保留相对顺序。

    设计约束：
    - wagon_id 不可变，与物理车厢 1:1 绑定
    - 合并时两列表按 order_key 归并排序
    - 拆分时按 wagon_id 归属各自子列表
    """
    wagon_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    order_key: float = field(default_factory=lambda: __import__('time').monotonic())
    payload: dict[str, Any] = field(default_factory=dict)


class ConsistDataLog:
    """列车持有的数据包列表，维护顺序，支持合并与拆分。

    排序：按 order_key 升序（小的在前）。
    """

    def __init__(self, packets: list[WagonDataPacket] | None = None) -> None:
        self._packets: list[WagonDataPacket] = sorted(
            packets or [], key=lambda p: p.order_key
        )

    @property
    def packets(self) -> list[WagonDataPacket]:
        return list(self._packets)

    def add(self, packet: WagonDataPacket) -> None:
        """插入一个数据包，保持有序。"""
        from bisect import insort
        insort(self._packets, packet, key=lambda p: p.order_key)

    def merge(self, other: "ConsistDataLog") -> "ConsistDataLog":
        """合并两个日志，返回新的有序日志（不修改原对象）。"""
        merged = sorted(self._packets + other._packets, key=lambda p: p.order_key)
        return ConsistDataLog(merged)

    def split(self, front_ids: set[str]) -> tuple["ConsistDataLog", "ConsistDataLog"]:
        """按 wagon_id 集合拆分，返回 (前段日志, 后段日志)。

        不在 front_ids 中的包归入后段。顺序各自保留。
        """
        front = [p for p in self._packets if p.wagon_id in front_ids]
        rear  = [p for p in self._packets if p.wagon_id not in front_ids]
        return ConsistDataLog(front), ConsistDataLog(rear)

    def __len__(self) -> int:
        return len(self._packets)

    def __repr__(self) -> str:
        return f"ConsistDataLog({len(self._packets)} packets)"
