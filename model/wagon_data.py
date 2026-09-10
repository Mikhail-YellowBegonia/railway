from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import uuid


@dataclass
class WagonDataPacket:
    """车厢数据包：**归属并托管于车厢自身**（A 桶域数据）。

    内容（payload）目前为空，未来可存放调度命令、车厢状态标记等。
    排序键（order_key）默认按创建时间戳，供车厢内多包排序。

    设计约束（2026-09-10 定，见 docs/wagon_centric_data.md §6.3 / T2-2）：
    - `wagon_id` 不可变，与物理车厢 1:1 绑定
    - 数据包**随车厢走**：解挂/连挂不需要任何"归并/拆分"记账——车厢是同一批
      对象（别名共享），它自己的包自然跟着它
    """
    wagon_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    order_key: float = field(default_factory=lambda: __import__('time').monotonic())
    payload: dict[str, Any] = field(default_factory=dict)


class WagonDataLog:
    """**单节车厢**持有的数据包列表（A 桶；不是编组级容器）。

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

    def __len__(self) -> int:
        return len(self._packets)

    def __repr__(self) -> str:
        return f"WagonDataLog({len(self._packets)} packets)"
