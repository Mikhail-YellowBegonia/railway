"""Opt-in route diagnostics. No routing or occupancy decisions are changed here."""
from __future__ import annotations

from contextvars import ContextVar
from datetime import datetime, timezone
from functools import wraps
from inspect import signature
import itertools
import json
from pathlib import Path
import tempfile


_enabled = False
_stream = None
_log_path: Path | None = None
_ids = itertools.count(1)
_parent = ContextVar("route_debug_parent", default=None)


def is_enabled() -> bool:
    return _enabled


def set_enabled(enabled: bool, *, log_path: Path | None = None) -> Path | None:
    """Toggle tracing; repeated toggles append to the same session file."""
    global _enabled, _stream, _log_path
    if _stream is not None:
        _stream.close()
        _stream = None
    _enabled = False
    if log_path is not None:
        _log_path = Path(log_path)
    if enabled:
        if _log_path is None:
            _log_path = Path(tempfile.mkdtemp(prefix="railway-route-debug-")) / "routes.jsonl"
        _stream = _log_path.open("a", encoding="utf-8")
        _enabled = True
        emit("session", log_path=str(_log_path), direction="+1 = node_a -> node_b")
    return _log_path


def format_edges(edges) -> str:
    return " -> ".join(f"E{eid}{'+' if direction > 0 else '-'}" for eid, direction in edges) or "[]"


def sequence_details(network, edges) -> dict:
    """Separate repeated edge occurrences, reversal pairs and disconnected joins."""
    edges = list(edges)
    occurrences = {}
    for index, (eid, _direction) in enumerate(edges):
        occurrences.setdefault(eid, []).append(index + 1)
    gaps = []
    reversals = []
    for index, (left, right) in enumerate(zip(edges, edges[1:])):
        a, b = network.edges.get(left[0]), network.edges.get(right[0])
        if a is None or b is None:
            continue
        arrival = a.node_b_id if left[1] > 0 else a.node_a_id
        departure = b.node_a_id if right[1] > 0 else b.node_b_id
        if arrival != departure:
            gaps.append({"after": index + 1, "arrival_node": arrival, "departure_node": departure})
        if left[0] == right[0] and left[1] == -right[1]:
            reversals.append(index + 1)
    return {
        "edges": edges,
        "sequence": format_edges(edges),
        "repeated_edges": {str(eid): positions for eid, positions in occurrences.items() if len(positions) > 1},
        "reversal_after": reversals,
        "disconnected_after": gaps,
    }


def emit(kind: str, **data) -> None:
    global _enabled
    if not _enabled:
        return
    record = {"id": next(_ids), "time": datetime.now(timezone.utc).isoformat(),
              "parent": _parent.get(), "kind": kind, **data}
    line = json.dumps(record, ensure_ascii=False)
    # Flush every record: a crash or an overflowing terminal must not lose evidence.
    try:
        _stream.write(line + "\n")
        _stream.flush()
    except OSError as exc:
        _enabled = False
        print(f"[ROUTE_DEBUG] 日志写入失败，已关闭记录：{exc}")
        return
    print("[ROUTE_DEBUG] " + line)


def _input_value(name, value):
    if name in ("network", "cost_fn", "passable_fn"):
        return None
    if name == "item":
        return {"command": value.command.value, "goal": value.goal,
                "anchors": [{"node_id": a.node_id, "exit_edge_id": a.exit_edge_id} for a in value.anchors],
                "path_policy": value.effective_path_policy.value}
    if name == "start" and hasattr(value, "edge_id"):
        return {"edge_id": value.edge_id, "t": value.t, "direction": value.direction}
    if value is None or isinstance(value, (str, int, float, bool, list, tuple)):
        return value
    return str(value)


def trace_search(function):
    """Trace all returns, including same-edge shortcuts and failed searches.

    Nested Dijkstra candidates and the assembled anchor route have parent ids,
    so a candidate is never confused with the final plan resolution.
    """
    sig = signature(function)

    @wraps(function)
    def wrapped(*args, **kwargs):
        if not _enabled:
            return function(*args, **kwargs)
        bound = sig.bind(*args, **kwargs)
        bound.apply_defaults()
        values = bound.arguments
        network = values["network"]
        query_id = next(_ids)
        emit("search.begin", query=query_id, source=function.__module__ + "." + function.__name__,
             inputs={name: _input_value(name, value) for name, value in values.items()
                     if name not in ("network", "cost_fn", "passable_fn")})
        token = _parent.set(query_id)
        try:
            result = function(*args, **kwargs)
            path = result
            details = {}
            if isinstance(result, tuple):
                path = result[0]
                details.update(start_offset=result[1], end_offset=result[2])
            elif hasattr(result, "path"):
                path = result.path
                details.update(failure=result.failure, no_path=result.is_no_path)
            if path is not None:
                details.update(sequence_details(network, path.edges), total_cost=path.total_cost)
                if hasattr(path, "start_offset"):
                    details.update(start_offset=path.start_offset, end_offset=path.end_offset)
            emit("search.result", query=query_id, found=path is not None, **details)
            return result
        except Exception as exc:
            emit("search.error", query=query_id, error=repr(exc))
            raise
        finally:
            _parent.reset(token)
    return wrapped


def train_snapshot(train) -> dict:
    occ = train.state.occupancy
    try:
        edge_id, t, direction = train.current_directed_edge_and_t()
        actual_head = {"edge": (edge_id, direction), "t": t}
    except (ValueError, IndexError, KeyError) as exc:
        actual_head = {"error": repr(exc)}
    winner = train.state.consist.control_winner()
    plan = winner.plan if winner is not None else None
    item = plan.current() if plan is not None else None
    return {
        "train": [w.wagon_id for w in train.state.consist.wagons],
        "actual_head": actual_head,
        "occupied": sequence_details(train.network, occ.occupied),
        "occupied_offset": occ.occupied_offset, "s": occ.s, "abs_s": train.state.abs_s,
        "route": sequence_details(train.network, occ.route),
        "goal": train.state.goal, "remaining": train.state.remaining_to_goal,
        "plan_pointer": plan.pointer if plan is not None else None,
        "plan_command": item.command.value if item is not None else None,
    }


def trace_train_change(function):
    """Snapshots at route assignment, coupling and explicit/automatic reversal."""
    @wraps(function)
    def wrapped(train, *args, **kwargs):
        if not _enabled:
            return function(train, *args, **kwargs)
        before = train_snapshot(train)
        partner = (train_snapshot(args[0] if args else kwargs["rear"])
                   if function.__name__ == "couple_with" else None)
        result = function(train, *args, **kwargs)
        after_train = result if function.__name__ == "couple_with" else train
        emit("train." + function.__name__, before=before, partner=partner,
             after=train_snapshot(after_train) if after_train is not None else None)
        return result
    return wrapped
