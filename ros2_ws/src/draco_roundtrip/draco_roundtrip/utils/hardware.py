"""Helpers for sizing worker pools based on host CPU characteristics."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class WorkerPlan:
    workers: int
    mode: str
    total_cpu: int
    reserved: int
    cap: Optional[int]


def recommended_worker_count(requested: int,
                             reserve_ratio: float = 0.25,
                             min_reserve: int = 1,
                             cap: Optional[int] = None,
                             env_prefix: str = "DRACO") -> WorkerPlan:
    """Determine an appropriate worker count given host CPU availability.

    Parameters
    ----------
    requested:
        Explicit worker count supplied via CLI. Values > 0 are honored.
    reserve_ratio:
        Fraction of total CPUs to keep free when auto-sizing.
    min_reserve:
        Minimum number of CPUs to keep free.
    cap:
        Optional hard upper bound for the auto-sized worker count.
    env_prefix:
        Prefix for environment overrides. e.g. DRACO_WORKER_RESERVE, DRACO_WORKER_CAP.

    Returns
    -------
    WorkerPlan
        Structured summary containing the selected worker count.
    """
    total = os.cpu_count() or 1

    # Explicit override from CLI wins.
    if requested > 0:
        return WorkerPlan(workers=requested,
                          mode="cli",
                          total_cpu=total,
                          reserved=max(total - requested, 0),
                          cap=cap)

    # Environment overrides.
    env_reserve = os.environ.get(f"{env_prefix}_WORKER_RESERVE")
    env_cap = os.environ.get(f"{env_prefix}_WORKER_CAP")
    reserve = min_reserve
    if env_reserve:
        try:
            reserve = max(0, int(env_reserve))
        except ValueError:
            reserve = min_reserve
    else:
        reserve = max(min_reserve, int(total * reserve_ratio))

    if reserve >= total:
        reserve = max(total - 1, 0)

    workers = max(1, total - reserve)

    hard_cap = cap
    if env_cap:
        try:
            hard_cap = max(1, int(env_cap))
        except ValueError:
            pass

    if hard_cap is not None:
        workers = min(workers, hard_cap)

    return WorkerPlan(workers=workers,
                      mode="auto",
                      total_cpu=total,
                      reserved=reserve,
                      cap=hard_cap)
