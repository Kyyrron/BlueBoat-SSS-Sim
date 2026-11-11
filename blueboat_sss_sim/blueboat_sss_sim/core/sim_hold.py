"""Hold the Gazebo simulation while a slow sonar ping renders (2026-11-09).

Why: the ray-cast sensor costs 0.06-0.15 s per ping-side in open water but
1-16 s with a quay in range (the 2026-11-09 harbour session lost 501 of its
2 212 pings at x0.10 that way). A fixed slow-down sized for those spikes
would waste ~10x the time over the open water. Instead the sonar node lets
the simulation run at its real-time factor and **pauses Gazebo only while a
render would overrun the next ping tick**: every ping is kept at its 50 ms
sim-time period, and the run costs ``max(budget, render)`` of wall time per
ping. The interface is unchanged (one ``/clock``, the same topics); a held
run is a slowed run whose clock rate varies.

Modes (``RenderHold.mode``):

* ``when_needed`` -- a timer armed just before the next tick, at the wall
  budget (``ping period / real-time factor``) minus a margin
  (``max(margin_s, 0.1 x budget)``), pauses the world if the render is still
  running when it fires; the world resumes as soon as the render ends. A
  ping therefore costs ``render + margin`` when the render overruns
  (2026-11-10; it was ``render + 0.4 x budget`` with the old 0.6 arm point).
* ``always`` -- pause before every render, resume after (deterministic,
  slower: the render never overlaps the simulation).
* ``off`` -- render as before.

The world is always resumed in a ``finally``; a pause request that fails
three times in a row turns the hold off (logged), so a missing service never
stalls the sonar. ROS-free and Qt-free; the gz-transport bindings are
imported only by :class:`GzWorldControl`, so tests drive :class:`RenderHold`
with a fake control.
"""

from __future__ import annotations

import threading
from typing import Callable, Optional

#: Accepted values of the node's ``hold_sim_while_rendering`` parameter.
HOLD_MODES = ("auto", "when_needed", "always", "off")


def resolve_hold_mode(mode: str, renderer_kind: str) -> str:
    """The effective mode: ``auto`` holds for the ray-cast only (the
    geometric renderer keeps its 50 ms period and its byte-identical path)."""
    m = str(mode or "auto").strip().lower()
    if m not in HOLD_MODES:
        raise ValueError(f"hold_sim_while_rendering {mode!r} not in {HOLD_MODES}")
    if m == "auto":
        return "when_needed" if renderer_kind == "raycast" else "off"
    return m


class GzWorldControl:
    """Pause / resume one Gazebo world through gz-transport
    (``/world/<name>/control``, ``gz.msgs.WorldControl`` -> ``Boolean``)."""

    def __init__(self, world_name: str, timeout_ms: int = 1000) -> None:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.transport13 import Node
        self.world_name = str(world_name)
        self.service = f"/world/{self.world_name}/control"
        self._node = Node()
        self._req_t, self._rep_t = WorldControl, Boolean
        self._timeout_ms = int(timeout_ms)

    def available(self) -> bool:
        """The world's control service is advertised (Gazebo is running)."""
        try:
            return self.service in set(self._node.service_list())
        except Exception:  # noqa: BLE001 -- treated as absent
            return False

    def set_paused(self, paused: bool) -> bool:
        req = self._req_t()
        req.pause = bool(paused)
        ok, rep = self._node.request(self.service, req, self._req_t, self._rep_t,
                                     self._timeout_ms)
        return bool(ok) and bool(getattr(rep, "data", True))


class RenderHold:
    """Runs a render with the simulation held per ``mode`` (module doc)."""

    #: consecutive failed pause requests that turn the hold off
    MAX_FAILURES = 3

    def __init__(self, control, mode: str, budget_s: float,
                 margin_s: float = 0.005,
                 log: Optional[Callable[[str], None]] = None,
                 timer_factory=threading.Timer) -> None:
        if mode not in ("when_needed", "always", "off"):
            raise ValueError(f"RenderHold mode {mode!r}")
        self.control = control
        self.mode = mode
        self.budget_s = max(float(budget_s), 1e-3)
        self.margin_s = max(float(margin_s), 0.0)
        self._log = log or (lambda _m: None)
        self._timer_factory = timer_factory
        self._lock = threading.Lock()
        self._paused = False
        self._failures = 0
        #: renders during which the world was held / total renders
        self.holds = 0
        self.renders = 0
        #: whether the last render held the world
        self.last_held = False

    @property
    def arm_delay_s(self) -> float:
        """Wall seconds after a render starts at which ``when_needed``
        pauses: the budget minus ``max(margin_s, 0.1 x budget)``."""
        return max(self.budget_s - max(self.margin_s, 0.1 * self.budget_s), 0.0)

    @property
    def active(self) -> bool:
        return self.mode != "off" and self.control is not None

    # --------------------------------------------------------------- core
    def _pause_locked(self) -> None:
        if self._paused or not self.active:
            return
        try:
            ok = self.control.set_paused(True)
        except Exception as e:  # noqa: BLE001
            ok = False
            self._log(f"pause request failed ({type(e).__name__}: {e})")
        if ok:
            self._paused = True
            self._failures = 0
            return
        self._failures += 1
        if self._failures >= self.MAX_FAILURES:
            self.mode = "off"
            self._log(f"{self.MAX_FAILURES} pause requests failed in a row: holding "
                      "the simulation is OFF (pings may fall behind the sim clock)")

    def _world_gone(self) -> bool:
        """The world's control service is no longer advertised (Gazebo torn
        down with the launch): nothing is left to resume."""
        available = getattr(self.control, "available", None)
        if not callable(available):
            return False
        try:
            return not available()
        except Exception:  # noqa: BLE001
            return False

    def _resume_locked(self) -> None:
        if not self._paused:
            return
        errors = []
        for _attempt in (1, 2):
            try:
                if self.control.set_paused(False):
                    self._paused = False
                    return
            except Exception as e:  # noqa: BLE001
                errors.append(f"{type(e).__name__}: {e}")
        self._paused = False
        if self._world_gone():
            return          # a launch teardown took Gazebo with it: nothing to resume
        self._log("could not resume the simulation" +
                  (f" ({errors[-1]})" if errors else "") +
                  ": it may still be paused (resume it from Gazebo, or stop the launch)")

    def resume(self) -> None:
        """Resume the world if this hold paused it (shutdown path)."""
        with self._lock:
            self._resume_locked()

    def run(self, render: Callable[[], object]):
        """``render()`` with the simulation held per the mode; its result."""
        self.renders += 1
        self.last_held = False
        if not self.active:
            return render()
        if self.mode == "always":
            with self._lock:
                self._pause_locked()
            try:
                return render()
            finally:
                with self._lock:
                    self.last_held = self._paused
                    self.holds += int(self._paused)
                    self._resume_locked()
        done = threading.Event()

        def fire() -> None:
            with self._lock:
                if not done.is_set():
                    self._pause_locked()

        timer = self._timer_factory(self.arm_delay_s, fire)
        timer.daemon = True
        timer.start()
        try:
            return render()
        finally:
            with self._lock:
                done.set()
            timer.cancel()
            with self._lock:
                self.last_held = self._paused
                self.holds += int(self._paused)
                self._resume_locked()
