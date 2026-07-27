#!/usr/bin/env python3
"""Tests for frozen N-1 cone gaps and return ordering."""

from __future__ import annotations

import math
import unittest

try:
    from tools.perception import reference_tracker
except ImportError:
    reference_tracker = None


class TestReferenceGap(unittest.TestCase):
    def setUp(self) -> None:
        self.assertIsNotNone(reference_tracker, "reference_tracker must be available")

    def test_lock_map_creates_adjacent_gaps_and_return_is_far_to_near(self) -> None:
        tracks = [
            reference_tracker.Track(11, 0.6, 0.4, confirmed=True),
            reference_tracker.Track(12, 1.8, -0.2, confirmed=True),
            reference_tracker.Track(13, 3.1, 0.3, confirmed=True),
        ]
        gap_map = reference_tracker.lock_gap_map(tracks, (0.0, 0.0), (4.0, 0.0))
        self.assertEqual(len(gap_map.gaps), 2)
        self.assertEqual([gap.left_cone_id for gap in gap_map.return_gaps], [12, 11])
        first = gap_map.return_gaps[0]
        self.assertAlmostEqual(first.approach_x_m, first.center_x_m + 0.6)
        self.assertAlmostEqual(first.exit_x_m, first.center_x_m - 0.6)
        self.assertAlmostEqual(first.heading_rad, math.pi, places=5)

    def test_state_machine_freezes_only_when_two_confirmed_cones_exist(self) -> None:
        state = reference_tracker.PerceptionStateMachine()
        state.step(task_start=True)
        self.assertEqual(state.state, "OUTBOUND_MAP")
        state.step(turnaround=True, confirmed_cones=1)
        self.assertEqual(state.state, "TURN_LOCK")
        state.step(confirmed_cones=1)
        self.assertEqual(state.state, "SAFE_STOP")


if __name__ == "__main__":
    unittest.main()
