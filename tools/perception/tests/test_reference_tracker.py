#!/usr/bin/env python3
"""Tests for static cone association and lifecycle."""

from __future__ import annotations

import unittest

try:
    from tools.perception import reference_tracker
except ImportError:
    reference_tracker = None


class TestReferenceTracker(unittest.TestCase):
    def setUp(self) -> None:
        self.assertIsNotNone(reference_tracker, "reference_tracker must be available")
        self.tracker = reference_tracker.StaticConeTracker(max_tracks=2)

    @staticmethod
    def observation(x: float, y: float, score: int = 200, far: bool = False):
        return reference_tracker.Observation(x, y, 0.01, 0.0, 0.01, score, far)

    def test_three_hits_in_five_frames_confirms_stable_id(self) -> None:
        for _ in range(3):
            self.tracker.step([self.observation(1.0, 0.2)])
        track = self.tracker.tracks[0]
        self.assertTrue(track.confirmed)
        self.assertEqual(track.cone_id, 1)

    def test_far_target_requires_four_hits_in_six_frames(self) -> None:
        for _ in range(3):
            self.tracker.step([self.observation(5.0, 0.2, far=True)])
        self.assertFalse(self.tracker.tracks[0].confirmed)
        self.tracker.step([self.observation(5.0, 0.2, far=True)])
        self.assertTrue(self.tracker.tracks[0].confirmed)

    def test_mahalanobis_gate_rejects_far_measurement(self) -> None:
        self.tracker.step([self.observation(1.0, 0.0)])
        self.tracker.step([self.observation(2.0, 0.0)])
        self.assertEqual(len(self.tracker.tracks), 2)

    def test_prediction_holds_eight_frames_then_deletes_at_twenty(self) -> None:
        self.tracker.step([self.observation(1.0, 0.0)])
        for _ in range(8):
            self.tracker.step([])
        self.assertTrue(self.tracker.tracks[0].prediction_only)
        for _ in range(12):
            self.tracker.step([])
        self.assertEqual(self.tracker.tracks, [])

    def test_overlapping_tracks_merge_to_lower_id(self) -> None:
        for _ in range(3):
            self.tracker.step([self.observation(1.0, 0.0), self.observation(1.3, 0.0)])
        self.assertEqual(len(self.tracker.tracks), 2)
        self.tracker.tracks[1].x_m = 1.05
        self.tracker.merge_overlaps(0.15)
        self.assertEqual(len(self.tracker.tracks), 1)
        self.assertEqual(self.tracker.tracks[0].cone_id, 1)


if __name__ == "__main__":
    unittest.main()
