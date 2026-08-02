import json
from pathlib import Path
import tempfile
import unittest

from robot_agent.active_ir_scan_contract import ActiveIrRay, ActiveIrScanResult
from robot_agent.navigation_memory_store import (
    LEGACY_MEMORY_SCHEMA,
    MAX_MEMORY_BYTES,
    MEMORY_SCHEMA,
    NavigationMemoryStore,
)
from robot_agent.physical_scan_evidence import (
    MAX_COLLISION_SUPPORTS_PER_HAZARD,
    MAX_COLLISION_SUPPORTS_PER_MAP,
    BODY_RELATIVE_BEARING_CONVENTION,
    MAX_SCAN_ATTEMPTS_PER_HAZARD,
    MAX_SCAN_ATTEMPTS_PER_MAP,
    AngularCollisionSupport,
    ScanAttemptEvidence,
    ScanRayEvidence,
    retain_scan_attempt_diversity,
)
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.provisional_hazard_map import (
    HAZARD_SCAN_EVICTION,
    HAZARD_CAPACITY_EVICTION,
    MAP_SCAN_EVICTION,
    MAP_SUPPORT_EVICTION,
    MAX_HAZARDS_PER_MAP,
    PER_HAZARD_SCAN_EVICTION,
    ProvisionalHazard,
    ProvisionalHazardMap,
)


def attempt(index):
    rays = tuple(
        ScanRayEvidence(
            requested_relative_bearing_mdeg=(ordinal - 8) * 1_000,
            actual_relative_bearing_mdeg=(ordinal - 8) * 1_000,
            blocked=ordinal % 2 == 0,
            raw=50,
            filtered=50,
        )
        for ordinal in range(16)
    )
    return ScanAttemptEvidence(
        scan_id="scan-{:03d}".format(index),
        completed_at_ms=2_000 + index,
        status="CANCELLED",
        reason="bilateral_boundaries_not_observed",
        rays=rays,
        left_boundary_mdeg=None,
        right_boundary_mdeg=None,
    )


def hazard(index, history=()):
    return ProvisionalHazard(
        hypothesis_id="hazard-{:02d}".format(index),
        frame_id="frame-a",
        anchor_x_mm=0,
        anchor_y_mm=0,
        anchor_heading_mdeg=0,
        centroid_x_mm=140 + index,
        centroid_y_mm=0,
        radius_mm=70,
        first_seen_at_ms=1_000,
        last_seen_at_ms=1_000,
        evidence_count=1,
        last_state_version=index + 1,
        last_raw_ir_proximity=20,
        last_filtered_ir_proximity=20,
        scan_evidence_history=tuple(history),
    )


def support(index, *, scan_id=None, completed_at_ms=None):
    return AngularCollisionSupport(
        source_scan_id=scan_id or "support-scan-{:04d}".format(index),
        completed_at_ms=(
            5_000 + index
            if completed_at_ms is None
            else completed_at_ms
        ),
        pose_x_mm=index,
        pose_y_mm=index // 7,
        pose_heading_mdeg=(index % 360) * 1_000 - 180_000,
        actual_relative_bearing_mdeg=(index % 181) * 1_000 - 90_000,
        based_on_map_version=index,
    )


class PhysicalScanEvidenceTests(unittest.TestCase):
    @staticmethod
    def _record_scan(hazard_map, target, scan_id, completed_at_ms):
        based_on_map_version = hazard_map.revision
        hazard_map.record_observation(
            PhysicalPose(),
            {
                "state_version": based_on_map_version + 100,
                "infrared": {
                    "blocked": False,
                    "raw": 60,
                    "filtered": 60,
                },
            },
            completed_at_ms - 100,
        )
        result = ActiveIrScanResult(
            scan_id=scan_id,
            target_hypothesis_id=target,
            frame_id=hazard_map.frame_id,
            map_generation_id=hazard_map.map_generation_id,
            based_on_map_version=based_on_map_version,
            started_at_ms=completed_at_ms - 90,
            completed_at_ms=completed_at_ms,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            stop_confirmed=True,
            restored_start_heading=True,
            rays=(ActiveIrRay(
                ordinal=1,
                requested_relative_bearing_mdeg=0,
                actual_relative_bearing_mdeg=0,
                observed_at_ms=completed_at_ms - 10,
                state_version=based_on_map_version + 101,
                raw=20,
                filtered=20,
                blocked=True,
            ),),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
        )
        return hazard_map.record_scan_result(
            result,
            scan_pose=PhysicalPose(),
        )

    @staticmethod
    def _one_sided_attempt(scan_id, *, positive, completed_at_ms):
        if positive:
            bearings = ((10_000, True), (30_000, False))
            left_boundary = 20_000
            right_boundary = None
        else:
            bearings = ((-30_000, False), (-10_000, True))
            left_boundary = None
            right_boundary = -20_000
        return ScanAttemptEvidence(
            scan_id=scan_id,
            completed_at_ms=completed_at_ms,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            rays=tuple(
                ScanRayEvidence(
                    requested_relative_bearing_mdeg=bearing,
                    actual_relative_bearing_mdeg=bearing,
                    blocked=blocked,
                    raw=20 if blocked else 60,
                    filtered=20 if blocked else 60,
                )
                for bearing, blocked in bearings
            ),
            left_boundary_mdeg=left_boundary,
            right_boundary_mdeg=right_boundary,
            scan_pose=PhysicalPose(),
            based_on_map_version=1,
        )

    def test_complementary_attempts_unlock_only_their_verified_pose(self):
        positive = self._one_sided_attempt(
            "scan-positive",
            positive=True,
            completed_at_ms=2_000,
        )
        negative = self._one_sided_attempt(
            "scan-negative",
            positive=False,
            completed_at_ms=2_100,
        )
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=(hazard(0, (positive, negative)),),
        )

        ready = hazard_map.route_evidence(
            "hazard-00",
            pose=PhysicalPose(),
        )
        moved = hazard_map.route_evidence(
            "hazard-00",
            pose=PhysicalPose(x_mm=1),
        )

        self.assertTrue(ready["ready"])
        self.assertEqual(
            ready["reason"],
            "COMPLEMENTARY_BOUNDARIES_AT_CURRENT_POSE",
        )
        self.assertFalse(moved["ready"])
        self.assertEqual(
            moved["reason"],
            "NO_SCAN_EVIDENCE_AT_CURRENT_VERIFIED_POSE",
        )

    def test_duplicate_attempt_does_not_evict_unique_boundary_shape(self):
        positive = self._one_sided_attempt(
            "scan-positive",
            positive=True,
            completed_at_ms=2_000,
        )
        old_negative = self._one_sided_attempt(
            "scan-negative-old",
            positive=False,
            completed_at_ms=2_100,
        )
        new_negative = self._one_sided_attempt(
            "scan-negative-new",
            positive=False,
            completed_at_ms=2_200,
        )

        retained = retain_scan_attempt_diversity(
            (positive, old_negative, new_negative),
            limit=2,
        )

        self.assertEqual(
            [item.scan_id for item in retained],
            ["scan-positive", "scan-negative-new"],
        )

    def test_same_coarse_signature_at_different_verified_poses_survives(self):
        first = self._one_sided_attempt(
            "scan-pose-a",
            positive=True,
            completed_at_ms=2_000,
        )
        second = ScanAttemptEvidence(
            **{
                **first.__dict__,
                "scan_id": "scan-pose-b",
                "completed_at_ms": 2_100,
                "scan_pose": PhysicalPose(x_mm=1),
            }
        )

        retained = retain_scan_attempt_diversity(
            (first, second),
            limit=2,
        )

        self.assertEqual(
            [item.scan_id for item in retained],
            ["scan-pose-a", "scan-pose-b"],
        )
        self.assertEqual(
            first.observation_pattern,
            second.observation_pattern,
        )

    def test_ids_timestamps_and_raw_jitter_do_not_fake_scan_diversity(self):
        first = self._one_sided_attempt(
            "scan-old",
            positive=True,
            completed_at_ms=2_000,
        )
        jittered_rays = tuple(
            ScanRayEvidence(
                ray.requested_relative_bearing_mdeg,
                ray.actual_relative_bearing_mdeg,
                ray.blocked,
                21 if ray.raw == 20 else 61,
                21 if ray.filtered == 20 else 61,
            )
            for ray in first.rays
        )
        newer = ScanAttemptEvidence(
            **{
                **first.__dict__,
                "scan_id": "scan-new",
                "completed_at_ms": 2_100,
                "rays": jittered_rays,
                "based_on_map_version": 99,
            }
        )

        retained = retain_scan_attempt_diversity(
            (first, newer),
            limit=1,
        )

        self.assertEqual([item.scan_id for item in retained], ["scan-new"])

    def test_partial_all_clear_arc_does_not_erase_blocked_hypothesis(self):
        partial = ScanAttemptEvidence(
            scan_id="scan-partial-clear",
            completed_at_ms=3_000,
            status="CANCELLED",
            reason="scan_deadline_exceeded",
            rays=(
                ScanRayEvidence(0, 0, False, 60, 60),
                ScanRayEvidence(30_000, 29_000, False, 61, 60),
            ),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
        )

        self.assertEqual(partial.observation_pattern, "ALL_CLEAR")
        self.assertEqual(partial.arc_coverage, "POSITIVE_ARC_ONLY")
        self.assertEqual(partial.hypothesis_relation, "NO_EVIDENCE")

    def test_deadline_all_clear_bilateral_sample_is_not_full_conflict(self):
        timed_out = ScanAttemptEvidence(
            scan_id="scan-deadline-clear",
            completed_at_ms=3_000,
            status="CANCELLED",
            reason="scan_deadline_exceeded",
            rays=(
                ScanRayEvidence(-30_000, -30_000, False, 60, 60),
                ScanRayEvidence(0, 0, False, 60, 60),
                ScanRayEvidence(30_000, 30_000, False, 60, 60),
            ),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
        )

        self.assertEqual(timed_out.arc_coverage, "BILATERAL_ARC")
        self.assertEqual(timed_out.hypothesis_relation, "NO_EVIDENCE")

    def test_pruning_full_all_clear_detail_keeps_hypothesis_contested(self):
        conflict = ScanAttemptEvidence(
            scan_id="scan-full-clear",
            completed_at_ms=3_000,
            status="COMPLETED",
            reason="bilateral_boundaries_not_observed",
            rays=(
                ScanRayEvidence(-30_000, -30_000, False, 60, 60),
                ScanRayEvidence(30_000, 30_000, False, 60, 60),
            ),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
            scan_pose=PhysicalPose(),
            based_on_map_version=1,
            all_clear_arc_covers_target_hypothesis=True,
        )
        with_detail = hazard(0, (conflict,))
        pruned = ProvisionalHazard(
            **{
                **with_detail.__dict__,
                "scan_evidence_history": (),
            }
        )

        self.assertEqual(pruned.collision_contested_at_ms, 3_000)
        self.assertFalse(pruned.active_for_collision)

    def test_later_blocked_support_reactivates_contested_hypothesis(self):
        conflict = ScanAttemptEvidence(
            scan_id="scan-full-clear",
            completed_at_ms=3_000,
            status="COMPLETED",
            reason="bilateral_boundaries_not_observed",
            rays=(
                ScanRayEvidence(-30_000, -30_000, False, 60, 60),
                ScanRayEvidence(30_000, 30_000, False, 60, 60),
            ),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
            scan_pose=PhysicalPose(),
            based_on_map_version=1,
            all_clear_arc_covers_target_hypothesis=True,
        )
        blocked = self._one_sided_attempt(
            "scan-later-blocked",
            positive=True,
            completed_at_ms=4_000,
        )
        with_detail = hazard(0, (conflict, blocked))
        detail_pruned = ProvisionalHazard(
            **{
                **with_detail.__dict__,
                "scan_evidence_history": (),
            }
        )

        self.assertEqual(detail_pruned.last_seen_at_ms, 1_000)
        self.assertEqual(detail_pruned.collision_contested_at_ms, 3_000)
        self.assertEqual(
            max(
                item.completed_at_ms
                for item in detail_pruned.collision_supports
            ),
            4_000,
        )
        self.assertTrue(detail_pruned.active_for_collision)
        reloaded = ProvisionalHazard.from_dict(
            dict(detail_pruned.to_dict())
        )
        self.assertTrue(reloaded.active_for_collision)
        self.assertEqual(
            reloaded.collision_supports,
            detail_pruned.collision_supports,
        )

    def test_later_bilateral_boundaries_reactivate_contested_hypothesis(self):
        base = hazard(0)
        rescanned = ProvisionalHazard(
            **{
                **base.__dict__,
                "collision_contested_at_ms": 3_000,
                "scan_completed_at_ms": 4_000,
                "scan_left_boundary_mdeg": 30_000,
                "scan_right_boundary_mdeg": -30_000,
            }
        )

        self.assertEqual(rescanned.last_seen_at_ms, 1_000)
        self.assertEqual(rescanned.collision_supports, ())
        self.assertTrue(rescanned.bilateral_scan_complete)
        self.assertTrue(rescanned.active_for_collision)

    def test_scan_evidence_publishes_body_relative_bearing_convention(self):
        value = attempt(1).to_dict()

        self.assertEqual(
            value["bearing_convention"],
            BODY_RELATIVE_BEARING_CONVENTION,
        )
        self.assertEqual(ScanAttemptEvidence.from_dict(value), attempt(1))

    def test_scan_id_is_bounded_and_control_safe_for_persistence(self):
        source = attempt(1)
        accepted = ScanAttemptEvidence(
            **{
                **source.__dict__,
                "scan_id": "s" * 128,
            }
        )
        self.assertEqual(len(accepted.scan_id), 128)
        for invalid in ("s" * 129, "scan\ncontrol"):
            with self.subTest(invalid_length=len(invalid)):
                with self.assertRaisesRegex(ValueError, "scan attempt"):
                    ScanAttemptEvidence(
                        **{
                            **source.__dict__,
                            "scan_id": invalid,
                        }
                    )

    def test_legacy_persisted_hazard_loads_without_scan_history(self):
        value = dict(hazard(0).to_dict())
        for field in (
            "scan_evidence_history",
            "scan_attempts_evicted",
            "scan_attempts_eviction_reason",
            "bilateral_scan_complete",
            "collision_supports",
            "collision_supports_evicted",
            "collision_supports_eviction_reason",
            "collision_contested_at_ms",
        ):
            value.pop(field)

        loaded = ProvisionalHazard.from_dict(value)

        self.assertEqual(loaded.scan_evidence_history, ())
        self.assertFalse(loaded.bilateral_scan_complete)

    def test_legacy_scan_history_materializes_collision_supports(self):
        scanned = hazard(0, (
            self._one_sided_attempt(
                "legacy-blocked-scan",
                positive=True,
                completed_at_ms=2_000,
            ),
        ))
        value = dict(scanned.to_dict())
        for field in (
            "scan_attempts_evicted",
            "scan_attempts_eviction_reason",
            "collision_supports",
            "collision_supports_evicted",
            "collision_supports_eviction_reason",
            "collision_contested_at_ms",
        ):
            value.pop(field)

        loaded = ProvisionalHazard.from_dict(value)

        self.assertEqual(len(loaded.collision_supports), 1)
        retained = loaded.collision_supports[0]
        self.assertEqual(retained.source_scan_id, "legacy-blocked-scan")
        self.assertEqual(retained.actual_relative_bearing_mdeg, 10_000)

    def test_attempt_history_is_bounded_per_hazard_and_per_map(self):
        with self.assertRaisesRegex(ValueError, "scan history"):
            hazard(
                0,
                [
                    attempt(index)
                    for index in range(MAX_SCAN_ATTEMPTS_PER_HAZARD + 1)
                ],
            )
        histories = []
        attempt_index = 0
        hazard_count = (
            MAX_SCAN_ATTEMPTS_PER_MAP
            // MAX_SCAN_ATTEMPTS_PER_HAZARD
            + 1
        )
        for hazard_index in range(hazard_count):
            count = (
                MAX_SCAN_ATTEMPTS_PER_HAZARD
                if hazard_index < hazard_count - 1
                else 1
            )
            histories.append(tuple(
                attempt(attempt_index + offset)
                for offset in range(count)
            ))
            attempt_index += count
        with self.assertRaisesRegex(ValueError, "map bound"):
            ProvisionalHazardMap(
                frame_id="frame-a",
                map_generation_id="map-a",
                hazards=tuple(
                    hazard(index, history)
                    for index, history in enumerate(histories)
                ),
            )
        self.assertEqual(MAX_SCAN_ATTEMPTS_PER_HAZARD, 16)
        self.assertEqual(MAX_SCAN_ATTEMPTS_PER_MAP, 64)

    def test_per_hazard_scan_eviction_is_counted_and_persisted(self):
        target = hazard(
            0,
            tuple(
                attempt(index)
                for index in range(MAX_SCAN_ATTEMPTS_PER_HAZARD)
            ),
        )
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=(target,),
        )

        updated = self._record_scan(
            hazard_map,
            target.hypothesis_id,
            "scan-newest",
            10_000,
        )

        self.assertEqual(len(updated.scan_evidence_history), 16)
        self.assertEqual(updated.scan_attempts_evicted, 1)
        self.assertEqual(
            updated.scan_attempts_eviction_reason,
            PER_HAZARD_SCAN_EVICTION,
        )
        self.assertEqual(hazard_map.scan_attempt_retention(), {
            "per_hazard_capacity": MAX_SCAN_ATTEMPTS_PER_HAZARD,
            "map_capacity": MAX_SCAN_ATTEMPTS_PER_MAP,
            "retained_count": MAX_SCAN_ATTEMPTS_PER_HAZARD,
            "evicted_count": 1,
            "last_eviction_reason": PER_HAZARD_SCAN_EVICTION,
        })

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore(
                path=Path(directory) / "scan-retention-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                frame_id="frame-a",
                generation_id="map-a",
                pose=PhysicalPose(),
                hazard_map=hazard_map,
            )
            memory.save()
            reloaded = NavigationMemoryStore.load(
                path=memory.path,
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
            )

        reloaded_hazard = reloaded.hazard_map.get(target.hypothesis_id)
        self.assertEqual(reloaded_hazard.scan_attempts_evicted, 1)
        self.assertEqual(reloaded.hazard_map.scan_attempts_evicted, 1)
        self.assertEqual(
            reloaded.hazard_map.scan_attempts_eviction_reason,
            PER_HAZARD_SCAN_EVICTION,
        )

    def test_map_scan_eviction_is_counted_on_affected_hazard(self):
        full = tuple(
            hazard(
                index,
                tuple(
                    attempt(index * MAX_SCAN_ATTEMPTS_PER_HAZARD + offset)
                    for offset in range(MAX_SCAN_ATTEMPTS_PER_HAZARD)
                ),
            )
            for index in range(
                MAX_SCAN_ATTEMPTS_PER_MAP
                // MAX_SCAN_ATTEMPTS_PER_HAZARD
            )
        )
        target = hazard(len(full))
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=full + (target,),
        )

        updated = self._record_scan(
            hazard_map,
            target.hypothesis_id,
            "scan-map-protected",
            10_000,
        )

        self.assertEqual(
            sum(
                len(item.scan_evidence_history)
                for item in hazard_map.hazards
            ),
            MAX_SCAN_ATTEMPTS_PER_MAP,
        )
        self.assertEqual(updated.scan_evidence_history[0].scan_id,
                         "scan-map-protected")
        affected = [
            item for item in hazard_map.hazards
            if item.scan_attempts_evicted
        ]
        self.assertEqual(len(affected), 1)
        self.assertEqual(affected[0].scan_attempts_evicted, 1)
        self.assertEqual(
            affected[0].scan_attempts_eviction_reason,
            MAP_SCAN_EVICTION,
        )
        self.assertEqual(hazard_map.scan_attempts_evicted, 1)
        self.assertEqual(
            hazard_map.scan_attempts_eviction_reason,
            MAP_SCAN_EVICTION,
        )

    def test_hazard_eviction_counts_its_retained_scan_attempts(self):
        hazards = (
            hazard(0, (attempt(0), attempt(1))),
        ) + tuple(
            hazard(index) for index in range(1, MAX_HAZARDS_PER_MAP)
        )
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=hazards,
        )

        hazard_map.record_observation(
            PhysicalPose(x_mm=10_000),
            {
                "state_version": 100,
                "infrared": {
                    "blocked": True,
                    "raw": 20,
                    "filtered": 20,
                },
            },
            20_000,
        )

        self.assertEqual(hazard_map.scan_attempts_evicted, 2)
        self.assertEqual(
            hazard_map.scan_attempts_eviction_reason,
            HAZARD_SCAN_EVICTION,
        )
        self.assertEqual(
            hazard_map.scan_attempt_retention()["retained_count"],
            0,
        )

    def test_worst_runtime_detail_history_stays_below_memory_limit(self):
        hazards = tuple(
            hazard(
                index,
                (
                    tuple(
                        attempt(
                            index * MAX_SCAN_ATTEMPTS_PER_HAZARD + offset
                        )
                        for offset in range(
                            MAX_SCAN_ATTEMPTS_PER_HAZARD
                        )
                    )
                    if index
                    < MAX_SCAN_ATTEMPTS_PER_MAP
                    // MAX_SCAN_ATTEMPTS_PER_HAZARD
                    else ()
                ),
            )
            for index in range(MAX_HAZARDS_PER_MAP)
        )
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=hazards,
        )
        memory = NavigationMemoryStore(
            path=Path("runtime-detail-memory.json"),
            robot_id="ev3rstorm-01",
            controller_instance_id="ev3-main",
            frame_id="frame-a",
            generation_id="map-a",
            pose=PhysicalPose(),
            hazard_map=hazard_map,
        )
        memory_bytes = json.dumps(
            memory.to_dict(),
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")

        self.assertEqual(len(hazard_map.hazards), MAX_HAZARDS_PER_MAP)
        self.assertLess(len(memory_bytes), MAX_MEMORY_BYTES)
        self.assertGreater(len(memory_bytes), 128 * 1024)

    def test_materialized_support_caps_fit_two_mebibyte_memory(self):
        with self.assertRaisesRegex(ValueError, "support index"):
            ProvisionalHazard(
                **{
                    **hazard(999).__dict__,
                    "collision_supports": tuple(
                        support(index)
                        for index in range(
                            MAX_COLLISION_SUPPORTS_PER_HAZARD + 1
                        )
                    ),
                }
            )
        support_index = 0
        hazards = []
        for hazard_index in range(
            MAX_COLLISION_SUPPORTS_PER_MAP
            // MAX_COLLISION_SUPPORTS_PER_HAZARD
        ):
            retained = tuple(
                support(support_index + offset)
                for offset in range(MAX_COLLISION_SUPPORTS_PER_HAZARD)
            )
            support_index += MAX_COLLISION_SUPPORTS_PER_HAZARD
            hazards.append(ProvisionalHazard(
                **{
                    **hazard(hazard_index).__dict__,
                    "collision_supports": retained,
                    "scan_evidence_history": (
                        tuple(
                            attempt(
                                hazard_index
                                * MAX_SCAN_ATTEMPTS_PER_HAZARD
                                + offset
                            )
                            for offset in range(
                                MAX_SCAN_ATTEMPTS_PER_HAZARD
                            )
                        )
                        if hazard_index
                        < MAX_SCAN_ATTEMPTS_PER_MAP
                        // MAX_SCAN_ATTEMPTS_PER_HAZARD
                        else ()
                    ),
                }
            ))
        hazards.extend(
            hazard(index)
            for index in range(len(hazards), MAX_HAZARDS_PER_MAP)
        )
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=tuple(hazards),
        )
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore(
                path=Path(directory) / "maximum-runtime-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                frame_id="frame-a",
                generation_id="map-a",
                pose=PhysicalPose(),
                hazard_map=hazard_map,
            )
            memory_bytes = json.dumps(
                memory.to_dict(),
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")
            memory.save()
            reloaded = NavigationMemoryStore.load(
                path=memory.path,
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
            )

        self.assertEqual(
            sum(len(item.collision_supports) for item in hazard_map.hazards),
            MAX_COLLISION_SUPPORTS_PER_MAP,
        )
        self.assertLess(len(memory_bytes), MAX_MEMORY_BYTES)
        self.assertEqual(reloaded.to_dict()["schema"], MEMORY_SCHEMA)

        with self.assertRaisesRegex(ValueError, "map bound"):
            ProvisionalHazardMap(
                frame_id="frame-a",
                map_generation_id="map-a",
                hazards=tuple(hazards[:8]) + (ProvisionalHazard(
                    **{
                        **hazard(8).__dict__,
                        "collision_supports": (support(99_999),),
                    }
                ),),
            )

    def test_hazard_capacity_eviction_is_persisted_and_published(self):
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=tuple(
                hazard(index) for index in range(MAX_HAZARDS_PER_MAP)
            ),
        )
        oldest_id = hazard_map.hazards[0].hypothesis_id
        hazard_map.record_observation(
            PhysicalPose(x_mm=10_000),
            {
                "state_version": 100,
                "infrared": {
                    "blocked": True,
                    "raw": 20,
                    "filtered": 20,
                },
            },
            20_000,
        )

        self.assertNotIn(oldest_id, hazard_map.hazard_ids)
        self.assertEqual(len(hazard_map.hazards), MAX_HAZARDS_PER_MAP)
        self.assertEqual(hazard_map.hazards_evicted, 1)
        self.assertEqual(
            hazard_map.hazards_eviction_reason,
            HAZARD_CAPACITY_EVICTION,
        )
        self.assertEqual(hazard_map.context()["hazard_retention"], {
            "capacity": MAX_HAZARDS_PER_MAP,
            "retained_count": MAX_HAZARDS_PER_MAP,
            "evicted_count": 1,
            "last_eviction_reason": HAZARD_CAPACITY_EVICTION,
        })

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore(
                path=Path(directory) / "evicted-hazard-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                frame_id="frame-a",
                generation_id="map-a",
                pose=PhysicalPose(),
                hazard_map=hazard_map,
            )
            memory.save()
            reloaded = NavigationMemoryStore.load(
                path=memory.path,
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
            )

        self.assertEqual(reloaded.hazard_map.hazards_evicted, 1)
        self.assertEqual(
            reloaded.hazard_map.hazards_eviction_reason,
            HAZARD_CAPACITY_EVICTION,
        )
        self.assertEqual(reloaded.to_dict()["schema"], MEMORY_SCHEMA)

    def test_checked_in_live_v1_memory_migrates_to_v2_in_memory(self):
        fixture = (
            Path(__file__).resolve().parents[1]
            / "docs"
            / "data"
            / "EXP-EV3-LIVE-BOX-20260801-pre-fix-memory.json"
        )
        self.assertEqual(
            json.loads(fixture.read_text(encoding="utf-8"))["schema"],
            LEGACY_MEMORY_SCHEMA,
        )

        loaded = NavigationMemoryStore.load(
            path=fixture,
            robot_id="ev3rstorm-01",
            controller_instance_id="ev3rstorm-01.ev3-main",
        )

        self.assertEqual(loaded.hazard_map.hazards_evicted, 0)
        self.assertIsNone(loaded.hazard_map.hazards_eviction_reason)
        self.assertEqual(loaded.hazard_map.scan_attempts_evicted, 0)
        self.assertIsNone(
            loaded.hazard_map.scan_attempts_eviction_reason
        )
        self.assertEqual(loaded.to_dict()["schema"], MEMORY_SCHEMA)

    def test_initial_v2_memory_loads_with_zero_scan_eviction_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "initial-v2-memory.json"
            memory = NavigationMemoryStore(
                path=path,
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                frame_id="frame-a",
                generation_id="map-a",
                pose=PhysicalPose(),
                hazard_map=ProvisionalHazardMap(
                    frame_id="frame-a",
                    map_generation_id="map-a",
                    hazards=(hazard(0),),
                ),
            )
            value = dict(memory.to_dict())
            value.pop("scan_attempts_evicted")
            value.pop("scan_attempts_eviction_reason")
            for item in value["hazards"]:
                item.pop("scan_attempts_evicted")
                item.pop("scan_attempts_eviction_reason")
            path.write_text(json.dumps(value), encoding="utf-8")

            loaded = NavigationMemoryStore.load(
                path=path,
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
            )

        self.assertEqual(loaded.hazard_map.scan_attempts_evicted, 0)
        self.assertIsNone(
            loaded.hazard_map.scan_attempts_eviction_reason
        )

    def test_new_support_prunes_oldest_map_fact_with_counter(self):
        support_index = 0
        full = []
        for hazard_index in range(
            MAX_COLLISION_SUPPORTS_PER_MAP
            // MAX_COLLISION_SUPPORTS_PER_HAZARD
        ):
            retained = tuple(
                support(support_index + offset)
                for offset in range(MAX_COLLISION_SUPPORTS_PER_HAZARD)
            )
            support_index += MAX_COLLISION_SUPPORTS_PER_HAZARD
            full.append(ProvisionalHazard(
                **{
                    **hazard(hazard_index).__dict__,
                    "collision_supports": retained,
                }
            ))
        target = hazard(len(full))
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=tuple(full) + (target,),
        )
        hazard_map.record_observation(
            PhysicalPose(),
            {
                "state_version": 100,
                "infrared": {
                    "blocked": False,
                    "raw": 60,
                    "filtered": 60,
                },
            },
            19_000,
        )
        result = ActiveIrScanResult(
            scan_id="new-protected-scan",
            target_hypothesis_id=target.hypothesis_id,
            frame_id="frame-a",
            map_generation_id="map-a",
            based_on_map_version=0,
            started_at_ms=19_100,
            completed_at_ms=20_000,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            stop_confirmed=True,
            restored_start_heading=True,
            rays=(ActiveIrRay(
                ordinal=1,
                requested_relative_bearing_mdeg=0,
                actual_relative_bearing_mdeg=0,
                observed_at_ms=19_500,
                state_version=101,
                raw=20,
                filtered=20,
                blocked=True,
            ),),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
        )

        updated = hazard_map.record_scan_result(
            result,
            scan_pose=PhysicalPose(),
        )

        self.assertEqual(
            sum(len(item.collision_supports) for item in hazard_map.hazards),
            MAX_COLLISION_SUPPORTS_PER_MAP,
        )
        self.assertEqual(updated.collision_supports[0].source_scan_id,
                         "new-protected-scan")
        evicted = [
            item for item in hazard_map.hazards
            if item.collision_supports_evicted
        ]
        self.assertEqual(len(evicted), 1)
        self.assertEqual(evicted[0].collision_supports_evicted, 1)
        self.assertEqual(
            evicted[0].collision_supports_eviction_reason,
            MAP_SUPPORT_EVICTION,
        )


if __name__ == "__main__":
    unittest.main()
