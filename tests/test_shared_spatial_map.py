from copy import deepcopy
from unittest import TestCase

from robot_agent.shared_frame_transform import CalibratedFrameTransform
from robot_agent.shared_spatial_map import (
    MAX_SHARED_POSE_HISTORY,
    SharedSpatialMapCompositor,
    SharedSpatialMapError,
)


WORLD_FRAME_ID = "shared-world"
WORLD_GENERATION_ID = "shared-generation-1"


class Provider:
    def __init__(self, value=None, error=None):
        self.value = value
        self.error = error
        self.snapshot_calls = 0
        self.write_calls = 0
        self.close_calls = 0

    def snapshot(self):
        self.snapshot_calls += 1
        if self.error is not None:
            raise self.error
        return self.value

    def offer(self, *args, **kwargs):
        self.write_calls += 1
        raise AssertionError("shared compositor attempted a source write")

    def close(self, *args, **kwargs):
        self.close_calls += 1
        raise AssertionError("shared compositor attempted to close a source")


def frame_transform(
    robot_id,
    controller_id,
    frame_id,
    generation_id,
    *,
    tx_mm=0,
    ty_mm=0,
    yaw_mdeg=0,
    position_uncertainty_mm=10,
    yaw_uncertainty_mdeg=1_000,
    provenance=("FIXED_START_POSE",),
    world_frame_id=WORLD_FRAME_ID,
    world_generation_id=WORLD_GENERATION_ID,
):
    return CalibratedFrameTransform(
        source_robot_id=robot_id,
        source_controller_id=controller_id,
        source_frame_id=frame_id,
        source_generation_id=generation_id,
        world_frame_id=world_frame_id,
        world_generation_id=world_generation_id,
        tx_mm=tx_mm,
        ty_mm=ty_mm,
        yaw_mdeg=yaw_mdeg,
        position_uncertainty_mm=position_uncertainty_mm,
        yaw_uncertainty_mdeg=yaw_uncertainty_mdeg,
        provenance=provenance,
    )


def pose(
    frame_id,
    x_mm,
    y_mm,
    heading_mdeg,
    *,
    state_version=1,
    source_id="navigation-pose",
    provenance="LOCAL_ODOMETRY",
    observed_at_unix_ms=1_000,
    age_ms=0,
):
    return {
        "x_mm": x_mm,
        "y_mm": y_mm,
        "heading_mdeg": heading_mdeg,
        "frame_id": frame_id,
        "state_version": state_version,
        "source_id": source_id,
        "provenance": provenance,
        "observed_at_unix_ms": observed_at_unix_ms,
        "age_ms": age_ms,
    }


def local_map(
    robot_id,
    controller_id,
    frame_id,
    generation_id,
    *,
    current_pose=None,
    history=None,
    collision_geometry=None,
    map_version=1,
    captured_at_unix_ms=1_000,
):
    if current_pose is None:
        current_pose = pose(frame_id, 0, 0, 0)
    if history is None:
        history = [current_pose]
    return {
        "schema": "robot-spatial-map/v1",
        "read_only": True,
        "status": "pose_only",
        "map_id": "{}-local-map".format(robot_id),
        "robot_id": robot_id,
        "controller_instance_id": controller_id,
        "frame_id": frame_id,
        "local_generation_id": generation_id,
        "frame_kind": "LOCAL_ODOMETRY",
        "map_version": map_version,
        "robot_pose": current_pose,
        "pose_history": history,
        "pose_history_evicted": 0,
        "collision_geometry": collision_geometry,
        "captured_at_unix_ms": captured_at_unix_ms,
        "age_ms": 0,
        "cells": [],
        "sensor_rays": [],
        "object_hypotheses": [],
    }


def navigation_trace(frame_id):
    return {
        "schema": "robot-navigation-trace/v1",
        "read_only": True,
        "frame_id": frame_id,
        "provenance": (
            "PROVISIONAL_ENCODER_ODOMETRY + PROVISIONAL_YAW_ONLY"
        ),
        "final_goal": {
            "kind": "DIRECTIONAL_HEADING",
            "origin_x_mm": 0,
            "origin_y_mm": 0,
            "target_x_mm": 420,
            "target_y_mm": 0,
            "goal_radius_mm": 120,
            "distance_to_goal_mm": 375,
            "desired_heading_mdeg": 0,
            "minimum_forward_progress_mm": 420,
            "heading_tolerance_mdeg": 5_000,
            "current_forward_progress_mm": 45,
            "current_lateral_offset_mm": 0,
            "remaining_forward_progress_mm": 375,
            "navigation_enforced": False,
        },
        "imu_heading": {
            "heading_mdeg": 0,
            "reference": "EPISODE_START",
            "observed_at_unix_ms": 1_000,
        },
        "planned_leg": {
            "kind": "SIDE_SEARCH",
            "scope": "SEARCH_POSITION_ONLY",
            "clearance_proven": False,
            "passage_proven": False,
            "route_eligible": False,
            "selected_side": "LEFT",
            "bind_pose": {
                "x_mm": 45,
                "y_mm": 0,
                "heading_mdeg": 0,
            },
            "waypoint": {
                "x_mm": 90,
                "y_mm": 210,
                "heading_mdeg": 90_000,
            },
        },
        "advisory_waypoint": None,
        "planar_scan_views": [{
            "scan_id": "scan-1",
            "observed_at_unix_ms": 1_000,
            "scan_pose": {
                "x_mm": 10,
                "y_mm": 20,
                "heading_mdeg": 0,
            },
            "projection": {
                "schema": "blast-planar-scan-projection/v1",
                "frame": "EPISODE_LOCAL_ODOMETRY",
                "quality": "PROVISIONAL_YAW_ONLY",
                "vertical_pitch_compensated": False,
                "ultrasonic_beam_width_modeled": False,
                "scan_turn_translation_compensated": False,
                "points": [{
                    "side": "center",
                    "measured_range_mm": 300.0,
                    "relative_bearing_mdeg": 0,
                    "sensor_origin_x_mm": 110,
                    "sensor_origin_y_mm": 20,
                    "beam_heading_mdeg": 0,
                    "nominal_echo_x_mm": 410,
                    "nominal_echo_y_mm": 20,
                }],
            },
        }],
    }


def compositor(*bindings):
    return SharedSpatialMapCompositor(
        world_frame_id=WORLD_FRAME_ID,
        world_generation_id=WORLD_GENERATION_ID,
        bindings=tuple(bindings),
    )


class SharedSpatialMapCompositorTests(TestCase):
    def test_navigation_trace_is_transformed_for_identity_translation_and_yaw(self):
        cases = (
            {
                "name": "identity",
                "tx_mm": 0,
                "ty_mm": 0,
                "yaw_mdeg": 0,
                "goal": (0, 0, 420, 0, 0),
                "bind_pose": (45, 0, 0),
                "waypoint": (90, 210, 90_000),
                "scan_pose": (10, 20, 0),
                "ray": (110, 20, 0, 410, 20),
            },
            {
                "name": "translation",
                "tx_mm": 1_000,
                "ty_mm": 500,
                "yaw_mdeg": 0,
                "goal": (1_000, 500, 1_420, 500, 0),
                "bind_pose": (1_045, 500, 0),
                "waypoint": (1_090, 710, 90_000),
                "scan_pose": (1_010, 520, 0),
                "ray": (1_110, 520, 0, 1_410, 520),
            },
            {
                "name": "quarter_turn",
                "tx_mm": 1_000,
                "ty_mm": 500,
                "yaw_mdeg": 90_000,
                "goal": (1_000, 500, 1_000, 920, 90_000),
                "bind_pose": (1_000, 545, 90_000),
                "waypoint": (790, 590, -180_000),
                "scan_pose": (980, 510, 90_000),
                "ray": (980, 610, 90_000, 980, 910),
            },
        )
        for case in cases:
            with self.subTest(case=case["name"]):
                calibration = frame_transform(
                    "blast-01",
                    "blast-hub",
                    "blast-local",
                    "episode-a",
                    tx_mm=case["tx_mm"],
                    ty_mm=case["ty_mm"],
                    yaw_mdeg=case["yaw_mdeg"],
                )
                value = local_map(
                    "blast-01",
                    "blast-hub",
                    "blast-local",
                    "episode-a",
                )
                value["navigation_trace"] = navigation_trace(
                    "blast-local"
                )

                robot = compositor(
                    (Provider(value), calibration)
                ).snapshot()["robots"][0]
                trace = robot["navigation_trace"]

                self.assertEqual(robot["status"], "available")
                self.assertEqual(trace["frame_id"], WORLD_FRAME_ID)
                self.assertEqual(
                    trace["world_generation_id"], WORLD_GENERATION_ID
                )
                self.assertEqual(trace["local_frame_id"], "blast-local")
                self.assertEqual(
                    trace["local_generation_id"], "episode-a"
                )
                self.assertEqual(trace["source_robot_id"], "blast-01")
                self.assertEqual(
                    trace["source_controller_instance_id"], "blast-hub"
                )
                self.assertTrue(trace["read_only"])
                self.assertEqual(
                    trace["provenance"],
                    value["navigation_trace"]["provenance"],
                )
                self.assertEqual(
                    trace["transform_provenance"],
                    ["FIXED_START_POSE"],
                )
                goal = trace["final_goal"]
                self.assertEqual(
                    (
                        goal["origin_x_mm"],
                        goal["origin_y_mm"],
                        goal["target_x_mm"],
                        goal["target_y_mm"],
                        goal["desired_heading_mdeg"],
                    ),
                    case["goal"],
                )
                self.assertFalse(goal["navigation_enforced"])
                self.assertEqual(
                    trace["imu_heading"]["heading_mdeg"],
                    case["yaw_mdeg"],
                )
                self.assertEqual(
                    tuple(trace["planned_leg"]["bind_pose"].values()),
                    case["bind_pose"],
                )
                self.assertEqual(
                    tuple(trace["planned_leg"]["waypoint"].values()),
                    case["waypoint"],
                )
                projection = trace["planar_scan_views"][0]["projection"]
                self.assertEqual(projection["frame"], "SHARED_FIXED_START")
                self.assertEqual(
                    projection["local_frame"],
                    "EPISODE_LOCAL_ODOMETRY",
                )
                self.assertEqual(
                    tuple(
                        trace["planar_scan_views"][0]["scan_pose"].values()
                    ),
                    case["scan_pose"],
                )
                ray = projection["points"][0]
                self.assertEqual(
                    (
                        ray["sensor_origin_x_mm"],
                        ray["sensor_origin_y_mm"],
                        ray["beam_heading_mdeg"],
                        ray["nominal_echo_x_mm"],
                        ray["nominal_echo_y_mm"],
                    ),
                    case["ray"],
                )
                self.assertEqual(ray["measured_range_mm"], 300.0)
                self.assertEqual(ray["relative_bearing_mdeg"], 0)

    def test_malformed_navigation_trace_isolated_from_healthy_source(self):
        broken_value = local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        broken_value["navigation_trace"] = navigation_trace("blast-local")
        broken_value["navigation_trace"]["planar_scan_views"][0][
            "projection"
        ]["points"][0]["nominal_echo_x_mm"] = 999
        healthy_value = local_map(
            "ev3rstorm-01",
            "ev3-controller",
            "ev3-local",
            "generation-a",
        )

        value = compositor(
            (
                Provider(broken_value),
                frame_transform(
                    "blast-01",
                    "blast-hub",
                    "blast-local",
                    "episode-a",
                ),
            ),
            (
                Provider(healthy_value),
                frame_transform(
                    "ev3rstorm-01",
                    "ev3-controller",
                    "ev3-local",
                    "generation-a",
                ),
            ),
        ).snapshot()

        self.assertEqual(value["status"], "degraded")
        robots = {robot["robot_id"]: robot for robot in value["robots"]}
        self.assertEqual(robots["ev3rstorm-01"]["status"], "available")
        self.assertIsNone(robots["ev3rstorm-01"]["navigation_trace"])
        self.assertEqual(robots["blast-01"]["status"], "unavailable")
        self.assertEqual(
            robots["blast-01"]["reason_code"],
            "source_navigation_trace_invalid",
        )
        self.assertIsNone(robots["blast-01"]["navigation_trace"])

    def test_navigation_trace_frame_mismatch_fails_closed(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        value = local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        value["navigation_trace"] = navigation_trace("retired-frame")

        robot = compositor(
            (Provider(value), calibration)
        ).snapshot()["robots"][0]

        self.assertEqual(robot["status"], "unavailable")
        self.assertEqual(
            robot["reason_code"],
            "source_navigation_trace_frame_mismatch",
        )
        self.assertIsNone(robot["navigation_trace"])

    def test_navigation_enforcement_and_proof_flags_are_not_promoted(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        value = local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        trace = navigation_trace("blast-local")
        trace["final_goal"]["navigation_enforced"] = True
        trace["planned_leg"].update({
            "kind": "LATERAL_CLEARANCE",
            "scope": "LOCAL_DETOUR_ROUTE",
            "route_eligible": True,
        })
        value["navigation_trace"] = trace

        shared = compositor((Provider(value), calibration)).snapshot()
        transformed = shared["robots"][0]["navigation_trace"]

        self.assertTrue(transformed["final_goal"]["navigation_enforced"])
        self.assertTrue(transformed["planned_leg"]["route_eligible"])
        self.assertFalse(transformed["planned_leg"]["clearance_proven"])
        self.assertFalse(transformed["planned_leg"]["passage_proven"])
        self.assertIsNone(shared["navigation_authority"])

    def test_source_without_navigation_trace_remains_available(self):
        calibration = frame_transform(
            "ev3rstorm-01",
            "ev3-controller",
            "ev3-local",
            "generation-a",
        )
        value = local_map(
            "ev3rstorm-01",
            "ev3-controller",
            "ev3-local",
            "generation-a",
        )

        robot = compositor(
            (Provider(value), calibration)
        ).snapshot()["robots"][0]

        self.assertEqual(robot["status"], "available")
        self.assertIsNone(robot["navigation_trace"])

    def test_exact_two_source_fixture_is_canonical_and_world_aligned(self):
        ev3_transform = frame_transform(
            "ev3rstorm-01",
            "ev3-controller",
            "ev3-local",
            "ev3-generation-4",
            provenance=("MEASURED_FIXED_START",),
        )
        blast_transform = frame_transform(
            "blast-01",
            "blast-01.hub",
            "blast-local",
            "blast-episode-7",
            tx_mm=1_000,
            ty_mm=500,
            yaw_mdeg=90_000,
            position_uncertainty_mm=12,
            yaw_uncertainty_mdeg=2_000,
        )
        circle = {
            "geometry": "SYMMETRIC_CIRCLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "radius_mm": 90,
        }
        rectangle = {
            "geometry": "ASYMMETRIC_RECTANGLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "front_extent_mm": 150,
            "rear_extent_mm": 90,
            "left_extent_mm": 80,
            "right_extent_mm": 80,
            "clearance_margin_mm": 15,
            "calibration_status": "PROVISIONAL",
            "calibration_evidence": "MEASURED_BODY_EXTENTS",
        }
        ev3_current = pose("ev3-local", 40, -10, 5_000)
        ev3_source = Provider(local_map(
            "ev3rstorm-01",
            "ev3-controller",
            "ev3-local",
            "ev3-generation-4",
            current_pose=ev3_current,
            history=[pose("ev3-local", 0, 0, 0), ev3_current],
            collision_geometry=circle,
            map_version=8,
            captured_at_unix_ms=1_100,
        ))
        blast_current = pose(
            "blast-local",
            100,
            0,
            -90_000,
            source_id="blast-navigation-motion-executor",
            provenance="PROVISIONAL_ENCODER_ODOMETRY",
        )
        blast_source = Provider(local_map(
            "blast-01",
            "blast-01.hub",
            "blast-local",
            "blast-episode-7",
            current_pose=blast_current,
            history=[pose("blast-local", 0, 0, 0), blast_current],
            collision_geometry=rectangle,
            map_version=3,
            captured_at_unix_ms=1_200,
        ))

        value = compositor(
            (ev3_source, ev3_transform),
            (blast_source, blast_transform),
        ).snapshot()

        self.assertEqual(value["schema"], "robot-spatial-map/v2")
        self.assertTrue(value["read_only"])
        self.assertEqual(value["status"], "available")
        self.assertEqual(value["reason_code"], "all_sources_available")
        self.assertEqual(value["frame_id"], WORLD_FRAME_ID)
        self.assertEqual(value["frame_kind"], "SHARED_FIXED_START")
        self.assertEqual(
            value["snapshot_semantics"], "LATEST_AVAILABLE_NOT_ATOMIC"
        )
        self.assertEqual(
            value["provenance"],
            "CALIBRATED_FIXED_START_SE2_PROJECTION",
        )
        self.assertEqual(
            value["world_generation_id"], WORLD_GENERATION_ID
        )
        self.assertEqual(value["captured_at_unix_ms"], 1_200)
        self.assertEqual(
            [robot["robot_id"] for robot in value["robots"]],
            ["blast-01", "ev3rstorm-01"],
        )

        blast = value["robots"][0]
        self.assertEqual(blast["status"], "available")
        self.assertEqual(blast["local_generation_id"], "blast-episode-7")
        self.assertEqual(blast["captured_at_unix_ms"], 1_200)
        self.assertEqual(blast["source_age_ms"], 0)
        self.assertEqual(
            (
                blast["robot_pose"]["x_mm"],
                blast["robot_pose"]["y_mm"],
                blast["robot_pose"]["heading_mdeg"],
            ),
            (1_000, 600, 0),
        )
        self.assertEqual(
            (
                blast["pose_history"][0]["x_mm"],
                blast["pose_history"][0]["y_mm"],
                blast["pose_history"][0]["heading_mdeg"],
            ),
            (1_000, 500, 90_000),
        )
        self.assertEqual(blast["collision_geometry"], rectangle)
        self.assertEqual(
            blast["frame_transform"]["position_uncertainty_mm"], 12
        )
        self.assertEqual(
            blast["frame_transform"]["yaw_uncertainty_mdeg"], 2_000
        )
        self.assertEqual(
            blast["frame_transform"]["provenance"],
            ["FIXED_START_POSE"],
        )

        ev3 = value["robots"][1]
        self.assertEqual(
            (
                ev3["robot_pose"]["x_mm"],
                ev3["robot_pose"]["y_mm"],
                ev3["robot_pose"]["heading_mdeg"],
            ),
            (40, -10, 5_000),
        )
        self.assertEqual(ev3["collision_geometry"], circle)

    def test_frame_or_generation_mismatch_isolated_as_unavailable(self):
        calibration = frame_transform(
            "blast-01",
            "blast-01.hub",
            "blast-local",
            "episode-current",
        )
        for field, wrong in (
            ("robot_id", "other-robot"),
            ("controller_instance_id", "other-controller"),
            ("frame_id", "retired-frame"),
            ("local_generation_id", "retired-episode"),
        ):
            value = local_map(
                "blast-01",
                "blast-01.hub",
                "blast-local",
                "episode-current",
            )
            value[field] = wrong
            with self.subTest(field=field):
                snapshot = compositor(
                    (Provider(value), calibration)
                ).snapshot()
                self.assertEqual(snapshot["status"], "unavailable")
                self.assertEqual(
                    snapshot["robots"][0]["reason_code"],
                    "source_identity_mismatch",
                )
                self.assertIsNone(snapshot["robots"][0]["robot_pose"])

    def test_incomplete_source_contract_is_not_drawn_as_available(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        for missing in (
            "map_id", "map_version", "captured_at_unix_ms", "age_ms",
        ):
            value = local_map(
                "blast-01", "blast-hub", "blast-local", "episode-a"
            )
            del value[missing]
            with self.subTest(missing=missing):
                robot = compositor(
                    (Provider(value), calibration)
                ).snapshot()["robots"][0]
                self.assertEqual(robot["status"], "unavailable")
                self.assertEqual(
                    robot["reason_code"], "source_contract_mismatch"
                )

        for missing in (
            "state_version", "source_id", "provenance",
            "observed_at_unix_ms", "age_ms",
        ):
            value = local_map(
                "blast-01", "blast-hub", "blast-local", "episode-a"
            )
            del value["robot_pose"][missing]
            with self.subTest(missing=missing):
                robot = compositor(
                    (Provider(value), calibration)
                ).snapshot()["robots"][0]
                self.assertEqual(robot["status"], "unavailable")
                self.assertEqual(robot["reason_code"], "source_pose_invalid")

    def test_one_broken_source_does_not_hide_the_healthy_robot(self):
        healthy_transform = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        broken_transform = frame_transform(
            "ev3rstorm-01", "ev3-controller", "ev3-local", "generation-a"
        )
        healthy = Provider(local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        ))
        broken = Provider(error=RuntimeError("source disconnected"))

        value = compositor(
            (broken, broken_transform),
            (healthy, healthy_transform),
        ).snapshot()

        self.assertEqual(value["status"], "degraded")
        self.assertEqual(value["reason_code"], "some_sources_unavailable")
        self.assertEqual(value["robots"][0]["status"], "available")
        self.assertEqual(value["robots"][1]["status"], "unavailable")
        self.assertEqual(
            value["robots"][1]["reason_code"], "source_snapshot_failed"
        )

    def test_duplicate_bindings_and_invalid_provider_are_rejected(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        provider = Provider(local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        ))
        with self.assertRaisesRegex(
            SharedSpatialMapError, "duplicate shared map binding"
        ):
            compositor(
                (provider, calibration),
                (provider, calibration),
            )
        with self.assertRaisesRegex(
            SharedSpatialMapError, "provider snapshot"
        ):
            compositor((object(), calibration))

    def test_history_is_bounded_and_reports_local_eviction(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        history = [
            pose("blast-local", index, 0, 0, state_version=index + 1)
            for index in range(MAX_SHARED_POSE_HISTORY + 3)
        ]
        value = local_map(
            "blast-01",
            "blast-hub",
            "blast-local",
            "episode-a",
            current_pose=history[-1],
            history=history,
        )
        value["pose_history_evicted"] = 2

        robot = compositor(
            (Provider(value), calibration)
        ).snapshot()["robots"][0]

        self.assertEqual(len(robot["pose_history"]), MAX_SHARED_POSE_HISTORY)
        self.assertEqual(robot["pose_history"][0]["x_mm"], 3)
        self.assertEqual(robot["pose_history_evicted"], 5)

    def test_snapshot_only_reads_and_never_writes_or_closes_sources(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        provider = Provider(local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        ))
        shared = compositor((provider, calibration))

        first = shared.snapshot()
        second = shared.snapshot()

        self.assertEqual(first["status"], "available")
        self.assertEqual(second["status"], "available")
        self.assertEqual(provider.snapshot_calls, 2)
        self.assertEqual(provider.write_calls, 0)
        self.assertEqual(provider.close_calls, 0)

    def test_returned_snapshot_is_deeply_detached(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        geometry = {
            "geometry": "SYMMETRIC_CIRCLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "radius_mm": 100,
        }
        source_value = local_map(
            "blast-01",
            "blast-hub",
            "blast-local",
            "episode-a",
            collision_geometry=geometry,
        )
        source_before = deepcopy(source_value)
        shared = compositor((Provider(source_value), calibration))

        first = shared.snapshot()
        first["robots"][0]["robot_pose"]["x_mm"] = 999
        first["robots"][0]["collision_geometry"]["radius_mm"] = 999
        first["robots"][0]["frame_transform"]["provenance"].append(
            "CHANGED"
        )

        second = shared.snapshot()
        self.assertEqual(second["robots"][0]["robot_pose"]["x_mm"], 0)
        self.assertEqual(
            second["robots"][0]["collision_geometry"]["radius_mm"], 100
        )
        self.assertEqual(
            second["robots"][0]["frame_transform"]["provenance"],
            ["FIXED_START_POSE"],
        )
        self.assertEqual(source_value, source_before)

    def test_no_cells_rays_hypotheses_or_geometry_facts_are_invented(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        geometry = {
            "geometry": "SYMMETRIC_CIRCLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "radius_mm": 100,
        }
        value = local_map(
            "blast-01",
            "blast-hub",
            "blast-local",
            "episode-a",
            collision_geometry=geometry,
        )
        value.update({
            "bounds": {"min_x_mm": -100, "max_x_mm": 100},
            "cells": [{"x_mm": 10, "y_mm": 20, "state": "OCCUPIED"}],
            "sensor_rays": [{"state": "MEASURED"}],
            "qualitative_observations": [{"relation": "NEAR_OBSTACLE"}],
            "scan_evidence_history": [{"scan_id": "scan-a"}],
            "object_hypotheses": [{"hypothesis_id": "object-a"}],
        })

        shared = compositor((Provider(value), calibration)).snapshot()

        self.assertIsNone(shared["bounds"])
        self.assertEqual(shared["cells"], [])
        self.assertEqual(shared["sensor_rays"], [])
        self.assertEqual(shared["qualitative_observations"], [])
        self.assertEqual(shared["scan_evidence_history"], [])
        self.assertEqual(shared["object_hypotheses"], [])
        self.assertIsNone(shared["navigation_authority"])
        robot = shared["robots"][0]
        self.assertEqual(robot["collision_geometry"], geometry)
        self.assertEqual(robot["object_hypotheses"], [])
        for forbidden in (
            "bounds",
            "cells",
            "sensor_rays",
            "qualitative_observations",
            "scan_evidence_history",
            "navigation_authority",
        ):
            self.assertNotIn(forbidden, robot)
        self.assertIsNone(robot["navigation_trace"])

    def test_provisional_ultrasonic_cluster_is_transformed_per_robot_only(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a",
            tx_mm=100, ty_mm=200,
        )
        value = local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a",
        )
        value["status"] = "qualitative_only"
        value["object_hypotheses"] = [{
            "hypothesis_id": "blast-ultrasonic-a",
            "classification": "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER",
            "label": "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER",
            "x_mm": 100,
            "y_mm": 0,
            "geometry_kind": "PROVISIONAL_ULTRASONIC_ECHO_CLUSTER",
            "support_radius_mm": 35,
            "support_points": [{
                "side": "center", "x_mm": 100, "y_mm": 0,
                "measured_range_mm": 100,
                "relative_bearing_mdeg": 0,
            }],
            "source_scan_ids": ["dense-scan"],
            "bearing": "FRONT",
            "relation": "FRONT_OF_SCAN",
            "evidence_count": 1,
            "confidence_milli": 200,
            "source_id": "blast-settled-measured-planar-projection",
            "provenance": (
                "SETTLED_MEASURED_ULTRASONIC + PROVISIONAL_YAW_ONLY"
            ),
            "quality": "PROVISIONAL_YAW_ONLY",
            "settled_measured_only": True,
            "provisional": True,
            "read_only": True,
            "observed_at_unix_ms": 1_000,
            "age_ms": 0,
        }]

        shared = compositor((Provider(value), calibration)).snapshot()

        self.assertEqual(shared["object_hypotheses"], [])
        hypotheses = shared["robots"][0]["object_hypotheses"]
        self.assertEqual(len(hypotheses), 1)
        self.assertEqual(
            (hypotheses[0]["x_mm"], hypotheses[0]["y_mm"]),
            (200, 200),
        )
        self.assertEqual(
            (
                hypotheses[0]["support_points"][0]["x_mm"],
                hypotheses[0]["support_points"][0]["y_mm"],
            ),
            (200, 200),
        )


if __name__ == "__main__":
    import unittest

    unittest.main()
