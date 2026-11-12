import threading
import time
import unittest
from unittest.mock import patch

from swarmcontrol import swarm
from swarmcontrol.formation import calculate_formation_offsets


class FakeHandler:
    def __init__(self, sysid, lat=10.0, lon=10.0, alt=0.0, armed=False, calls=None):
        self.sysid = sysid
        self.connection = object()
        self.lat, self.lon, self.current_alt = lat, lon, alt
        self.armed = armed
        self.yaw = 0.0
        self.calls = calls if calls is not None else []
        self.auto_land = True
        self.auto_fly = True
        self.auto_takeoff = True

    def arm_drone(self):
        self.calls.append((self.sysid, "arm"))
        self.armed = True

    def disarm_drone(self):
        self.calls.append((self.sysid, "disarm"))
        self.armed = False

    def takeoff(self, alt):
        self.calls.append((self.sysid, "takeoff", alt))
        if self.auto_takeoff:
            self.armed, self.current_alt = True, alt

    def fly_to_gps(self, lat, lon, alt, set_guided=True):
        self.calls.append((self.sysid, "fly_to", lat, lon, alt))
        if self.auto_fly:
            self.lat, self.lon = lat, lon

    def land(self):
        self.calls.append((self.sysid, "land"))
        if self.auto_land:
            self.armed, self.current_alt = False, 0.0

    def rtl(self):
        self.calls.append((self.sysid, "rtl"))
        if self.auto_land:
            self.armed, self.current_alt = False, 0.0


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.patchers = [patch.object(swarm, "_start_inactivity_watchdog"),
                         patch.object(swarm, "TAKEOFF_STAGGER_S", 0),
                         patch.object(swarm, "SERIAL_DISPATCH_SPACING_S", 0)]
        for p in self.patchers:
            p.start()
        swarm._cancel_worker()
        swarm.set_leader(2)

    def tearDown(self):
        swarm._cancel_worker()
        worker = swarm._current_worker
        if worker:
            worker.join(1)
        for p in reversed(self.patchers):
            p.stop()

    def _wait_for(self, condition, timeout=1):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            if condition():
                return
            time.sleep(0.005)
        self.fail("condition was not reached")

    def test_selected_leader_takes_off_and_navigates_first(self):
        calls = []
        follower = FakeHandler(1, lon=10.0002, calls=calls)
        leader = FakeHandler(2, calls=calls)
        swarm.perform_swarm_command({"cmd": "fly_to", "lat": 10.001, "lon": 10.0,
                                     "alt": 6, "formation": False}, [follower, leader])
        self._wait_for(lambda: len([c for c in calls if c[1] == "fly_to"]) == 2)
        self.assertEqual([(c[0], c[1]) for c in calls],
                         [(2, "takeoff"), (1, "takeoff"), (2, "fly_to"), (1, "fly_to")])
        self.assertEqual([c[4] for c in calls if c[1] == "fly_to"], [6, 6])
        self.assertAlmostEqual(calls[-1][3] - calls[-2][3], 0.0002)

    def test_follower_takeoff_waits_for_airborne_leader(self):
        calls = []
        follower = FakeHandler(1, calls=calls)
        leader = FakeHandler(2, calls=calls)
        leader.auto_takeoff = False
        swarm.perform_swarm_command({"cmd": "takeoff", "alt": 5}, [follower, leader])
        self._wait_for(lambda: calls)
        self.assertEqual(calls, [(2, "takeoff", 5.0)])
        leader.armed, leader.current_alt = True, 5.0
        self._wait_for(lambda: len(calls) == 2)
        self.assertEqual(calls[1][:2], (1, "takeoff"))

    def test_land_waits_for_leader_grounded_and_disarmed(self):
        calls = []
        follower = FakeHandler(1, alt=5, armed=True, calls=calls)
        leader = FakeHandler(2, alt=5, armed=True, calls=calls)
        leader.auto_land = False
        swarm.perform_swarm_command({"cmd": "land"}, [follower, leader])
        self._wait_for(lambda: calls)
        self._wait_for(lambda: len(calls) >= 2)
        self.assertEqual(calls[:2], [(2, "land"), (1, "fly_to", 10.0, 10.0, 5.0)])
        leader.current_alt = 0.1  # altitude alone does not satisfy the gate
        time.sleep(0.03)
        self.assertEqual([c for c in calls if c[1] == "land"], [(2, "land")])
        leader.armed = False
        self._wait_for(lambda: len(calls) == 3)
        self.assertEqual(calls[2], (1, "land"))

    def test_land_timeout_holds_followers(self):
        calls = []
        follower = FakeHandler(1, alt=5, armed=True, calls=calls)
        leader = FakeHandler(2, alt=5, armed=True, calls=calls)
        leader.auto_land = False
        with patch.object(swarm, "LAND_TIMEOUT_S", 0.03):
            swarm.perform_swarm_command({"cmd": "rtl"}, [follower, leader])
            worker = swarm._current_worker
            worker.join(1)
        self.assertEqual(calls, [(2, "rtl"), (1, "fly_to", 10.0, 10.0, 5.0)])

    def test_land_cancels_older_navigation(self):
        calls = []
        follower = FakeHandler(1, lon=10.0002, alt=5, armed=True, calls=calls)
        leader = FakeHandler(2, alt=5, armed=True, calls=calls)
        leader.auto_fly = False  # mission remains waiting at the first waypoint
        swarm.perform_swarm_command({"cmd": "fly_to", "lat": 10.001, "lon": 10.0,
                                     "alt": 5, "formation": False}, [follower, leader])
        self._wait_for(lambda: any(c[1] == "fly_to" for c in calls))
        swarm.perform_swarm_command({"cmd": "land"}, [follower, leader])
        self._wait_for(lambda: len([c for c in calls if c[1] == "land"]) == 2)
        self.assertTrue(all(c[2:4] == (10.0, 10.0002) for c in calls
                            if c[0] == 1 and c[1] == "fly_to"))
        self.assertEqual([c[0] for c in calls if c[1] == "land"], [2, 1])

    def test_crossing_v_entry_rejected_before_navigation(self):
        leader = FakeHandler(2)
        right, left = calculate_formation_offsets(10.0, 10.0, 0, 2, "V", 10)
        # Followers start on opposite sides of their V target; endpoints are clear.
        first = FakeHandler(1, *left)
        second = FakeHandler(3, *right)
        targets = swarm._target_map({"formation": True, "pattern": "V", "spacing_m": 10},
                                    [leader, first, second], leader, 10.0, 10.0, 5)
        with self.assertRaisesRegex(ValueError, "unsafe path"):
            swarm._safe_sequential_paths([leader, first, second], targets)

    def test_crossing_v_exit_rejected(self):
        leader = FakeHandler(2)
        right, left = calculate_formation_offsets(10.0, 10.0, 0, 2, "V", 10)
        first = FakeHandler(1, *right, alt=5, armed=True)
        second = FakeHandler(3, *left, alt=5, armed=True)
        # Leaving the V by swapping sides crosses a stationary craft's start.
        targets = {leader: (10.0, 10.0, 5),
                   first: (left[0], left[1], 5),
                   second: (right[0], right[1], 5)}
        with self.assertRaisesRegex(ValueError, "unsafe path"):
            swarm._safe_sequential_paths([leader, first, second], targets)

    def test_emergency_disarm_is_immediate_leader_first(self):
        calls = []
        follower = FakeHandler(1, calls=calls)
        leader = FakeHandler(2, calls=calls)
        swarm.perform_swarm_command({"cmd": "disarm"}, [follower, leader])
        self.assertEqual(calls, [(2, "disarm"), (1, "disarm")])

    def test_manual_follow_lands_after_leader_touchdown(self):
        calls = []
        leader = FakeHandler(2, alt=5, armed=True, calls=calls)
        follower = FakeHandler(1, lon=10.0002, alt=5, armed=True, calls=calls)
        swarm.perform_swarm_command({"cmd": "manual_mode", "alt": 5,
                                     "formation": False}, [follower, leader])
        self._wait_for(lambda: any(c[1] == "fly_to" for c in calls))
        fly = next(c for c in calls if c[1] == "fly_to")
        self.assertAlmostEqual(fly[3] - leader.lon, 0.0002)
        leader.armed, leader.current_alt = False, 0.0
        self._wait_for(lambda: any(c[1] == "land" for c in calls))
        self.assertEqual([c[0] for c in calls if c[1] == "land"], [1])

    def test_manual_follow_holds_when_rc_leader_enters_leg(self):
        calls = []
        leader = FakeHandler(2, alt=5, armed=True, calls=calls)
        follower = FakeHandler(1, lon=10.0002, alt=5, armed=True, calls=calls)
        follower.auto_fly = False
        swarm.perform_swarm_command({"cmd": "manual_mode", "alt": 5,
                                     "formation": False}, [follower, leader])
        self._wait_for(lambda: any(c[1] == "fly_to" for c in calls))
        leader.lat = 10.001
        self._wait_for(lambda: any(c[1] == "fly_to" and c[2] == 10.001 for c in calls))
        leader.lat, leader.lon = 10.0005, 10.0002
        self._wait_for(lambda: calls[-1][1] == "fly_to" and calls[-1][2:4] == (10.0, 10.0002))
        self.assertEqual(calls[-1][0], 1)

    def test_replaced_manual_worker_does_not_clear_new_status_late(self):
        calls, states = [], []
        leader = FakeHandler(2, alt=5, armed=True, calls=calls)
        follower = FakeHandler(1, lon=10.0002, alt=5, armed=True, calls=calls)
        with patch.object(swarm, "_manual_status", side_effect=states.append):
            swarm.perform_swarm_command({"cmd": "manual_mode", "alt": 5,
                                         "formation": False}, [follower, leader])
            self._wait_for(lambda: states == [True])
            old_worker = swarm._current_worker
            swarm.perform_swarm_command({"cmd": "land"}, [follower, leader])
            old_worker.join(1)
            self._wait_for(lambda: states[-1] is False)
            self.assertEqual(states, [True, False])

    def test_changing_leader_cancels_active_mission(self):
        calls = []
        follower = FakeHandler(1, lon=10.0002, alt=5, armed=True, calls=calls)
        leader = FakeHandler(2, alt=5, armed=True, calls=calls)
        leader.auto_fly = False
        swarm.perform_swarm_command({"cmd": "fly_to", "lat": 10.001, "lon": 10.0,
                                     "alt": 5, "formation": False}, [follower, leader])
        self._wait_for(lambda: any(c[1] == "fly_to" for c in calls))
        worker = swarm._current_worker
        swarm.set_leader(1)
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertTrue(all(c[2:4] == (10.0, 10.0002) for c in calls
                            if c[0] == 1 and c[1] == "fly_to"))

    def test_zero_altitude_is_rejected(self):
        calls = []
        leader = FakeHandler(2, calls=calls)
        swarm.perform_swarm_command({"cmd": "takeoff", "alt": 0}, [leader])
        self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
