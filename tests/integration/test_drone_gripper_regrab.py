#!/usr/bin/env python3
"""Regression test for RoboticsAcademy#3932.

"Package delivery: drone magnet fails to re-grab the box after a reset
performed mid-carry."

Background
----------
The ``drone_gripper`` plugin (RoboticsInfrastructure,
``Industrial/drone_gripper``) attaches the payload (``package_box_01``) to the
drone with a dynamically created ``gz::sim::components::DetachableJoint`` whose
parent is the drone's gripper link and whose child is the box's canonical link.

In dartsim, ``AttachFixedJoint`` does *not* add a constraint: it moves the
child's BodyNode into the **parent's skeleton** (``bn->moveTo<WeldJoint>``).
So while attached, the box body physically lives inside the drone's skeleton.

On ``WorldReset(all=true)`` after a mid-carry reset the orchestration
(``pause -> remove drone -> WorldReset(all=true) -> respawn``) tears down the
drone model. ``Physics::RemovePhysicsEntities`` processes ``EachRemoved<Model>``
*before* ``EachRemoved<DetachableJoint>``, so the drone skeleton - which still
owns the box body - is destroyed before the dartsim un-reparenting runs. The
box body is left corrupted: an ECM-level joint is created fine on the next grab
(so ``attached`` may even report ``true``) but the physics engine never
enforces it.

The fix in ``drone_gripper.cpp`` flushes the joint-entity removal synchronously
(``_ecm.ProcessRemoveEntityRequests()``) right after requesting it, so the
dartsim un-reparenting happens one ``Physics::Update`` earlier, before the drone
model is removed.

This test is self-contained: it launches headless ``gz sim -s`` with a minimal
zero-g world, a ``package_box_01``-style box and a minimal drone that loads the
real ``libdrone_gripper.so``. It then does grab -> mid-carry reset -> respawn ->
re-grab and asserts the box *physically* follows the drone (not merely that the
``attached`` topic is true).

Requirements (skipped otherwise)
--------------------------------
* ``gz`` (Gazebo Harmonic) on ``PATH`` plus its Python transport bindings.
* ``rclpy`` (ROS 2) importable.
* ``libdrone_gripper.so`` built. Point ``DRONE_GRIPPER_LIB`` at its directory
  (the one containing the ``.so``), otherwise common colcon install dirs are
  searched.
* Run with ``GZ_SIM_INTEGRATION=1`` to opt in.

Usage
-----
    GZ_SIM_INTEGRATION=1 DRONE_GRIPPER_LIB=$PWD/install/drone_gripper/lib \
        pytest -s tests/integration/test_drone_gripper_regrab.py

    # or directly:
    GZ_SIM_INTEGRATION=1 ... python3 tests/integration/test_drone_gripper_regrab.py
"""

import os
import signal
import subprocess
import sys
import tempfile
import threading
import time


BOX_MODEL = "package_box_01"
SPAWN = (-1.0, -1.0, 0.25)

WORLD_SDF = """<?xml version="1.0"?>
<sdf version="1.8">
  <world name="default">
    <gravity>0 0 0</gravity>
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
        </collision>
      </link>
    </model>

    <model name="package_box_01">
      <static>false</static>
      <link name="main">
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0004</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.0004</iyy><iyz>0</iyz><izz>0.0004</izz>
          </inertia>
        </inertial>
        <collision name="col">
          <geometry><box><size>0.14 0.12 0.11</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>0.14 0.12 0.11</size></box></geometry>
        </visual>
      </link>
      <pose>-1 -1 0.061 0 0 0</pose>
    </model>
  </world>
</sdf>
"""

DRONE_SDF = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="drone">
    <link name="base_link">
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.1</iyy><iyz>0</iyz><izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name="col"><geometry><box><size>0.5 0.5 0.1</size></box></geometry></collision>
      <visual name="vis"><geometry><box><size>0.5 0.5 0.1</size></box></geometry></visual>
    </link>
    <plugin filename="libdrone_gripper.so" name="drone_gripper::DroneGripper">
      <gripper_link>base_link</gripper_link>
      <attach_distance>0.2</attach_distance>
    </plugin>
  </model>
</sdf>
"""


def _find_plugin_dir():
    env = os.environ.get("DRONE_GRIPPER_LIB")
    if env and os.path.isfile(os.path.join(env, "libdrone_gripper.so")):
        return env
    roots = [
        os.getcwd(),
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    ]
    for root in roots:
        for sub in ("install/drone_gripper/lib", "build/drone_gripper",
                    "build/drone_gripper/lib"):
            cand = os.path.join(root, sub)
            if os.path.isfile(os.path.join(cand, "libdrone_gripper.so")):
                return cand
    return None


def _preflight():
    if os.environ.get("GZ_SIM_INTEGRATION") != "1":
        return "set GZ_SIM_INTEGRATION=1 to run the Gazebo integration test"
    if subprocess.call(["bash", "-lc", "command -v gz"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
        return "gz (Gazebo Harmonic) not found on PATH"
    if _find_plugin_dir() is None:
        return "libdrone_gripper.so not found; set DRONE_GRIPPER_LIB"
    try:
        import rclpy  # noqa: F401
        import rclpy.node  # noqa: F401
        from gz.transport13 import Node  # noqa: F401
        from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        return "missing python deps (%s)" % exc
    return None


class _Harness:
    def __init__(self, tmpdir):
        self.tmpdir = tmpdir
        self.proc = None
        self.node = None
        self.poses = {}
        self.attached = None
        self._thread = None

    def start(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                               QoSProfile, QoSReliabilityPolicy)
        from std_msgs.msg import Bool, String
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V

        world = os.path.join(self.tmpdir, "world.sdf")
        with open(world, "w") as fh:
            fh.write(WORLD_SDF)
        self.drone_sdf = os.path.join(self.tmpdir, "drone.sdf")
        with open(self.drone_sdf, "w") as fh:
            fh.write(DRONE_SDF)

        plugin_dir = _find_plugin_dir()
        env = dict(os.environ)
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = plugin_dir + os.pathsep + \
            env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        env["LD_LIBRARY_PATH"] = plugin_dir + os.pathsep + \
            env.get("LD_LIBRARY_PATH", "")
        # Redirect server output to a file rather than a pipe: gz sim is
        # verbose and an undrained pipe fills up and blocks the server (which
        # would look like a hang, not a crash).
        self.log_path = os.path.join(self.tmpdir, "gzserver.log")
        self.log_file = open(self.log_path, "wb")
        self.proc = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "-v3", world],
            stdout=self.log_file, stderr=subprocess.STDOUT,
            env=env, preexec_fn=os.setsid)

        class T(Node):
            def __init__(self):
                super().__init__("drone_gripper_regrab_test")
                self.attached = None
                latched = QoSProfile(
                    depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
                self.mag = self.create_publisher(Bool, "/drone/gripper/magnet", 10)
                self.grasp = self.create_publisher(
                    String, "/drone/gripper/graspable", latched)
                self.create_subscription(
                    Bool, "/drone/gripper/attached", self._a, 10)

            def _a(self, m):
                self.attached = m.data

        rclpy.init()
        self.node = T()
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, "/world/default/pose/info", self._p)
        self._stop = threading.Event()

        def _spin():
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.05)

        self._thread = threading.Thread(target=_spin, daemon=True)
        self._thread.start()

    def _p(self, m):
        for p in m.pose:
            self.poses[p.name] = (p.position.x, p.position.y, p.position.z)

    def req(self, service, request, req_type, timeout=10000):
        from gz.msgs10.boolean_pb2 import Boolean
        res = self.gz.request(service, request, req_type, Boolean, timeout)
        return res[1] if isinstance(res, tuple) else res

    def control(self, **kwargs):
        from gz.msgs10.world_control_pb2 import WorldControl
        wc = WorldControl()
        for k, v in kwargs.items():
            getattr(wc, k).CopyFrom(v) if hasattr(v, "CopyFrom") \
                else setattr(wc, k, v)
        return self.req("/world/default/control", wc, WorldControl)

    def remove_drone(self):
        from gz.msgs10.entity_pb2 import Entity
        e = Entity(name="drone", type=Entity.MODEL)
        return self.req("/world/default/remove", e, Entity, 5000)

    def spawn_drone(self):
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        sdf = open(self.drone_sdf).read()
        r = EntityFactory(name="drone", sdf=sdf, allow_renaming=False)
        r.pose.position.x, r.pose.position.y, r.pose.position.z = SPAWN
        self.req("/world/default/create/blocking", r, EntityFactory, 15000)

    def set_drone_z(self, z):
        from gz.msgs10.pose_pb2 import Pose
        r = Pose(name="drone")
        r.position.x, r.position.y, r.position.z = SPAWN[0], SPAWN[1], z
        self.req("/world/default/set_pose/blocking", r, Pose)

    def wait(self, pred, timeout=20):
        end = time.time() + timeout
        while time.time() < end:
            if pred():
                return True
            time.sleep(0.1)
        return False

    def server_alive(self):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.world_control_pb2 import WorldControl
        try:
            res = self.gz.request("/world/default/control", WorldControl(),
                                  WorldControl, Boolean, 4000)
        except Exception:  # noqa: BLE001
            return False
        return isinstance(res, tuple) and res[0]

    def read_log(self, tail=4000):
        try:
            with open(self.log_path, "r", errors="replace") as fh:
                return "---- gzserver log (tail) ----\n" + fh.read()[-tail:]
        except Exception:  # noqa: BLE001
            return ""

    def arm_and_grab(self, label):
        from std_msgs.msg import Bool, String
        self.node.grasp.publish(String(data=BOX_MODEL))
        for _ in range(10):
            self.node.mag.publish(Bool(data=True))
            time.sleep(0.05)
        return self.wait(lambda: self.node.attached is True, 15)

    def stop(self):
        try:
            import rclpy
            self._stop.set()
            rclpy.shutdown()
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=5)
        except Exception:  # noqa: BLE001
            pass
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:  # noqa: BLE001
                self.proc.kill()
        if self.proc:
            try:
                self.log_file.close()
                with open(self.log_path, "rb") as fh:
                    self.server_log = fh.read().decode("utf-8", "replace")
            except Exception:  # noqa: BLE001
                self.server_log = ""


def run_regrab():
    """Returns (passed: bool, detail: str). Assumes _preflight() is clean."""
    tmpdir = tempfile.mkdtemp(prefix="drone_gripper_regrab_")
    h = _Harness(tmpdir)
    try:
        h.start()
        if not h.wait(lambda: BOX_MODEL in h.poses, 30):
            return False, "box model never appeared in /world/default/pose/info"
        if "drone" not in h.poses:
            h.spawn_drone()
            if not h.wait(lambda: "drone" in h.poses, 20):
                return False, "could not spawn the drone"
        time.sleep(2)

        if not h.arm_and_grab("setup"):
            return False, "initial grab did not attach"

        box_z0 = h.poses.get(BOX_MODEL, (0, 0, 0))[2]

        # Mid-carry reset, mirroring manager.py::reset_sim().
        from gz.msgs10.world_reset_pb2 import WorldReset
        h.control(pause=True)
        h.remove_drone()
        h.control(pause=True, reset=WorldReset(all=True))
        time.sleep(1.5)
        if not h.server_alive():
            return False, "gzserver died during the mid-carry reset\n" + \
                h.read_log()

        # Respawn and re-grab.
        h.spawn_drone()
        if not h.wait(lambda: "drone" in h.poses, 20):
            return False, "could not respawn the drone"
        time.sleep(2)
        if not h.arm_and_grab("re-grab"):
            return False, "re-grab did not report attached after reset"

        # Enforce the constraint physically: lift the drone, the box must follow.
        time.sleep(1.0)
        h.set_drone_z(SPAWN[2] + 0.6)
        time.sleep(1.5)
        box_z1 = h.poses.get(BOX_MODEL, (0, 0, -9))[2]
        if box_z1 <= box_z0 + 0.3:
            return False, ("attached reported true but the box did not follow "
                           "(box z %.3f -> %.3f): the physics constraint is "
                           "not enforced" % (box_z0, box_z1))
        return True, "box followed the drone after reset (z %.3f -> %.3f)" % (
            box_z0, box_z1)
    finally:
        h.stop()


def test_drone_gripper_regrab_after_midcarry_reset():
    reason = _preflight()
    if reason:
        try:
            import pytest
            pytest.skip(reason)
        except ImportError:
            print("SKIP:", reason)
            return
    passed, detail = run_regrab()
    assert passed, detail


if __name__ == "__main__":
    reason = _preflight()
    if reason:
        print("SKIP:", reason)
        sys.exit(0)
    ok, detail = run_regrab()
    print(("PASS" if ok else "FAIL") + ":", detail)
    sys.exit(0 if ok else 1)
