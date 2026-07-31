#!/usr/bin/env python3
import os
import shlex
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "start_d_task_uav.sh"


class StartDTaskUavScriptTest(unittest.TestCase):
    def write_executable(self, path, content):
        path.write_text(content, encoding="utf-8")
        path.chmod(0o755)

    def test_help_describes_safe_startup(self):
        result = subprocess.run(
            ["bash", str(SCRIPT_PATH), "--help"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("等待小车合法指令", result.stdout)
        self.assertIn("--mqtt-host", result.stdout)
        self.assertIn("--video-device", result.stdout)
        self.assertIn("--mavros-fcu-url", result.stdout)
        self.assertIn("--dry-run", result.stdout)
        self.assertIn("mission_topics", result.stdout)
        self.assertIn("轻量 rosbag", result.stdout)

    def test_script_is_executable_and_installed_by_catkin(self):
        self.assertTrue(os.access(str(SCRIPT_PATH), os.X_OK))
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        python_install_block = cmake.split("catkin_install_python(PROGRAMS", 1)[1].split(
            "DESTINATION", 1
        )[0]
        self.assertNotIn("scripts/start_d_task_uav.sh", python_install_block)
        self.assertIn(
            "install(PROGRAMS\n  scripts/start_d_task_uav.sh",
            cmake,
        )

    def test_dry_run_preserves_start_order_and_never_triggers_takeoff(self):
        result = subprocess.run(
            [
                "bash",
                str(SCRIPT_PATH),
                "--dry-run",
                "--mqtt-host",
                "10.0.0.8",
                "--mqtt-port",
                "2883",
                "--video-device",
                "/dev/video9",
                "--mavros-fcu-url",
                "serial:///dev/ttyACM0:921600",
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        self.assertEqual(result.returncode, 0, result.stdout)
        expected_commands = [
            "roscore",
            "roslaunch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:921600",
            "roslaunch livox_ros_driver2 msg_MID360s.launch",
            "roslaunch d_task_uav_control d_task_uav.launch "
            "mqtt_host:=10.0.0.8 mqtt_port:=2883 video_device:=/dev/video9",
            "rosbag record",
        ]
        offsets = [result.stdout.index(command) for command in expected_commands]
        self.assertEqual(offsets, sorted(offsets), result.stdout)
        self.assertNotIn("rostopic pub", result.stdout)
        self.assertIn("mission_topics.bag", result.stdout)
        expected_topics = {
            "/uav_protocol/mission_config",
            "/uav_protocol/mission_start",
            "/uav_protocol/task_state",
            "/uav_protocol/local_event",
            "/uav_protocol/local_fault",
            "/uav_protocol/local_health",
            "/uav_protocol/dynamic_descent_allowed",
            "/uav_protocol/safety_hold",
            "/d_task/positioning/status",
            "/mavros/state",
            "/mavros/local_position/odom",
            "/ego_bridge/flight_state",
            "/ego_bridge/control_mode",
            "/ego_bridge/override_cmd",
            "/d_task/vision/platform_detection",
            "/d_task/vision/apriltag_range",
            "/d_task/tracking/platform_estimate",
            "/uav_protocol/car/pose",
            "/uav_protocol/car/pose_meta",
        }
        rosbag_line = next(
            line for line in result.stdout.splitlines() if "rosbag record" in line
        )
        rosbag_args = shlex.split(rosbag_line)[3:]
        self.assertIn("--split", rosbag_args)
        self.assertIn("--size=256", rosbag_args)
        output_index = rosbag_args.index("-O")
        recorded_topics = set(rosbag_args[output_index + 2 :])
        self.assertEqual(recorded_topics, expected_topics)
        for excluded_topic in ("image_raw", "apriltag_debug", "cloud_registered"):
            self.assertNotIn(excluded_topic, result.stdout)

    def test_starts_stack_and_cleans_up_owned_processes(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            bin_dir = root / "bin"
            state_dir = root / "state"
            log_dir = root / "logs"
            bin_dir.mkdir()
            state_dir.mkdir()
            camera = root / "video0"
            camera.touch()

            for setup_name in ("ros_setup", "positioning_setup"):
                (root / setup_name).write_text(":\n", encoding="utf-8")
            (root / "catkin_setup").write_text(
                'printf "%s\\n" "$*" > "$FAKE_ROS_STATE/catkin_setup_args"\n',
                encoding="utf-8",
            )

            self.write_executable(
                bin_dir / "roscore",
                """#!/usr/bin/env bash
touch "$FAKE_ROS_STATE/master"
cleanup() { rm -f "$FAKE_ROS_STATE/master"; exit 0; }
trap cleanup INT TERM
while true; do sleep 0.05; done
""",
            )
            self.write_executable(
                bin_dir / "roslaunch",
                """#!/usr/bin/env bash
echo "$*" >> "$FAKE_ROS_STATE/commands"
case "$1" in
  mavros) marker=mavros ;;
  livox_ros_driver2) marker=livox ;;
  d_task_uav_control) marker=backend ;;
  *) exit 64 ;;
esac
if test "${FAKE_FAIL_COMPONENT:-}" = "$marker"; then
  echo "simulated $marker launch failure" >&2
  exit 1
fi
touch "$FAKE_ROS_STATE/$marker"
if test "${FAKE_EXIT_COMPONENT:-}" = "$marker"; then
  sleep 0.2
  rm -f "$FAKE_ROS_STATE/$marker"
  exit 7
fi
cleanup() { rm -f "$FAKE_ROS_STATE/$marker"; exit 0; }
trap cleanup INT TERM
while true; do sleep 0.05; done
""",
            )
            self.write_executable(
                bin_dir / "rosnode",
                """#!/usr/bin/env bash
case "$1" in
  list)
    test -e "$FAKE_ROS_STATE/master" || exit 1
    test ! -e "$FAKE_ROS_STATE/mavros" || echo /mavros
    test ! -e "$FAKE_ROS_STATE/livox" || echo /livox_lidar_publisher2
    test ! -e "$FAKE_ROS_STATE/backend" || echo /d_task_mission
    ;;
  ping)
    node="${@: -1}"
    case "$node" in
      /mavros) marker=mavros ;;
      /livox_lidar_publisher2) marker=livox ;;
      /d_task_mission) marker=backend ;;
      *) exit 1 ;;
    esac
    if test -e "$FAKE_ROS_STATE/$marker"; then
      echo "xmlrpc reply from http://fake:12345 time=1ms"
    else
      echo "cannot ping [$node]: unknown node" >&2
    fi
    ;;
  *) exit 2 ;;
esac
""",
            )
            self.write_executable(
                bin_dir / "rostopic",
                """#!/usr/bin/env bash
test "${@: -1}" = /mavros/state || exit 1
echo 'connected: True'
""",
            )
            self.write_executable(
                bin_dir / "rosbag",
                """#!/usr/bin/env bash
test "$1" = record || exit 64
shift
echo "$*" > "$FAKE_ROS_STATE/rosbag_command"
touch "$FAKE_ROS_STATE/rosbag"
if test "${FAKE_EXIT_ROSBAG:-}" = true; then
  rm -f "$FAKE_ROS_STATE/rosbag"
  exit 9
fi
cleanup() { rm -f "$FAKE_ROS_STATE/rosbag"; exit 0; }
trap cleanup INT TERM
while true; do sleep 0.05; done
""",
            )

            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{bin_dir}:{env['PATH']}",
                    "FAKE_ROS_STATE": str(state_dir),
                    "D_TASK_ROS_SETUP": str(root / "ros_setup"),
                    "D_TASK_POSITIONING_SETUP": str(root / "positioning_setup"),
                    "D_TASK_CATKIN_SETUP": str(root / "catkin_setup"),
                    "D_TASK_LOG_ROOT": str(log_dir),
                    "D_TASK_STARTUP_TIMEOUT_S": "2",
                    "D_TASK_SKIP_MQTT_CHECK": "true",
                }
            )
            process = subprocess.Popen(
                ["bash", str(SCRIPT_PATH), "--video-device", str(camera)],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not (state_dir / "backend").exists():
                if process.poll() is not None:
                    break
                time.sleep(0.05)

            self.assertIsNone(process.poll())
            self.assertTrue((state_dir / "backend").exists())
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not (state_dir / "rosbag").exists():
                if process.poll() is not None:
                    break
                time.sleep(0.05)
            self.assertTrue((state_dir / "rosbag").exists())
            process.send_signal(signal.SIGTERM)
            output, _ = process.communicate(timeout=5)

            self.assertEqual(process.returncode, 0, output)
            commands = (state_dir / "commands").read_text(encoding="utf-8").splitlines()
            self.assertEqual(
                [line.split()[0] for line in commands],
                ["mavros", "livox_ros_driver2", "d_task_uav_control"],
            )
            self.assertFalse((state_dir / "master").exists())
            self.assertFalse((state_dir / "mavros").exists())
            self.assertFalse((state_dir / "livox").exists())
            self.assertFalse((state_dir / "backend").exists())
            self.assertFalse((state_dir / "rosbag").exists())
            rosbag_command = (state_dir / "rosbag_command").read_text(
                encoding="utf-8"
            )
            self.assertIn("mission_topics.bag", rosbag_command)
            self.assertNotIn("image_raw", rosbag_command)
            self.assertEqual(
                (state_dir / "catkin_setup_args").read_text(encoding="utf-8").strip(),
                "--extend",
            )

            optional_env = env.copy()
            optional_env["FAKE_EXIT_ROSBAG"] = "true"
            optional = subprocess.Popen(
                ["bash", str(SCRIPT_PATH), "--video-device", str(camera)],
                env=optional_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not (state_dir / "backend").exists():
                if optional.poll() is not None:
                    break
                time.sleep(0.05)
            time.sleep(0.35)
            if optional.poll() is not None:
                optional_output, _ = optional.communicate(timeout=5)
                self.fail(
                    "rosbag failure stopped flight stack:\n" + optional_output
                )
            self.assertTrue((state_dir / "backend").exists())
            optional.send_signal(signal.SIGTERM)
            optional_output, _ = optional.communicate(timeout=5)
            self.assertEqual(optional.returncode, 0, optional_output)
            self.assertIn("警告：mission_topics 已退出", optional_output)

            (state_dir / "commands").write_text("", encoding="utf-8")
            failed_env = env.copy()
            failed_env["FAKE_FAIL_COMPONENT"] = "livox"
            failed_env["D_TASK_STARTUP_TIMEOUT_S"] = "1"
            failed = subprocess.run(
                ["bash", str(SCRIPT_PATH), "--video-device", str(camera)],
                env=failed_env,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=5,
            )

            self.assertNotEqual(failed.returncode, 0, failed.stdout)
            self.assertIn("MID360 驱动在 1s 内未就绪", failed.stdout)
            self.assertNotIn("全部软件已启动", failed.stdout)
            failed_commands = (state_dir / "commands").read_text(
                encoding="utf-8"
            ).splitlines()
            self.assertNotIn("d_task_uav_control", [
                line.split()[0] for line in failed_commands
            ])
            self.assertFalse((state_dir / "master").exists())
            self.assertFalse((state_dir / "mavros").exists())
            self.assertFalse((state_dir / "backend").exists())

            (state_dir / "commands").write_text("", encoding="utf-8")
            exited_env = env.copy()
            exited_env["FAKE_EXIT_COMPONENT"] = "backend"
            exited = subprocess.run(
                ["bash", str(SCRIPT_PATH), "--video-device", str(camera)],
                env=exited_env,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=5,
            )

            self.assertNotEqual(exited.returncode, 0, exited.stdout)
            self.assertIn("d_task_uav 异常退出，状态码：7", exited.stdout)
            self.assertNotIn("某个受管 ROS 进程", exited.stdout)


if __name__ == "__main__":
    unittest.main()
