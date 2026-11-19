
import csv
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import ClockType
from tf2_ros import Buffer, TransformListener, TransformException

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def extract_pose(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    return t.x, t.y, yaw_from_quaternion(q)


def angle_error(a, b):
    difference = a - b
    return abs(math.atan2(
        math.sin(difference),
        math.cos(difference)
    ))


class ExperimentRecorder(Node):
    def __init__(self):
        super().__init__("experiment_recorder")

        self.declare_parameter("duration", 120.0)
        self.declare_parameter("position_threshold", 0.25)
        self.declare_parameter("angle_threshold", 0.25)
        self.declare_parameter("confirmation_duration", 10.0)
        self.declare_parameter("sample_period", 0.1)
        self.declare_parameter("results_dir", "results")
        self.declare_parameter("run_name", "experiment")

        self.duration = float(
            self.get_parameter("duration").value
        )
        self.position_threshold = float(
            self.get_parameter("position_threshold").value
        )
        self.angle_threshold = float(
            self.get_parameter("angle_threshold").value
        )
        self.confirmation_duration = float(
            self.get_parameter("confirmation_duration").value
        )
        self.sample_period = float(
            self.get_parameter("sample_period").value
        )
        self.results_dir = Path(
            self.get_parameter("results_dir").value
        ).expanduser()
        self.run_name = str(
            self.get_parameter("run_name").value
        )

        if self.duration <= 0 or self.sample_period <= 0:
            raise ValueError("Duration and sample_period must be positive")

        if self.confirmation_duration <= 0:
            raise ValueError("confirmation_duration must be positive")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.t0 = None
        self.last_sample_time = None
        self.good_interval_start = None
        self.convergence_time = None
        self.data = []
        self.finished = False

        self.timer = self.create_timer(
            self.sample_period,
            self.collect_sample
        )

        self.get_logger().info(
            "Recorder started. Waiting for AMCL and Ground Truth..."
        )

    def collect_sample(self):
        if self.finished:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if now <= 0:
            return

        sample_time = now - 0.2

        if sample_time <= 0:
            return

        stamp = Time(
            nanoseconds=int(sample_time * 1e9),
            clock_type=ClockType.ROS_TIME
        )

        try:
            amcl_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", stamp
            )

            gt_tf = self.tf_buffer.lookup_transform(
                "map", "base_link_gt", stamp
            )

        except TransformException:
            # Если TF пропал, непрерывность интервала
            # сходимости нельзя считать подтверждённой.
            self.good_interval_start = None
            self.last_sample_time = None
            return

        amcl_x, amcl_y, amcl_yaw = extract_pose(amcl_tf)
        gt_x, gt_y, gt_yaw = extract_pose(gt_tf)

        if self.t0 is None:
            self.t0 = sample_time
            self.get_logger().info(
                "First AMCL estimate received. Recording started."
            )

        t = sample_time - self.t0

        if t < 0:
            return

        pos_error = math.hypot(
            amcl_x - gt_x,
            amcl_y - gt_y
        )

        yaw_error = angle_error(
            amcl_yaw, gt_yaw
        )

        self.data.append({
            "time": t,
            "gt_x": gt_x,
            "gt_y": gt_y,
            "gt_yaw": gt_yaw,
            "amcl_x": amcl_x,
            "amcl_y": amcl_y,
            "amcl_yaw": amcl_yaw,
            "position_error": pos_error,
            "angle_error": yaw_error
        })

        max_gap = max(0.5, 3 * self.sample_period)

        if self.last_sample_time is not None:
            if sample_time - self.last_sample_time > max_gap:
                self.good_interval_start = None

        self.last_sample_time = sample_time

        within_thresholds = (
            pos_error <= self.position_threshold
            and yaw_error <= self.angle_threshold
        )

        if within_thresholds:
            if self.good_interval_start is None:
                self.good_interval_start = t

            elapsed_good = t - self.good_interval_start

            if (
                self.convergence_time is None
                and elapsed_good >= self.confirmation_duration
            ):
                self.convergence_time = self.good_interval_start
                self.get_logger().info(
                    f"Convergence confirmed: "
                    f"{self.convergence_time:.2f} s"
                )
        else:
            self.good_interval_start = None

        if t >= self.duration:
            self.finished = True
            self.timer.cancel()
            self.save_results()

    def save_results(self):
        self.results_dir.mkdir(
            parents=True, exist_ok=True
        )

        base = self.results_dir / self.run_name

        csv_path = base.with_suffix(".csv")

        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(
                f, fieldnames=self.data[0].keys()
            )
            writer.writeheader()
            writer.writerows(self.data)

        summary = {
            "run_name": self.run_name,
            "duration": self.duration,
            "position_threshold": self.position_threshold,
            "angle_threshold": self.angle_threshold,
            "confirmation_duration": self.confirmation_duration,
            "convergence_time": self.convergence_time,
            "converged": self.convergence_time is not None,
            "sample_count": len(self.data)
        }

        json_path = base.with_suffix(".json")
        with json_path.open("w") as f:
            json.dump(summary, f, indent=4)

        times = [r["time"] for r in self.data]
        gt_x = [r["gt_x"] for r in self.data]
        gt_y = [r["gt_y"] for r in self.data]
        amcl_x = [r["amcl_x"] for r in self.data]
        amcl_y = [r["amcl_y"] for r in self.data]
        pos_errors = [r["position_error"] for r in self.data]
        yaw_errors = [r["angle_error"] for r in self.data]

        fig, axes = plt.subplots(
            3, 1, figsize=(10, 13)
        )

        axes[0].plot(
            gt_x, gt_y, label="Ground Truth",
            linewidth=2
        )
        axes[0].plot(
            amcl_x, amcl_y, label="AMCL",
            linestyle="--"
        )
        axes[0].set_title("Robot trajectories")
        axes[0].set_xlabel("X (m)")
        axes[0].set_ylabel("Y (m)")
        axes[0].axis("equal")
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(
            times, pos_errors,
            label="Position error"
        )
        axes[1].axhline(
            self.position_threshold,
            color="red",
            linestyle="--",
            label="Threshold"
        )
        axes[1].set_title("Position error")
        axes[1].set_xlabel("Time from t0 (s)")
        axes[1].set_ylabel("Position error (m)")
        axes[1].grid(True)

        axes[2].plot(
            times, yaw_errors,
            label="Yaw error"
        )
        axes[2].axhline(
            self.angle_threshold,
            color="red",
            linestyle="--",
            label="Threshold"
        )
        axes[2].set_title("Yaw error")
        axes[2].set_xlabel("Time from t0 (s)")
        axes[2].set_ylabel("Yaw error (rad)")
        axes[2].grid(True)

        if self.convergence_time is not None:
            for ax in axes[1:]:
                ax.axvline(
                    self.convergence_time,
                    color="green",
                    linestyle=":",
                    label=f"Convergence: {self.convergence_time:.2f} s"
                )
        else:
            axes[1].set_title(
                "Position error (convergence not confirmed)"
            )

        axes[1].legend()
        axes[2].legend()

        fig.suptitle(self.run_name)
        fig.tight_layout()

        png_path = base.with_suffix(".png")
        fig.savefig(png_path, dpi=160)
        plt.close(fig)

        self.get_logger().info(
            f"Results saved to: {self.results_dir}"
        )

        if self.convergence_time is None:
            self.get_logger().warn(
                "Convergence was not confirmed."
            )
        else:
            self.get_logger().info(
                f"Convergence time: {self.convergence_time:.2f} s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
