#!/usr/bin/env python3

import math
import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


TARGET_POSE_TOPIC = "/unitree_moveit_operator/target_pose"
STATUS_TOPIC = "/unitree_moveit_operator/status"
RESET_ZERO_SERVICE = "/unitree_moveit_operator/reset_zero"
ENABLE_MOTION_SERVICE = "/unitree_moveit_operator/enable_motion"
DISABLE_MOTION_SERVICE = "/unitree_moveit_operator/disable_motion"


def quaternion_from_rpy(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return (
        (sr * cp * cy) - (cr * sp * sy),
        (cr * sp * cy) + (sr * cp * sy),
        (cr * cp * sy) - (sr * sp * cy),
        (cr * cp * cy) + (sr * sp * sy),
    )


class UnitreeMoveItGui:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("my_controller_gui")
        self.pose_pub = self.node.create_publisher(PoseStamped, TARGET_POSE_TOPIC, 10)
        self.status_sub = self.node.create_subscription(
            String, STATUS_TOPIC, self._on_status, 10
        )
        self.reset_client = self.node.create_client(Trigger, RESET_ZERO_SERVICE)
        self.enable_motion_client = self.node.create_client(Trigger, ENABLE_MOTION_SERVICE)
        self.disable_motion_client = self.node.create_client(Trigger, DISABLE_MOTION_SERVICE)
        self.status_queue = queue.Queue()

        self.root = tk.Tk()
        self.root.title("Unitree MoveIt Operator")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.entries = {}
        self._build_form()

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()
        self.root.after(100, self._pump_status_queue)

    def _build_form(self):
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        defaults = {
            "x": "0.20",
            "y": "0.00",
            "z": "0.20",
            "roll": "0.00",
            "pitch": "0.00",
            "yaw": "0.00",
        }

        for row, (name, default) in enumerate(defaults.items()):
            ttk.Label(frame, text=name).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=3)
            entry = ttk.Entry(frame, width=16)
            entry.insert(0, default)
            entry.grid(row=row, column=1, sticky="ew", pady=3)
            self.entries[name] = entry

        button_frame = ttk.Frame(frame)
        button_frame.grid(row=len(defaults), column=0, columnspan=2, sticky="ew", pady=(10, 6))
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)

        ttk.Button(button_frame, text="Send Pose", command=self.publish_pose).grid(
            row=0, column=0, sticky="ew", padx=(0, 4)
        )
        ttk.Button(button_frame, text="Reset Zero", command=self.reset_zero).grid(
            row=0, column=1, sticky="ew", padx=(4, 0)
        )

        arm_frame = ttk.Frame(frame)
        arm_frame.grid(row=len(defaults) + 1, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        arm_frame.columnconfigure(0, weight=1)
        arm_frame.columnconfigure(1, weight=1)

        ttk.Button(arm_frame, text="Enable Motion", command=self.enable_motion).grid(
            row=0, column=0, sticky="ew", padx=(0, 4)
        )
        ttk.Button(arm_frame, text="Disable Motion", command=self.disable_motion).grid(
            row=0, column=1, sticky="ew", padx=(4, 0)
        )

        self.status_var = tk.StringVar(value="waiting for status")
        ttk.Label(frame, textvariable=self.status_var, anchor="w").grid(
            row=len(defaults) + 2, column=0, columnspan=2, sticky="ew", pady=(6, 0)
        )

    def _read_float(self, name):
        return float(self.entries[name].get())

    def publish_pose(self):
        try:
            x = self._read_float("x")
            y = self._read_float("y")
            z = self._read_float("z")
            roll = self._read_float("roll")
            pitch = self._read_float("pitch")
            yaw = self._read_float("yaw")
        except ValueError:
            self.status_var.set("invalid numeric input")
            return

        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)

        message = PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw

        self.pose_pub.publish(message)
        self.status_var.set("pose sent")

    def reset_zero(self):
        confirmed = messagebox.askyesno(
            "Confirm Reset Zero",
            "Command reset_zero motion to the zero joint target?",
        )
        if not confirmed:
            self.status_var.set("reset canceled")
            return

        if not self.reset_client.wait_for_service(timeout_sec=0.1):
            self.status_var.set("reset_zero service unavailable")
            return

        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_reset_done)
        self.status_var.set("reset requested")

    def enable_motion(self):
        self._call_trigger_service(
            self.enable_motion_client,
            "enable_motion service unavailable",
            "enable motion requested",
            self._on_motion_service_done,
        )

    def disable_motion(self):
        self._call_trigger_service(
            self.disable_motion_client,
            "disable_motion service unavailable",
            "disable motion requested",
            self._on_motion_service_done,
        )

    def _call_trigger_service(self, client, unavailable_message, pending_message, callback):
        if not client.wait_for_service(timeout_sec=0.1):
            self.status_var.set(unavailable_message)
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(callback)
        self.status_var.set(pending_message)

    def _on_status(self, message):
        self.status_queue.put(message.data)

    def _on_reset_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.status_queue.put(response.message)
            else:
                self.status_queue.put("reset failed: " + response.message)
        except Exception as exception:  # pylint: disable=broad-except
            self.status_queue.put("reset failed: " + str(exception))

    def _on_motion_service_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.status_queue.put(response.message)
            else:
                self.status_queue.put("motion service failed: " + response.message)
        except Exception as exception:  # pylint: disable=broad-except
            self.status_queue.put("motion service failed: " + str(exception))

    def _pump_status_queue(self):
        try:
            while True:
                self.status_var.set(self.status_queue.get_nowait())
        except queue.Empty:
            pass

        self.root.after(100, self._pump_status_queue)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    app = UnitreeMoveItGui()
    app.run()


if __name__ == "__main__":
    main()
