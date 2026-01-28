# ~/Project2_M76150518/ros2_ws/src/gimbal_lock_demo/gimbal_lock_demo/rqt_gimbal_plugin.py

import math

import rclpy
from geometry_msgs.msg import Vector3, Quaternion

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QLabel, QHBoxLayout,
    QSlider, QDoubleSpinBox, QPushButton
)
from python_qt_binding.QtCore import Qt, QTimer


def deg2rad(d): 
    return d * math.pi / 180.0


def normalize_quat(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x/n, y/n, z/n, w/n


def euler_to_quat_zyx(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return normalize_quat(x, y, z, w)


class RqtGimbalPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('RqtGimbalPlugin')

        # rqt may already init rclpy; only init if not already running
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node('rqt_gimbal_plugin')

        self.pub_euler = self.node.create_publisher(Vector3, '/cmd_euler_rpy', 10)
        self.pub_quat = self.node.create_publisher(Quaternion, '/cmd_quaternion', 10)

        # Spin the node periodically (rqt does NOT automatically spin)
        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self.spin_once)
        self.spin_timer.start(50)  # 20 Hz

        # ---------- UI ----------
        self.widget = QWidget()
        layout = QVBoxLayout()

        euler_box = QGroupBox("Euler Control (roll/pitch/yaw)")
        euler_layout = QVBoxLayout()

        self.lbl_warn = QLabel("")
        euler_layout.addWidget(self.lbl_warn)

        self.sl_roll = self._slider("Roll (deg)", -180, 180, 0, euler_layout)
        self.sl_pitch = self._slider("Pitch (deg)", -90, 90, 0, euler_layout)
        self.sl_yaw = self._slider("Yaw (deg)", -180, 180, 0, euler_layout)

        btn_send_euler = QPushButton("Publish Euler")
        btn_send_euler.clicked.connect(self.send_euler)
        euler_layout.addWidget(btn_send_euler)

        btn_auto = QPushButton("Auto Gimbal Lock Demo (Extra)")
        btn_auto.clicked.connect(self.auto_demo_start)
        euler_layout.addWidget(btn_auto)

        btn_reset = QPushButton("Reset Pose (Extra)")
        btn_reset.clicked.connect(self.reset_pose)
        euler_layout.addWidget(btn_reset)

        self.lbl_quat_from_euler = QLabel("Euler->Quat: (x,y,z,w) = (0,0,0,1)")
        euler_layout.addWidget(self.lbl_quat_from_euler)

        euler_box.setLayout(euler_layout)
        layout.addWidget(euler_box)

        quat_box = QGroupBox("Quaternion Control (x,y,z,w)")
        quat_layout = QVBoxLayout()

        row = QHBoxLayout()
        self.qx = self._spin("x", row, 0.0)
        self.qy = self._spin("y", row, 0.0)
        self.qz = self._spin("z", row, 0.0)
        self.qw = self._spin("w", row, 1.0)
        quat_layout.addLayout(row)

        btn_norm = QPushButton("Normalize Quaternion")
        btn_norm.clicked.connect(self.normalize_current_quat)
        quat_layout.addWidget(btn_norm)

        btn_send_quat = QPushButton("Publish Quaternion")
        btn_send_quat.clicked.connect(self.send_quat)
        quat_layout.addWidget(btn_send_quat)

        quat_box.setLayout(quat_layout)
        layout.addWidget(quat_box)

        self.widget.setLayout(layout)
        context.add_widget(self.widget)

        # Slider hooks
        self.sl_roll.valueChanged.connect(self.update_quat_label)
        self.sl_pitch.valueChanged.connect(self.update_warn)
        self.sl_yaw.valueChanged.connect(self.update_quat_label)

        # Auto demo timer
        self.demo_timer = QTimer()
        self.demo_timer.timeout.connect(self.auto_demo_step)
        self.demo_phase = 0
        self.demo_value = -60

        self.update_quat_label()

    def spin_once(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            pass

    def shutdown_plugin(self):
        # Stop timers
        try:
            self.demo_timer.stop()
        except Exception:
            pass
        try:
            self.spin_timer.stop()
        except Exception:
            pass

        # Destroy node (do NOT call rclpy.shutdown() inside rqt)
        try:
            self.node.destroy_node()
        except Exception:
            pass

    def _slider(self, title, mn, mx, val, parent):
        lbl = QLabel(f"{title}: {val}")
        s = QSlider(Qt.Horizontal)
        s.setMinimum(mn); s.setMaximum(mx); s.setValue(val)
        s.valueChanged.connect(lambda v: lbl.setText(f"{title}: {v}"))
        parent.addWidget(lbl); parent.addWidget(s)
        return s

    def _spin(self, name, row, init):
        box = QDoubleSpinBox()
        box.setDecimals(6)
        box.setRange(-1.5, 1.5)
        box.setSingleStep(0.01)
        box.setValue(init)
        row.addWidget(QLabel(name))
        row.addWidget(box)
        return box

    def update_warn(self):
        p = self.sl_pitch.value()
        if abs(abs(p) - 90) <= 2:
            self.lbl_warn.setText("WARNING: Near gimbal lock (pitch ≈ ±90°). Yaw and roll become coupled.")
        else:
            self.lbl_warn.setText("")
        self.update_quat_label()

    def update_quat_label(self):
        r = deg2rad(self.sl_roll.value())
        p = deg2rad(self.sl_pitch.value())
        y = deg2rad(self.sl_yaw.value())
        qx, qy, qz, qw = euler_to_quat_zyx(r, p, y)
        self.lbl_quat_from_euler.setText(
            f"Euler->Quat: (x,y,z,w)=({qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f})"
        )

    def send_euler(self):
        msg = Vector3()
        msg.x = deg2rad(self.sl_roll.value())
        msg.y = deg2rad(self.sl_pitch.value())
        msg.z = deg2rad(self.sl_yaw.value())
        self.pub_euler.publish(msg)

    def normalize_current_quat(self):
        x, y, z, w = normalize_quat(self.qx.value(), self.qy.value(), self.qz.value(), self.qw.value())
        self.qx.setValue(x); self.qy.setValue(y); self.qz.setValue(z); self.qw.setValue(w)

    def send_quat(self):
        x, y, z, w = normalize_quat(self.qx.value(), self.qy.value(), self.qz.value(), self.qw.value())
        msg = Quaternion()
        msg.x = x; msg.y = y; msg.z = z; msg.w = w
        self.pub_quat.publish(msg)

    def reset_pose(self):
        self.sl_roll.setValue(0)
        self.sl_pitch.setValue(0)
        self.sl_yaw.setValue(0)
        self.send_euler()

    def auto_demo_start(self):
        self.demo_phase = 0
        self.demo_value = -60
        self.sl_pitch.setValue(90)
        self.demo_timer.start(150)

    def auto_demo_step(self):
        if self.demo_phase == 0:
            self.sl_yaw.setValue(self.demo_value)
        else:
            self.sl_roll.setValue(self.demo_value)

        self.send_euler()

        self.demo_value += 5
        if self.demo_value > 60:
            self.demo_value = -60
            self.demo_phase += 1
            if self.demo_phase > 1:
                self.demo_timer.stop()

