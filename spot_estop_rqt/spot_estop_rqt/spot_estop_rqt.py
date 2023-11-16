#!/usr/bin/env python3

import sys
import os
import subprocess
from functools import partial

import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from spot_msgs.msg import BatteryStateArray
from composition_interfaces.srv import LoadNode, UnloadNode, ListNodes
from rcl_interfaces.msg import Parameter

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QScrollBar,
    QSlider,
)

from rqt_gui.main import Main

from spot_estop_rqt.moveit_action_client import MoveGroupActionClient

MAX_IP = os.getenv("MAX_IP")
MAX_USER = os.getenv("MAX_USER")
if MAX_IP is None:
    print("MAX_IP environment variable is not set.")
    exit()
if MAX_USER is None:
    print("MAX_USER environment variable is not set.")
    exit()


class EstopRqtPlugin(Plugin):
    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self._context = context

        ssh_commands = [
            f"ssh {MAX_USER}@{MAX_IP}",
            f'sleep 5 && ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p audio -X stuff \\"ros2 run rclcpp_components component_container --ros-args -r __node:=NucAudio\\\\n\\""',
        ]
        for ssh_command in ssh_commands:
            subprocess.Popen(ssh_command, shell=True)

        # Create the publisher for controlling lights
        self.lights_pub = self._context.node.create_publisher(
            String, "lights_camera", QoSProfile(depth=1)
        )

        # Create the subscriber for status updates
        self._context.node.create_subscription(
            BatteryStateArray, "/status/battery_states", self.status_callback, 1
        )

        # Create service clients
        self.map_frame_reset_client = self._context.node.create_client(
            Trigger, "/reset_map_frame"
        )
        self.path_reset_client = self._context.node.create_client(
            Empty, "/reset_travelled_path"
        )
        self.world_info_reset_client = self._context.node.create_client(
            Empty, "/reset_world_info"
        )
        self.octomap_reset_client = self._context.node.create_client(
            Empty, "/octomap_server/reset"
        )

        # Run "ros2 run rclcpp_components component_container --ros-args -r __node:=OperatorAudio" in operator's terminal
        self.load_audio_operator_node = self._context.node.create_client(
            LoadNode, "/OperatorAudio/_container/load_node"
        )
        self.unload_audio_operator_node = self._context.node.create_client(
            UnloadNode, "/OperatorAudio/_container/unload_node"
        )
        self.list_audio_operator_nodes = self._context.node.create_client(
            ListNodes, "/OperatorAudio/_container/list_nodes"
        )

        # Run "ros2 run rclcpp_components component_container --ros-args -r __node:=NucAudio" in nuc's terminal
        # Run "pulseaudio --daemonize" if NUC is connected in ssh
        self.load_audio_nuc_node = self._context.node.create_client(
            LoadNode, "/NucAudio/_container/load_node"
        )
        self.unload_audio_nuc_node = self._context.node.create_client(
            UnloadNode, "/NucAudio/_container/unload_node"
        )
        self.list_audio_nuc_nodes = self._context.node.create_client(
            ListNodes, "/NucAudio/_container/list_nodes"
        )

        # Create the main widget and set up the layout
        self.widget = QWidget()
        layout = QVBoxLayout()
        self.widget.setLayout(layout)

        # Set minimum size for the widget
        self.widget.setMinimumSize(200, 200)

        ## TABS
        # Create the tab widget
        tab_widget = QTabWidget()

        # Create the buttons and layouts for each tab
        first_tab_layout = QVBoxLayout()
        second_tab_layout = QVBoxLayout()
        third_tab_layout = QVBoxLayout()

        ### FIRST TAB
        ## BUTTONS

        # Create a horizontal slider for Stop
        self.stop_slider = QSlider(Qt.Horizontal)
        self.stop_slider.setRange(0, 100)

        # Set the background color of the groove to red and add rounded corners
        self.stop_slider.setStyleSheet(
            """
            QSlider {
                min-height: 120px;
                max-height: 120px;
            }
            
            QSlider::groove:horizontal {
                background: red;
                height: 15px;
                border-radius: 5px;
                margin: 20px;
            }
            
            QSlider::handle:horizontal {
                background: white;
                width: 50px;
                border-radius: 25px;
                margin: -20px 0;
                subcontrol-origin: margin;
                subcontrol-position: center center;
            }
        """
        )

        # Connect the slider's sliderReleased signal to a slot (function)
        self.stop_slider.sliderReleased.connect(self.stop_slider_released)

        # Add a QLabel for the text on the handle
        stop_label = QLabel("STOP")
        stop_label.setAlignment(Qt.AlignCenter)
        stop_label.setStyleSheet(
            "font: bold 20px; border-width: 5px; border-radius:20px; padding: 30px;"
        )

        # Create the buttons for sending commands
        # stop_button = QPushButton('STOP')
        # stop_button.clicked.connect(self.send_stop_command)
        # stop_button.setStyleSheet("background-color: red; font: bold 20px; border-width: 5px; border-radius:20px; padding: 20px")

        power_on = QPushButton("Power On")
        power_on.clicked.connect(self.send_power_on_command)
        power_on.setStyleSheet(
            "background-color: green; font: bold 20px; border-width: 5px; border-radius:20px; padding: 20px"
        )

        estop_button = QPushButton("Spot")
        estop_button.clicked.connect(self.run_ssh_command)
        estop_button.setStyleSheet(
            "background-color: blue; font: bold 20px; border-width: 5px; border-radius:20px; padding: 20px"
        )

        # Create the label for displaying status
        self.status_label = QLabel("Remaining: Unknown")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font: bold 16px")

        first_row_buttons = QHBoxLayout()
        first_row_buttons.addWidget(stop_label)
        first_row_buttons.addWidget(self.stop_slider)

        second_row_buttons = QHBoxLayout()
        second_row_buttons.addWidget(power_on)
        second_row_buttons.addWidget(estop_button)

        first_tab_layout.addLayout(first_row_buttons)
        first_tab_layout.addLayout(second_row_buttons)
        first_tab_layout.addWidget(self.status_label)

        ### SECOND TAB

        hazmat_button = QPushButton("Hazmat")
        hazmat_button.clicked.connect(self.hazmat_command)
        hazmat_button.setStyleSheet(
            "background-color: pink; font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px"
        )

        world_reset_button = QPushButton("World Reset")
        world_reset_button.clicked.connect(self.world_reset_command)
        world_reset_button.setStyleSheet(
            "background-color: pink; font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px"
        )

        # Create a toggle button for audio communication
        two_way_audio = QPushButton("2WayAudio")
        two_way_audio.setCheckable(True)
        two_way_audio.setStyleSheet(
            "QPushButton { font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px }"
            "QPushButton:checked { background-color: green }"
            "QPushButton:unchecked { background-color: transparent }"
        )
        two_way_audio.clicked.connect(partial(self.toggle_two_way_audio, two_way_audio))

        ## SCROLLBARS
        # Create scroll bars for lights
        front_light = QScrollBar(Qt.Horizontal)
        left_light = QScrollBar(Qt.Horizontal)
        right_light = QScrollBar(Qt.Horizontal)

        ## Create a toggle button for running/stopping gologpp frame runner
        self.toggle_button = QPushButton("Frame Runner")
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(False)  # Set the initial state
        self.toggle_button.toggled.connect(self.toggle_frame_runner)
        self.toggle_button.setStyleSheet(
            "background-color: transparent; font: bold 15px; border-width: 5px; border-radius: 15px; padding: 15px"
        )
        second_tab_layout.addWidget(self.toggle_button)

        # Connect callbacks to scroll bars
        front_light.valueChanged.connect(self.front_light_value_change_callback)
        left_light.valueChanged.connect(self.left_light_value_change_callback)
        right_light.valueChanged.connect(self.right_light_value_change_callback)

        front_light.sliderReleased.connect(self.front_light_callback)
        left_light.sliderReleased.connect(self.left_light_callback)
        right_light.sliderReleased.connect(self.right_light_callback)

        # Add scroll areas to the layout
        second_tab_layout.addWidget(front_light)
        second_tab_layout.addWidget(left_light)
        second_tab_layout.addWidget(right_light)

        row_buttons = QHBoxLayout()
        row_buttons.addWidget(hazmat_button)
        row_buttons.addWidget(world_reset_button)
        row_buttons.addWidget(two_way_audio)

        second_tab_layout.addLayout(row_buttons)

        ### THIRD TAB (BLOCKSWORLD)
        # Create a toggle button to scan block positions
        self.toggle_blocks_scan = QPushButton("Scan Blocks")
        self.toggle_blocks_scan.setCheckable(True)
        self.toggle_blocks_scan.setChecked(False)  # Set the initial state
        self.toggle_blocks_scan.toggled.connect(self.blocks_scan)
        self.toggle_blocks_scan.setStyleSheet(
            "background-color: transparent; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
        )
        third_tab_layout.addWidget(self.toggle_blocks_scan)
        self.blocks_scan_checked = False

        self._context.node.create_subscription(
            String, "/block_loc", self.block_loc_update, 1
        )

        # Create a label for displaying status
        self.bw_status_label = QLabel("Unknown")
        self.bw_status_label.setAlignment(Qt.AlignCenter)
        self.bw_status_label.setStyleSheet("font: bold 12px")
        third_tab_layout.addWidget(self.bw_status_label)

        # Create a button to get manipulator above blocks
        self.moveit = MoveGroupActionClient(self._context.node)
        blocks_position_button = QPushButton("Position the manipulator")
        blocks_position_button.clicked.connect(self.blocks_manipulator_position)
        blocks_position_button.setStyleSheet(
            "background-color: green; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
        )
        third_tab_layout.addWidget(blocks_position_button)

        # Create a toggle button to solve blocksworld
        self.toggle_blocks_execute = QPushButton("Execute agent")
        self.toggle_blocks_execute.setCheckable(True)
        self.toggle_blocks_execute.setChecked(False)  # Set the initial state
        self.toggle_blocks_execute.toggled.connect(self.blocks_execute)
        self.toggle_blocks_execute.setStyleSheet(
            "background-color: transparent; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
        )
        third_tab_layout.addWidget(self.toggle_blocks_execute)

        ## FINAL BOX
        # Create tab widgets
        first_tab_widget = QWidget()
        second_tab_widget = QWidget()
        third_tab_widget = QWidget()

        # Set layouts for tab widgets
        first_tab_widget.setLayout(first_tab_layout)
        second_tab_widget.setLayout(second_tab_layout)
        third_tab_widget.setLayout(third_tab_layout)

        # Add tab widgets to the tab widget
        tab_widget.addTab(first_tab_widget, "BASIC")
        tab_widget.addTab(second_tab_widget, "PLUS")
        tab_widget.addTab(third_tab_widget, "BW")

        # Apply style to the tab widget
        tab_widget.setStyleSheet(
            """
            QTabWidget::pane {
                border: none;
            }

            QTabWidget::tab-bar {
                alignment: left;
            }
            """
        )

        layout.addWidget(tab_widget)
        layout.addWidget(self.status_label)

        # Add the widget to the plugin
        self._widget = self.widget
        context.add_widget(self._widget)

        self.front_value = 0
        self.left_value = 0
        self.right_value = 0

    def toggle_frame_runner(self, checked):
        if checked:  # Button is checked, start the command
            ssh_command = f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -X screen -t runner ros2 launch gpp_action_examples frame_runner_launch.py"'
            subprocess.Popen(ssh_command, shell=True)
            self.toggle_button.setStyleSheet(
                "background-color: green; font: bold 15px; border-width: 5px; border-radius: 15px; padding: 15px"
            )
        else:  # Button is unchecked, stop the command
            self.toggle_button.setStyleSheet(
                "background-color: transparent; font: bold 15px; border-width: 5px; border-radius: 15px; padding: 15px"
            )
            # Implement code to kill the tab in the screen session
            kill_tab_command = (
                f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -X -p runner kill"'
            )
            subprocess.Popen(kill_tab_command, shell=True)

    def toggle_two_way_audio(self, button):
        if button.isChecked():
            # Operator side
            ssh_command = "screen -S deck -X screen -t capture ros2 run audio_capture audio_capture_node --ros-args -p format:=wave -r __ns:=/operator"  # Sent audio
            subprocess.Popen(ssh_command, shell=True)
            ssh_command = "screen -S deck -X screen -t play ros2 run audio_play audio_play_node --ros-args -p format:=wave -r __ns:=/nuc"  # Received audio
            subprocess.Popen(ssh_command, shell=True)

            # Nuc side
            ssh_command = f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -X screen -t capture ros2 run audio_capture audio_capture_node --ros-args -p format:=wave -r __ns:=/nuc"'
            subprocess.Popen(ssh_command, shell=True)
            ssh_command = f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -X screen -t play ros2 run audio_play audio_play_node --ros-args -p format:=wave -r __ns:=/operator"'
            subprocess.Popen(ssh_command, shell=True)
        else:
            # Operator
            ssh_command = "screen -S deck -X -p capture kill"
            subprocess.Popen(ssh_command, shell=True)
            ssh_command = "screen -S deck -X -p play kill"
            subprocess.Popen(ssh_command, shell=True)

            # NUC
            ssh_command = (
                f'ssh {MAX_USER}@{MAX_IP} "screen -S spot_session -X -p capture kill"'
            )
            subprocess.Popen(ssh_command, shell=True)
            ssh_command = (
                f'ssh {MAX_USER}@{MAX_IP} "screen -S spot_session -X -p play kill"'
            )
            subprocess.Popen(ssh_command, shell=True)

    def front_light_value_change_callback(self, value):
        self.front_value = int((value + 1) * 255 / 100)
        if self.front_value < 4:
            self.front_value = 0

    def left_light_value_change_callback(self, value):
        self.left_value = int((value + 1) * 255 / 100)
        if self.left_value < 4:
            self.left_value = 0

    def right_light_value_change_callback(self, value):
        self.right_value = int((value + 1) * 255 / 100)
        if self.right_value < 4:
            self.right_value = 0

    def front_light_callback(self):
        msg = String()
        msg.data = f"camera_light:{self.front_value}"
        self.lights_pub.publish(msg)

    def left_light_callback(self):
        msg = String()
        msg.data = f"left_light:{self.left_value}"
        self.lights_pub.publish(msg)

    def right_light_callback(self):
        msg = String()
        msg.data = f"right_light:{self.right_value}"
        self.lights_pub.publish(msg)

    def send_stop_command(self):
        ssh_command = (
            f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -X screen -t power_off ros2 service call power_off std_srvs/srv/Trigger"'
        )
        subprocess.Popen(ssh_command, shell=True)

    def stop_slider_released(self):
        if self.stop_slider.value() > 50:
            self.send_stop_command()
            self.stop_slider.setSliderPosition(0)  # Reset the slider to the left

    def send_power_on_command(self):
        ssh_command = (
            f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -X screen -t power_on ros2 service call power_off std_srvs/srv/Trigger"'
        )
        subprocess.Popen(ssh_command, shell=True)

    def status_callback(self, msg: BatteryStateArray):
        time_left = msg.battery_states[0].estimated_runtime.sec // 60
        battery_percent = msg.battery_states[0].charge_percentage

        self.status_label.setText(
            f"Remaining:  {int(battery_percent)}% | {time_left} min"
        )

    def run_ssh_command(self):
        ssh_commands = [
            f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p spot -X stuff \\"ros2 launch spot_driver_plus spot_launch.py\\\\n\\""',
            f'sleep 1 && ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p kinova -X stuff \\"ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.50.9 dof:=6 launch_rviz:=false\\\\n\\""',
            f'sleep 2 && ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p kinova_moveit -X stuff \\"ros2 launch spot_gen3_moveit move_group.launch.py\\\\n\\""',
            f'sleep 3 && ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p kinova_vision -X stuff \\"ros2 launch kinova_vision rgbd_launch.py\\\\n\\""',
            # f'sleep 4 && ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p lidar -X stuff \\"ros2 launch rrl_launchers velodyne-all-nodes-VLP16-composed-launch.py\\\\n\\""',
            f'sleep 5 && ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p realsenses -X stuff \\"ros2 launch rrl_launchers realsenses_launch.py\\\\n\\""',
            f'sleep 6 && ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p octomap -X stuff \\"ros2 launch octomap_server octomap_velodyne_launch.py\\\\n\\""',
            f'sleep 7 && ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p thermal -X stuff \\"ros2 launch seek_thermal_ros thermal_publisher_launch.py\\\\n\\""',
        ]
        for ssh_command in ssh_commands:
            subprocess.Popen(ssh_command, shell=True)

    def hazmat_command(self):
        ssh_command = f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -p Hazmat -X stuff \\"ros2 run world_info object_detection_yolov5 hazmat /kinova_color\\\\n\\""'
        subprocess.Popen(ssh_command, shell=True)

    def world_reset_command(self):
        if not self.map_frame_reset_client.wait_for_service(timeout_sec=1.0):
            self._context.node.get_logger().info(
                "service map_reset not available, skipping command"
            )
        else:
            self.map_frame_reset_client.call_async(Trigger.Request())

        if not self.path_reset_client.wait_for_service(timeout_sec=1.0):
            self._context.node.get_logger().info(
                "service path_reset not available, skipping command"
            )
        else:
            self.path_reset_client.call_async(Empty.Request())

        if not self.world_info_reset_client.wait_for_service(timeout_sec=1.0):
            self._context.node.get_logger().info(
                "service world_info_reset not available, skipping command"
            )
        else:
            self.world_info_reset_client.call_async(Empty.Request())

        if not self.octomap_reset_client.wait_for_service(timeout_sec=1.0):
            self._context.node.get_logger().info(
                "service octomap_reset not available, skipping command"
            )
        else:
            self.octomap_reset_client.call_async(Empty.Request())

    # BLOCKSWORLD
    def block_loc_update(self, msg):
        if self.blocks_scan_checked:
            self.bw_status_label.setText(f"{msg.data}")
        else:
            self.bw_status_label.setText("Unknown")

    def blocks_scan(self, checked):
        if checked:
            self.blocks_scan_checked = True
            ssh_command = f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -X screen -t blocksworld_scan ros2 run world_info aruco_node"'
            self.toggle_blocks_scan.setStyleSheet(
                "background-color: green; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
            )
        else:  # Button is unchecked, stop the command
            self.blocks_scan_checked = False
            self.bw_status_label.setText("Unknown")
            self.toggle_blocks_scan.setStyleSheet(
                "background-color: transparent; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
            )
            ssh_command = f'ssh {MAX_USER}@{MAX_IP} "screen -S spot_session -X -p blocksworld_scan kill"'
        subprocess.Popen(ssh_command, shell=True)

    def blocks_manipulator_position(self):
        self.moveit.send_goal([0, 36, -84, 0, -60, 90])

    def blocks_execute(self, checked):
        if checked:
            ssh_command = f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -X screen -t blocksworld_gpp_wrapper ros2 run webots_spot gpp_blocksworld_server"'
            subprocess.Popen(ssh_command, shell=True)
            ssh_command = f'ssh -t {MAX_USER}@{MAX_IP} "screen -S spot_session -X screen -t blocksworld_gpp_agent ros2 launch webots_spot blocksworld_launch.py"'
            subprocess.Popen(ssh_command, shell=True)
            self.toggle_blocks_execute.setStyleSheet(
                "background-color: green; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
            )
        else:  # Button is unchecked, stop the command
            self.bw_status_label.setText("Unknown")
            self.toggle_blocks_execute.setStyleSheet(
                "background-color: transparent; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
            )
            ssh_command = f'ssh {MAX_USER}@{MAX_IP} "screen -S spot_session -X -p blocksworld_gpp_wrapper kill"'
            subprocess.Popen(ssh_command, shell=True)
            ssh_command = f'ssh {MAX_USER}@{MAX_IP} "screen -S spot_session -X -p blocksworld_gpp_agent kill"'
            subprocess.Popen(ssh_command, shell=True)

    def shutdown_plugin(self):
        self._context.node.destroy_node()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


def main():
    """Run the plugin."""
    # Initialize the ROS node
    rclpy.init()

    # Create the plugin and run it
    sys.exit(Main().main(sys.argv, standalone="my_rqt_plugin.my_rqt_plugin"))


if __name__ == "__main__":
    main()
