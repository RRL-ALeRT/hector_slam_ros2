#!/usr/bin/env python3

"""Provides a programmatic estop to stop the robot."""
from __future__ import print_function

import os
import argparse
import curses
import threading
import signal
import sys
import time

import bosdyn.client.util
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.exceptions import UnableToConnectToRobotError, ProxyConnectionError

import rclpy
from std_msgs.msg import String


class EstopNoGui():
    """Provides a software estop without a GUI.

    To use this estop, create an instance of the EstopNoGui class and use the stop() and allow()
    functions programmatically.
    """

    def __init__(self, client, timeout_sec, name=None):

        # Force server to set up a single endpoint system
        ep = EstopEndpoint(client, name, timeout_sec)
        ep.force_simple_setup()

        # Begin periodic check-in between keep-alive and robot
        self.estop_keep_alive = EstopKeepAlive(ep)

        # Release the estop
        self.estop_keep_alive.allow()

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Cleanly shut down estop on exit."""
        self.estop_keep_alive.end_periodic_check_in()

    def stop(self):
        self.estop_keep_alive.stop()

    def allow(self):
        self.estop_keep_alive.allow()

    def settle_then_cut(self):
        self.estop_keep_alive.settle_then_cut()


def main(argv):
    """If this file is the main, create an instance of EstopNoGui and wait for user to terminate.

    This has little practical use, because calling the function this way does not give the user
    any way to trigger an estop from the terminal.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--timeout', type=float, default=10, help='Timeout in seconds')
    options = parser.parse_args(argv)

    # Create robot object
    SPOT_IP = os.getenv('SPOT_IP')
    if SPOT_IP is None:
        print("SPOT_IP environment variable is not set.")
        return
    BOSDYN_CLIENT_USERNAME = os.getenv('BOSDYN_CLIENT_USERNAME')
    if BOSDYN_CLIENT_USERNAME is None:
        print("BOSDYN_CLIENT_USERNAME environment variable is not set.")
        return
    BOSDYN_CLIENT_PASSWORD = os.getenv('BOSDYN_CLIENT_PASSWORD')
    if BOSDYN_CLIENT_PASSWORD is None:
        print("BOSDYN_CLIENT_PASSWORD environment variable is not set.")
        return

    sdk = bosdyn.client.create_standard_sdk('estop_nogui')
    robot = sdk.create_robot(SPOT_IP)

    global status_pub
    status_pub = node.create_publisher(String, 'estop_status', 1)

    connected_ = False
    count = 0
    while rclpy.ok() and not connected_:
        try:
            msg = String()
            msg.data = "Connecting"
            status_pub.publish(msg)
            print("Trying to connect")

            bosdyn.client.util.authenticate(robot)
            # Create estop client for the robot
            estop_client = robot.ensure_client(EstopClient.default_service_name)

            connected_ = True

            # Create nogui estop
            global estop_nogui
            estop_nogui = EstopNoGui(estop_client, options.timeout, "Estop NoGUI")

            # Create robot state client for the robot
            state_client = robot.ensure_client(RobotStateClient.default_service_name)

        except:
            count += 1
            print(f"Retry#{count} in 10s")
            msg = String()
            msg.data = f"Retry#{count} in 10s"
            status_pub.publish(msg)
            time.sleep(10)


    # Initialize curses screen display
    stdscr = curses.initscr()

    def estop_command_callback(msg):
        """Callback function for estop_command messages."""
        try:
            if msg.data == 'stop':
                estop_nogui.stop()
                # estop_nogui.settle_then_cut()
            elif msg.data == 'release':
                estop_nogui.allow()
        # If the user attempts to toggle estop without valid endpoint
        except bosdyn.client.estop.EndpointUnknownError:
            clean_exit("This estop endpoint no longer valid. Exiting...")

    # node.create_subscription(String, 'estop_command', estop_command_callback, 1)

    def cleanup_example(msg):
        """Shut down curses and exit the program."""
        print('Exiting')
        #pylint: disable=unused-argument
        estop_nogui.estop_keep_alive.shutdown()

        # Clean up and close curses
        stdscr.keypad(False)
        curses.echo()
        stdscr.nodelay(False)
        curses.endwin()
        print(msg)

    def clean_exit(msg=''):
        msg = String()
        msg.data = "EXITED"
        status_pub.publish(msg)
        rclpy.shutdown()
        cleanup_example(msg)
        estop_nogui.stop()
        exit(0)

    def sigint_handler(sig, frame):
        """Exit the application on interrupt."""
        clean_exit()

    def run_example():
        """Run the actual example with the curses screen display"""
        # Set up curses screen display to monitor for stop request
        curses.noecho()
        stdscr.keypad(True)
        stdscr.nodelay(True)
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
        # If terminal cannot handle colors, do not proceed
        if not curses.has_colors():
            return

        # Curses eats Ctrl-C keyboard input, but keep a SIGINT handler around for
        # explicit kill signals outside of the program.
        signal.signal(signal.SIGINT, sigint_handler)

        # Clear screen
        stdscr.clear()

        # Display usage instructions in terminal
        stdscr.addstr('Estop w/o GUI running.\n')
        stdscr.addstr('\n')
        stdscr.addstr('[q] or [Ctrl-C]: Quit\n', curses.color_pair(2))
        stdscr.addstr('[SPACE]: Trigger estop\n', curses.color_pair(2))
        stdscr.addstr('[r]: Release estop\n', curses.color_pair(2))
        stdscr.addstr('[s]: Settle then cut estop\n', curses.color_pair(2))

        first_time = True

        # Monitor estop until user exits
        while rclpy.ok():
            # Retrieve user input (non-blocking)
            c = stdscr.getch()

            try:
                if first_time:
                    first_time = False
                    estop_nogui.allow()

                if c == ord(' '):
                    estop_nogui.stop()
                if c == ord('r'):
                    estop_nogui.allow()
                if c == ord('q') or c == 3:
                    clean_exit('Exit on user input')
                if c == ord('s'):
                    estop_nogui.settle_then_cut()
            # If the user attempts to toggle estop without valid endpoint
            except bosdyn.client.estop.EndpointUnknownError:
                clean_exit("This estop endpoint no longer valid. Exiting...")

            # Check if robot is estopped by any estops
            estop_status = 'NOT_STOPPED\n'
            msg = String()
            msg.data = "RUNNING"
            estop_status_color = curses.color_pair(1)
            state = state_client.get_robot_state()
            estop_states = state.estop_states

            # print(state.robot_state.battery_states[0].charge_percentage.value)
            for estop_state in estop_states:
                state_str = estop_state.State.Name(estop_state.state)
                if state_str == 'STATE_ESTOPPED':
                    msg = String()
                    msg.data = "STOPPED"
                    status_pub.publish(msg)
                    estop_status = 'STOPPED\n'
                    estop_status_color = curses.color_pair(3)
                    break
                elif state_str == 'STATE_UNKNOWN':
                    msg = String()
                    msg.data = "ERROR"
                    status_pub.publish(msg)
                    estop_status = 'ERROR\n'
                    estop_status_color = curses.color_pair(3)
                elif state_str == 'STATE_NOT_ESTOPPED':
                    pass
                else:
                    # Unknown estop status
                    clean_exit()

            msg.data += f" {int(state.battery_states[0].charge_percentage.value)}%"
            status_pub.publish(msg)

            # Display current estop status
            if not estop_nogui.estop_keep_alive.status_queue.empty():
                latest_status = estop_nogui.estop_keep_alive.status_queue.get()[1].strip()
                if latest_status != '':
                    # If you lose this estop endpoint, report it to user
                    stdscr.addstr(7, 0, latest_status, curses.color_pair(3))
            stdscr.addstr(6, 0, estop_status, estop_status_color)

            # Slow down loop
            time.sleep(0.5)


    # Run all curses code in a try so we can cleanly exit if something goes wrong
    try:
        run_example()
    except Exception as e:
        cleanup_example(e)
        raise e


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    global node
    node = rclpy.create_node('estop_command_subscriber')
    node_thread = threading.Thread(target=rclpy.spin, args=[node,])
    node_thread.start()
    main(sys.argv[1:])
    node_thread.join()
    node.destroy_node()
    rclpy.shutdown()
