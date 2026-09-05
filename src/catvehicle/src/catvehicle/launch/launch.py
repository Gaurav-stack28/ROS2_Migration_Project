#!/usr/bin/env python3

# Initial Date: June 2020
# ROS 2 migration
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import os
import threading
import time

from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


class launch:
    """
    ROS 2 replacement for the original ROS 1 roslaunch helper.

    This class starts and stops a ROS 2 Python launch file and passes
    runtime arguments to that launch file.

    Example:

        launchobj = launch(
            launchfile="/path/to/example.launch.py",
            robot="catvehicle_000",
            X=1.0,
            Y=2.0
        )

        launchobj.start()
        launchobj.shutdown()
    """

    def __init__(self, launchfile, **kwargs):

        self.launchfile = os.path.abspath(launchfile)

        if not os.path.isfile(self.launchfile):
            raise FileNotFoundError(
                "ROS 2 launch file not found: {}".format(self.launchfile)
            )

        if not self.launchfile.endswith(".launch.py"):
            raise ValueError(
                "ROS 2 launch files must use the .launch.py extension: {}".format(
                    self.launchfile
                )
            )

        self.runtime_args = []

        for key, value in kwargs.items():
            self.runtime_args.append(
                (str(key), str(value).lower() if isinstance(value, bool) else str(value))
            )

        self.launch_service = None
        self.launch_thread = None
        self.running = False

    def start(self):
        """
        Start the ROS 2 launch file.
        """

        if self.running:
            print("{} is already running.".format(self.launchfile))
            return

        print("Starting ROS 2 launch file: {}".format(self.launchfile))

        launch_arguments = [
            (name, value)
            for name, value in self.runtime_args
        ]

        include_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(self.launchfile),
            launch_arguments=launch_arguments
        )

        self.launch_service = LaunchService(
            argv=[]
        )

        self.launch_service.include_launch_description(
            include_launch
        )

        self.launch_thread = threading.Thread(
            target=self.launch_service.run,
            daemon=True
        )

        self.launch_thread.start()

        self.running = True

        time.sleep(2)

        if self.runtime_args:
            print(
                "{} started with runtime arguments {}".format(
                    self.launchfile,
                    self.runtime_args
                )
            )
        else:
            print("{} started.".format(self.launchfile))

    def shutdown(self):
        """
        Stop the ROS 2 launch file.
        """

        if not self.running:
            return

        print("Stopping ROS 2 launch file: {}".format(self.launchfile))

        if self.launch_service is not None:
            try:
                self.launch_service.shutdown()
            except Exception as exc:
                print(
                    "Warning while shutting down {}: {}".format(
                        self.launchfile,
                        exc
                    )
                )

        if self.launch_thread is not None:
            self.launch_thread.join(timeout=5.0)

        self.running = False

        print("{} terminated.".format(self.launchfile))