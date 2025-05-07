#!/usr/bin/env python3
import rospy
from quadrotor_msgs.msg import BfctrlStatue
import subprocess
import yaml
import os
import signal
import time
import std_msgs.msg as std_msgs


class BagRecord:
    def __init__(self) -> None:
        config_path = ""
        config_path = rospy.get_param("~config_file")
        is_record_begin = rospy.get_param("~is_record_begin", False)
        with open(config_path) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
        self.is_recording = False
        self.statue = BfctrlStatue.BFCTRL_STATUS_INIT
        self.topics = config["topics_record_list"]
        self.bag_path_base = rospy.get_param("~bag_path_base")
        self.params = {}
        self.params["collision"] = False
        self.param_updata_tick = 0
        if not self.bag_path_base.endswith("/"):
            self.bag_path_base += "/"
        if not os.path.exists(self.bag_path_base):
            os.mkdir(self.bag_path_base)
        self.bag_prefix = rospy.get_param("~bag_prefix")
        self.bag_prefix += "-" + time.strftime("%Y-%m-%d-%H-%M-%S")
        self.options = ["--lz4", f"-O {self.bag_path_base}{self.bag_prefix}.bag"]
        self.record_subprocess = None
        self.is_collision = False
        if is_record_begin:
            self.StartRecord()
        self.statueSub = rospy.Subscriber(
            "/bfctrl/statue", BfctrlStatue, self.StatueCallback
        )
        self.collisionSub = rospy.Subscriber(
            "/airsim_node/drone_1/collision", std_msgs.Bool, self.CollisionCallback
        )

    def StatueCallback(self, msg: BfctrlStatue):
        self.statue = msg.status

    def CollisionCallback(self, msg: std_msgs.Bool):
        if msg.data == True and self.statue == BfctrlStatue.BFCTRL_STATUS_CMD:
            self.is_collision = True
            self.params["collision"] = True

    def SaveParams(self):
        paramFile = open(self.bag_path_base + self.bag_prefix + "_param.yaml", "w")
        yaml.dump(self.params, paramFile)

    def StopRecord(self):
        self.is_recording = False
        rospy.loginfo("[Bag Recorder] Stop recording")
        os.killpg(os.getpgid(self.record_subprocess.pid), signal.SIGINT)
        self.record_subprocess.wait()
        self.SaveParams()

    def StartRecord(self):
        self.is_recording = True
        topic_str = ""
        for topic in self.topics:
            topic_str += topic + " "
        option_str = ""
        for option in self.options:
            option_str += option + " "
        # Define the command to be executed
        command = "rosbag record " + option_str + topic_str
        rospy.loginfo("[Bag Recorder] Start recording")
        rospy.loginfo(command)
        # Execute the command in the terminal
        self.record_subprocess = subprocess.Popen(
            command, shell=True, preexec_fn=os.setsid
        )

    def Monitior(self):
        self.param_updata_tick += 1
        if not self.is_recording and not self.statue == BfctrlStatue.BFCTRL_STATUS_INIT:
            self.StartRecord()
        if self.param_updata_tick > 100:
            self.param_updata_tick = 0
            params = rospy.get_param_names()
            for param in params:
                if param not in self.params.keys():
                    self.params[param] = rospy.get_param(param)


def signal_handler(sig, frame):
    print("Interrupted! Cleaning up...")
    try:
        if bag_record.is_recording:
            bag_record.StopRecord()
    except NameError:
        print("Variable 'bag_record' does not exist.")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("bag_record")
    bag_record = BagRecord()
    while not rospy.is_shutdown():
        bag_record.Monitior()
        rospy.sleep(0.033)
    if bag_record.is_recording:
        bag_record.StopRecord()
