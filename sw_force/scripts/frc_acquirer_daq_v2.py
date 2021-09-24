#!/usr/bin/python
import rospy
import time
import numpy as np
from scipy.signal import butter, filtfilt
from threading import Thread
from geometry_msgs.msg import Wrench
from sys import stdin
from include.daq_controller.DAQ_v2 import DAQ_v2

class FrcAcquirer():
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name, anonymous = True)
        rospy.loginfo("[%s] Starting Force Acquisition", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initServiceClients()
        self.initVariables()
        
    def initParameters(self):
        self.frc_left_topic = rospy.get_param("~frc_left_topic", "/frc_left")
        self.frc_right_topic = rospy.get_param("~frc_right_topic", "/frc_right")
        self.frc_topic = rospy.get_param("~frc_topic", "/frc")
        self.fs = rospy.get_param("~freq", 40)    #Max 30. Hz
        self.tcalib = rospy.get_param("~t_calibration", 1)
        self.calibrate = rospy.get_param("~calibrate", True)
        self.filter = rospy.get_param("~filter", True)
        self.window = rospy.get_param("~filter_window", 10)
        return
    
    def initSubscribers(self):
        pass
    
    def initPublishers(self):
        self.pub_left = rospy.Publisher(self.frc_left_topic, Wrench, queue_size = 10)
        self.pub_right = rospy.Publisher(self.frc_right_topic, Wrench, queue_size = 10)
        self.pub_frc = rospy.Publisher(self.frc_topic, Wrench, queue_size = 10)
        return
    
    def initServiceClients(self):
        pass
    
    def initVariables(self):
        self.frc_left = Wrench()
        self.frc_right = Wrench()
        self.frc = Wrench()
        self.exitFlag = False
        self.daq = DAQ_v2()        
        self.fly = []
        self.flz = []
        self.fry = []
        self.frz = []
        self.ffly = []
        self.fflz = []
        self.ffry = []
        self.ffrz = []
        self.rate = rospy.Rate(self.fs)
        return

    def calibration(self, t):
        initial_data = []
        tot_time = 0
        while tot_time < t:
            h3 = time.time()
            data = [float(i) for i in self.daq.get_forces().split('\t')]
            initial_data.append(data)
            h4 = time.time()
            tot_time += (h4-h3)
        initial_data = np.array(initial_data)
        return np.mean(initial_data,axis=0)

    def wait_for_exit(self):
        rospy.logwarn("[%s] Press enter to exit", self.name)
        x = stdin.readline().strip()
        self.exitFlag = True
        
    def process(self, data):
        if self.filter:
            self.fly.append(data[0])
            self.flz.append(data[1])
            self.fry.append(data[2])
            self.frz.append(data[3])
            if len(self.fly) > self.window:
                self.fly.pop(0)
                self.flz.pop(0)
                self.fry.pop(0)
                self.frz.pop(0)
                self.ffly = np.mean(self.fly)
                self.fflz = np.mean(self.flz)
                self.ffry = np.mean(self.fry)
                self.ffrz = np.mean(self.frz)
            else:
                self.ffly = 0
                self.fflz = 0
                self.ffry = 0
                self.ffrz = 0
        else:
            self.ffly = data[0]
            self.fflz = data[1]
            self.ffry = data[2]
            self.ffrz = data[3]
        return
    
    def makeMsg(self):
        self.frc_left.force.x, self.frc_left.force.y, self.frc_left.force.z = 0, self.ffly, self.fflz
        self.frc_right.force.x, self.frc_right.force.y, self.frc_right.force.z = 0, self.ffry, self.ffrz
        self.frc.torque.x = 0
        self.frc.torque.y = (self.frc_right.force.y - self.frc_left.force.y)/2.0
        self.frc.torque.z = (self.frc_right.force.z - self.frc_left.force.z)/2.0
        self.frc.force.x = 0
        self.frc.force.y = (self.frc_left.force.y + self.frc_right.force.y)/2.0
        self.frc.force.z = (self.frc_left.force.z + self.frc_right.force.z)/2.0

    def acquire(self):
        rospy.loginfo("[%s] Frc Acquisition ok", self.name)
        if self.calibrate:
            means = self.calibration(self.tcalib)
        rospy.loginfo("[%s] Calibration Done", self.name)
        while not self.exitFlag and not rospy.is_shutdown():
            daq = self.daq.get_forces().split('\t')
            data = [float(i) - means[c] for c, i in enumerate(daq)]
            self.process(data)
            self.makeMsg()
            self.pub_left.publish(self.frc_left)
            self.pub_right.publish(self.frc_right)
            self.pub_frc.publish(self.frc)
            self.rate.sleep()
        self.daq.close_port()
        rospy.loginfo("[%s] Port succesfully closed", self.name)
        
    def __exit__(self):
        print(test)
        pass

if __name__ == '__main__':
    try:
        frc = FrcAcquirer("frc_node")
        thread1 = Thread(target = frc.acquire)
        thread1.start()
        frc.wait_for_exit()
    except rospy.ROSInterruptException:
        print("Something's gone wrong. Exiting")
