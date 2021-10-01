#!/usr/bin/python

import time
import numpy as np
import rospy
import RPi.GPIO as GPIO
import Adafruit_PCA9685

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class BaseControl():
    def __init__(self, name):
        self.name = name
        self.rospy = rospy
        self.rospy.init_node('Control', anonymous = True)
        self.rospy.loginfo("[%s] Starting AGoRA Walker Base", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.mainControl()
        
    def initParameters(self):
        self.cmd_vel_topic = self.rospy.get_param("~cmd_vel_topic","cmd_vel")
        self.odom_topic = self.rospy.get_param("~odom_topic","odom")
        self.control_rate = self.rospy.get_param("~control_parameters/rate", 30)
        self.vel_limits = { "linear": self.rospy.get_param("~vel_limits/linear", 1.5),
                            "angular": self.rospy.get_param("~vel_limits/angular",1.0)}
        return

    def initSubscribers(self):
        self.sub_cmd_vel = self.rospy.Subscriber(self.cmd_vel_topic, Twist, self.callbackCmdVel)
        return

    def initPublishers(self):
        self.pub_odom = self.rospy.Publisher(self.odom_topic, Twist, queue_size = 10)
        return
    
    def initVariables(self):
        #Initialize raspberry pi GPIOs
        self.pinResetRight = 7 #gpio4
        self.pinEnableRight = 22 #gpio25

        self.pinResetLeft = 18 #gpio24
        self.pinEnableLeft = 16 #gpio23
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pinResetRight, GPIO.OUT)
        GPIO.setup(self.pinEnableRight, GPIO.OUT)
        GPIO.setup(self.pinResetLeft, GPIO.OUT)
        GPIO.setup(self.pinEnableLeft, GPIO.OUT)
        
        # Initialise the PCA9685 using the default address (0x40).
        self.left_motor = 1
        self.right_motor = 0
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(1000)
        self.pwm.set_pwm(0,0, 2048)
        self.pwm.set_pwm(1,0, 2048)
        
        self.rate = self.rospy.Rate(self.control_rate)
        self.change_vel = False
        return
    
    def enableMotion(self):
        rospy.loginfo("[%s] Enabling motion engine ...", self.name)
        GPIO.output(self.pinEnableRight, GPIO.LOW)
        GPIO.output(self.pinEnableRight, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.pinEnableRight, GPIO.LOW)

        GPIO.output(self.pinEnableLeft, GPIO.LOW)
        GPIO.output(self.pinEnableLeft, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.pinEnableLeft, GPIO.LOW)
        rospy.loginfo("[%s] Motion engine OK", self.name)
        return
    
    def clearEvents(self):
        rospy.loginfo("[%s] Clearing driver events ...", self.name)
        GPIO.output(self.pinResetRight, GPIO.LOW)
        GPIO.output(self.pinResetRight, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.pinResetRight, GPIO.LOW)

        GPIO.output(self.pinResetLeft, GPIO.LOW)
        GPIO.output(self.pinResetLeft, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.pinResetLeft, GPIO.LOW)
        rospy.loginfo("[%s] Driver events OK", self.name)
        
    def callbackCmdVel(self, msg):
        lin = self.vel_limits["linear"]*np.tanh(msg.linear.x)
        ang = self.vel_limits["angular"]*np.tanh(msg.angular.z)
        
        vl =  lin - (0.525/2.0)*ang
        vr =  lin + (0.525/2.0)*ang
        
        rpm_l = (vl*60/0.525)
        rpm_r = (vr*60/0.525)
        
        pwm_l = int(11.372*rpm_l + 2048)
        pwm_r = int(-11.372*rpm_r + 2048)
        
        if pwm_l <= 20:
            self.pwm.set_pwm(self.left_motor, 0, 20)
            return
        elif pwm_l > 2028 and pwm_l < 2068:
            self.pwm.set_pwm(self.left_motor, 0, 2048)
            return
        elif pwm_l >= 4080:
            self.pwm.set_pwm(self.left_motor, 0, 4080)
            return
        
        if pwm_r <= 20:
            self.pwm.set_pwm(self.right_motor, 0, 20)
            return
        elif pwm_r > 2028 and pwm_l < 2068:
            self.pwm.set_pwm(self.right_motor, 0, 2048)
            return
        elif pwm_r >= 4080:
            self.pwm.set_pwm(self.right_motor, 0, 4080)
            return
            
        self.pwm.set_pwm(self.right_motor, 0, int(pwm_r))
        self.pwm.set_pwm(self.left_motor, 0, int(pwm_l))
        return

        
    def mainControl(self):
        self.enableMotion()
        self.clearEvents()
        self.rospy.loginfo("[%s] Base control OK", self.name)
        try:
            while not self.rospy.is_shutdown():
                if self.change_vel:
                    self.commandVel()
                    self.updateOdom()
                    self.change_vel = False
                self.rate.sleep()
        except:
            self.pwm.set_pwm(self.right_motor, 0, 2048)
            self.pwm.set_pwm(self.left_motor, 0, 2048)
            
if __name__ == '__main__':
    try:
        sw = BaseControl('base')
    except rospy.ROSInterruptException:
        pass
