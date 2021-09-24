from USB6009_v2 import USB6009
import rospy

class DAQ_v2(object):
    def __init__(self):
        self.NIDevice = USB6009()
        rospy.loginfo("Connecting to USB6009")
        self.initParameters()
        
    def initParameters(self):
        self.ml = (1/0.0091)
        self.bl = (0.9078/0.0091)
        self.mr = (1/0.0088)
        self.br = (-0.9252/0.0088)
        self.channels = (3,2,7,6)

    def get_forces(self):
        channels_info = self.NIDevice.readchannel(self.channels)
        ry_force = -1*(channels_info[0][1]*self.mr + self.br)
        rz_force = channels_info[0][0]*self.mr + self.br
        lz_force = -1*(channels_info[0][2]*self.ml + self.bl)
        ly_force = -1*(channels_info[0][3]*self.ml + self.bl)
        return ("{0:.3f}\t{1:.3f}\t{2:.3f}\t{3:.3f}".format(ly_force, lz_force, ry_force, rz_force))

    def close_port(self):
        try:
            self.NIDevice.cerrar()
            rospy.loginfo("Closing of USB6009 succeed")
        except:
            rospy.loginfo("Closing of USB6009 failed")
        return
