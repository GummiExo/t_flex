#!/usr/bin/env python
import rospy
from gummi_ankle.msg import GaitPhase
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from rospkg import RosPack

class GaitPhasesDisplayer(object):
    def __init__(self):
        self.gait_phase = -1
        """Image Loading"""
        rp = RosPack()
        path = rp.get_path('gummi_ankle')
        self.img_hs = plt.imread(path+'/image/heel_strike.png',0)
        self.img_ff = plt.imread(path+'/image/flat_foot.png',0)
        self.img_mst = plt.imread(path+'/image/midstance.png',0)
        self.img_ho = plt.imread(path+'/image/heel_off.png',0)
        self.img_to = plt.imread(path+'/image/toe_off.png',0)
        self.img_msw = plt.imread(path+'/image/midswing.png',0)
        print("Images loaded")
        """Display init image"""
        self.fig = plt.figure()
        self.current_img = self.img_hs
        # self.im = plt.imshow(self.current_img, animated=True)
        self.im = plt.imshow(self.current_img)
        """ROS commands"""
        rospy.init_node('gait_phases_displayer', anonymous = True)
        print 'Gait phases displayer node started'
        rospy.Subscriber("/gait_phase_detection", GaitPhase, self.update_gait_phase)

    def update_gait_phase(self, detection):
        if self.gait_phase != detection.phase:
            plt.clf()
            """HEEL STRIKE"""
            if self.gait_phase == 1:
                self.current_img = self.img_hs
            """FLAT FOOT"""
            if self.gait_phase == 2:
                self.current_img = self.img_ff
            """MIDSTANCE"""
            if self.gait_phase == 3:
                self.current_img = self.img_mst
            """HEEL-OFF"""
            if self.gait_phase == 4:
                self.current_img = self.img_ho
            """TOE-OFF"""
            if self.gait_phase == 5:
                self.current_img = self.img_to
            """MIDSWING"""
            if self.gait_phase == 6:
                self.current_img = self.img_msw

    def animate(self,i):
        i = 0
        self.im = plt.imshow(self.current_img)

def main():
    displayer = GaitPhasesDisplayer()
    ani = animation.FuncAnimation(displayer.fig, displayer.animate, interval=50)
    plt.show()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
        main()
