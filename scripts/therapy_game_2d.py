import rospy, rospkg
import time
from dynamixel_controllers.srv import SetSpeed, TorqueEnable, SetKGain
from std_msgs.msg import Bool, Float64
import os
import sys

class TherapyController(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        package_directory = rospack.get_path('t_flex')
        os.chdir(package_directory + '/yaml')
        f = open("calibrationAngle.yaml", "r+")
        params = [f.readline().strip().split()[1] for i in range(4)]
        rospy.init_node('t_flex_therapy', anonymous = True)
        self.ValueToPubUp1 = float(params[0])
        self.ValueToPubDown1 = float(params[1])
        self.ValueToPubUp2 = float(params[2])
        self.ValueToPubDown2 = float(params[3])
        set_motor_speed(10)
        rospy.Subscriber("/kill_therapy", Bool, self.updateFlagTherapy)
        rospy.Subscriber("/movement_intention", Bool, self.updateMovementIntention)
        self.frontal_motor_pub = rospy.Publisher("/tilt1_controller/command", Float64, queue_size = 1, latch = False)
        self.posterior_motor_pub = rospy.Publisher("/tilt2_controller/command", Float64, queue_size = 1, latch = False)
        self.kill_therapy = False
        self.past_state = False
        self.movement_detected = False
        self.activate_motors = False
        self.frequency = 1.5

    def updateFlagTherapy(self, flag):
        self.kill_therapy = flag.data

    def updateMovementIntention(self, movement_intention):
        self.movement_detected = movement_intention.data
        if (self.movement_detected == True) and (self.past_state == False):
            self.activate_motors = True
            rospy.loginfo("Activating Motors")
        else:
            self.activate_motors = False
        self.past_state = self.movement_detected

    def automatic_movement(self):
        ''' Publisher Motor ID 1 and Motor ID 2'''
        self.frontal_motor_pub.publish(self.ValueToPubUp1)
        self.posterior_motor_pub.publish(self.ValueToPubDown2)
        time.sleep(1/self.frequency)
        self.frontal_motor_pub.publish(self.ValueToPubDown1)
        self.posterior_motor_pub.publish(self.ValueToPubUp2)
        time.sleep(1/self.frequency)

    def process(self):
        if self.activate_motors:
            self.automatic_movement()
            #release_motors()


def release_motors():
    value = False
    type_service = TorqueEnable
    service_frontal = '/tilt1_controller/torque_enable'
    service_posterior = '/tilt2_controller/torque_enable'
    ans = call_service(service=service_frontal, type=type_service, val = value)
    ans = call_service(service=service_posterior, type=type_service, val = value)

def set_motor_speed(speed):
    value = speed
    type_service = SetSpeed
    service_frontal = '/tilt1_controller/set_speed'
    service_posterior = '/tilt2_controller/set_speed'
    ans = call_service(service=service_frontal, type=type_service, val = value)
    ans = call_service(service=service_posterior, type=type_service, val = value)

def call_service(service,type,val):
    rospy.wait_for_service(service)
    try:
        srv =  rospy.ServiceProxy(service, type)
        ans = srv(val)
        return ans
    except rospy.ServiceException, e:
         rospy.loginfo("Service call failed: %s"%e)

def main():
    c = TherapyController()
    rospy.on_shutdown(release_motors)
    rospy.loginfo("Starting Therapy based on Game")
    while not (rospy.is_shutdown()):
        if c.kill_therapy:
            break
        c.process()
    rospy.loginfo("Controller Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)