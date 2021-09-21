#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from t_flex.msg import IMUData
import sys


class ImuPublisher(object):
    def __init__(self):
        rospy.init_node('new_imu_publisher', anonymous = True)
        self.initSubscribers()
        self.initPublishers()
      
    def initSubscribers(self):
        rospy.Subscriber("/imu", Imu, self.updateImuData)
        rospy.Subscriber("/kill_imu_data", Imu, self.updateKillImuData)
        
    def initPublishers(self):
        self.pub_imu_data = rospy.Publisher("/imu_data", IMUData, queue_size = 1, latch = False)
        
    def updateImuData(self, data):
        msg = IMUData()
        
        msg.gyro_x = data.angular_velocity.x
        msg.gyro_y = data.angular_velocity.y
        msg.gyro_z = data.angular_velocity.z
        
        msg.accel_x = data.linear_acceleration.x
        msg.accel_y = data.linear_acceleration.y
        msg.accel_z = data.linear_acceleration.z
        
        msg.header.seq = data.header.seq        
        msg.header.stamp = data.header.stamp
        msg.header.frame_id = data.header.frame_id
		
        self.pub_imu_data.publish(msg)

    def updateKillImuData(self,data):
        if data.data:
            rospy.loginfo("Killing node...")
            sys.exit(0)


def main():
    imu = ImuPublisher()
    rospy.loginfo("Publishing IMU data...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt as e:
        print(e)