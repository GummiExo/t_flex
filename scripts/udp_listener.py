#! /usr/bin/python

import rospy
import logging
import socket
import sys, getopt
from std_msgs.msg import Bool

class UDPListener(object):
    def __init__(self):
        opts, args = getopt.getopt(sys.argv[1:], "h:p:", [])
        for opt, arg in opts:
            if opt == "-h":
                self.host = arg
            elif opt == "-p":
                self.port = int(arg)
        rospy.init_node('udp_listener', anonymous = True)
        self.flag = rospy.Publisher("/enable_device", Bool, queue_size = 1, latch = False)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.host, self.port))
        FORMAT_CONS = '%(asctime)s %(name)-12s %(levelname)8s\t%(message)s'
        logging.basicConfig(level=logging.DEBUG, format=FORMAT_CONS)
        rospy.loginfo('UDP Server..\nListen on Host: ' + self.host + ' Port: '+ str(self.port))
        self.command = None

    def process(self):
        (data, addr) = self.s.recvfrom(128*1024)
        try:
            if ((int(data) == 0) or (int(data) == 1)):
                if not (self.command == int(data)):
                    self.command = int(data)
                    yield (self.command)
            else:
                rospy.logerr('Data received must be 0 or 1')
        except:
            rospy.logerr('Type data received must be a integer [0-1]')


def main():
    l = UDPListener()
    while not (rospy.is_shutdown()):
        for l.command in l.process():
            if l.command:
                rospy.loginfo('Assisting Dorsiflexion')
                l.flag.publish(True)
            else:
                rospy.loginfo('Assisting Plantarflexion')
                l.flag.publish(False)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print e
