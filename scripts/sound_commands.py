#!/usr/bin/python
import rospy
from std_msgs.msg import Bool, Header
from time import sleep
import pygame

class Generacion_pulsos:
    def __init__(self):
        self.inicio = True
        self.frec1 = True
        self.frec2 = False
        self.frec3 = False
        self.pulses = 1
        self.kill_process = False
        pygame.mixer.init()
        pygame.mixer.music.load("/home/pi/pulso.wav")
        ''' ROS commands '''
        rospy.init_node('sound_commands', anonymous = True)
        rospy.Subscriber("/kill_sound_commands", Bool, self.updateFlagSoundCommands)
        self.sound_command_pub = rospy.Publisher("/sound_command", Header, queue_size = 1, latch = False)
        self.msg = Header()

    def updateFlagSoundCommands(self, flag):
        self.kill_process = flag.data


    def comienzo(self):
        self.msg.frame_id = "/sound_command"
        if self.frec1 == True and self.pulses <= 56:
            #server = Server().boot()
            #server.start()
            sleep(3)
            #sine = Sine(261.63).out()
            self.msg.seq = self.pulses 
            self.msg.stamp = rospy.Time.now()
            self.sound_command_pub.publish(self.msg)
            pygame.mixer.music.play()
            #sleep(0.5)
            #server.stop()
            self.pulses += 1
        elif self.pulses > 56 and self.frec1 == True:
            self.pulses = 1
            self.frec1 = False
            self.frec2 = True
        if self.frec2 == True and self.pulses <= 80:
            #server = Server().boot()
            #server.start()
            sleep(2)
            #sine = Sine(261.63).out()
            self.msg.seq = self.pulses 
            self.msg.stamp = rospy.Time.now()
            self.sound_command_pub.publish(self.msg)
            pygame.mixer.music.play()
            #sleep(0.5)
            #server.stop()
            self.pulses += 1
        elif self.pulses > 80 and self.frec2 == True:
            self.pulses = 1
            self.frec2 = False
            self.frec3 = True
        if self.frec3 == True and self.pulses <= 100:
            #server = Server().boot()
            #server.start()
            sleep(1.5)
            #sine = Sine(261.63).out()
            self.msg.seq = self.pulses 
            self.msg.stamp = rospy.Time.now()
            self.sound_command_pub.publish(self.msg)
            pygame.mixer.music.play()
            #sleep(0.5)
            #server.stop()
            #self.pulses += 1
        elif self.pulses > 100 and self.frec3 == True:
            self.inicio = False
            self.kill_process = True  

    

def main():
    pulsos = Generacion_pulsos()
    rate = rospy.Rate(10) # Sensor Frequency 10 Hz
    sleep(10) 
    rospy.loginfo("Starting Sound Commands")
    while not (rospy.is_shutdown()):
        try:
            if pulsos.kill_process:
                break
            pulsos.comienzo()
        except Exception as e:
            print(e)
        rate.sleep()
    rospy.loginfo("Process Finished")


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
