#!/usr/bin/env python
import rospy

from iris_cobot.msg import TapAction
from sami.gripper import Gripper

gripper = None # type:Gripper
num_actions = 0

def gripperLed(action):
    if action.type == "double":
        global num_actions
        num_actions += 1
        print(num_actions)
        if action.component == 0:
            if action.direction:
                print("X Positive")
                gripper.set_led_animation(2)
                gripper.set_led_color(1)
            else:
                print("X Negative")
                gripper.set_led_animation(2)
                gripper.set_led_color(7)
        if action.component == 1:
            if action.direction:
                print("Y Positive")
                gripper.set_led_animation(2)
                gripper.set_led_color(2)
            else:
                print("Y Negative")
                gripper.set_led_animation(2)
                gripper.set_led_color(3)
        if action.component == 2:
            if action.direction:
                print("Z Positive")
                gripper.set_led_animation(14)
            else:
                print("Z Negative")
                gripper.set_led_animation(14)
        
        
    

def main():
    rospy.init_node('gripper_led', anonymous=True)

    # Global gripper
    global gripper
    gripper = Gripper('cr200-85', host='10.1.0.2', port=44221)

    gripper.set_led_preset(1)
    gripper.set_led_animation(14)
    gripper.set_led_speed(4)

    rospy.Subscriber("tap_action", TapAction, gripperLed, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
