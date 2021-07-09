#!/usr/bin/env python

import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from sensor_msgs.msg import JointState
from time import sleep

def jointStateToString(msg):
    return "pos={}, vel={} eff={}".format(msg.position, msg.velocity, msg.effort)

def gripperCommandCallback(msg):
    error_msg = "Invalid gripper command: " + jointStateToString(msg)

    if len(msg.position) == 0 or not (0 <= msg.position[0] <= 1):
        rospy.logerr(error_msg)

    if len(msg.velocity) == 0 or not (0 <= msg.velocity[0] <= 1):
        rospy.logerr(error_msg)
    
    if len(msg.position) == 0 or not (0 <= msg.effort[0] <= 1):
        rospy.logerr(error_msg)

    print("Received new gripper command: " + jointStateToString(msg))
    cmd = outputMsg.Robotiq2FGripper_robot_output()
    cmd.rACT = 1
    cmd.rGTO = 1
    cmd.rPR = int(msg.position[0] * 255)
    cmd.rSP = int(msg.velocity[0] * 255)
    cmd.rFR = int(msg.effort[0] * 255)
    print("{}, {}, {}".format(cmd.rPR, cmd.rSP, cmd.rFR))
    
    for i in range(10):
        pub.publish(cmd)
        rospy.sleep(0.1)

    
def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150

    if char == 'r':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0
        command.rGTO = 0
        command.rSP  = 0
        command.rFR  = 0


    if char == 'c':
        command.rPR = 255

    if char == 'o':
        command.rPR = 0   

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPR = int(char)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSP += 25
        if command.rSP > 255:
            command.rSP = 255
            
    if char == 'l':
        command.rSP -= 25
        if command.rSP < 0:
            command.rSP = 0

            
    if char == 'i':
        command.rFR += 25
        if command.rFR > 255:
            command.rFR = 255
            
    if char == 'd':
        command.rFR -= 25
        if command.rFR < 0:
            command.rFR = 0

    return command
        

def askForCommand(command):
    """Ask the user for a command to send to the gripper."""    

    currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
    currentCommand += '  rACT = '  + str(command.rACT)
    currentCommand += ', rGTO = '  + str(command.rGTO)
    currentCommand += ', rATR = '  + str(command.rATR)
    currentCommand += ', rPR = '   + str(command.rPR )
    currentCommand += ', rSP = '   + str(command.rSP )
    currentCommand += ', rFR = '   + str(command.rFR )


    print currentCommand

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'
    
    strAskForCommand += '-->'

    return raw_input(strAskForCommand)
                     

if __name__ == '__main__':
    #Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic.
    rospy.init_node('robotiq_2f_gripper_commander')
    rospy.sleep(5.0)
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
    sub = rospy.Subscriber('/gripper_command', JointState, gripperCommandCallback, queue_size=1)

    rospy.loginfo("Resetting Robotiq2fGripper")
    reset_cmd = outputMsg.Robotiq2FGripper_robot_output();
    reset_cmd = genCommand('r', reset_cmd)
    for i in range(10):
        pub.publish(reset_cmd)
        rospy.sleep(0.1)

    rospy.loginfo("Activating Robotiq2fGripper")
    activation_cmd = outputMsg.Robotiq2FGripper_robot_output();
    activation_cmd = genCommand('a', activation_cmd)
    
    for i in range(10):
        pub.publish(activation_cmd)
        rospy.sleep(0.1)

    rospy.spin()
   