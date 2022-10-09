# Todo: Make the calibration program automatic

import socket
import time
import keyboard
import stretch_body.robot
import serial
import math
import numpy

#ser=serial.Serial('/dev/ttyUSB3',baudrate=115200,timeout=1)
ser=serial.Serial('/dev/rfcomm0',baudrate=115200,timeout=1)

# global variables
robot=stretch_body.robot.Robot()
robot.startup()
#robot.home()
#robot.stow()
robot.lift.set_soft_motion_limit_min(0.2,limit_type='user')
robot.lift.set_soft_motion_limit_max(0.98,limit_type='user')

button_prev=False
v_des=stretch_body.wrist_yaw.WristYaw().params['motion']['default']['vel']
a_des=stretch_body.wrist_yaw.WristYaw().params['motion']['default']['accel']

state=0
num_state=4

LOW=15
HIGH=30
roll_threshold_L=LOW
roll_threshold_R=-LOW
pitch_threshold_L=LOW
pitch_threshold_R=-LOW
host = '172.26.163.219' #Server ip 1075
# host = '172.26.166.129' #Server ip 1082
port = 4000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def connect_socket():
    s.bind((host, port))

    print("Server Started")

def disconnect_socket():    
    c.close()
    robot.stop()

# @brief: given the string received from the tinypico, extract x,y,z angles from the 
# string, and store them in data[0-2] respectively
# data[3] contains the mode switching information
def getInfo(s):
  data=[0,0,0,0]
  #print(s)
  sep=s.split()
  if len(sep)>4:
      data[0]=float(sep[1])
      data[1]=float(sep[3])
      data[2]=float(sep[5])
      # data[3]=int(sep[7]) info for mode switching
  return data

# @brief: Try to obtain xyz data from the bluetooth, and break out from the loop
# once connection is established
# (Connection is established when the data is not all 0s)
def initialize():
  while 1:
    arduinoData=ser.readline()
    data=getInfo(arduinoData)
    print("x: ",data[0]," y: ",data[1]," z: ",data[2])
    X=data[0]
    Y=data[1]
    Z=data[2]
    if (X!=0 or Y!=0 or Z!=0):
      print("Connection established!\n")
      break

# @brief: Set the threshold for each direction
def calibrate():
    roll_threshold_L=LOW
    roll_threshold_R=-LOW
    pitch_threshold_L=LOW
    pitch_threshold_R=-LOW


# state 0: base movement
# state 1: arm movement
# state 2: wrist
# state 3: gripper movement
# (state 4: stop)
initialize()
calibrate()

def is_pressed(msg):
    if (msg=="Switch"):
        return True
    elif (msg=="No operation"):
        return False
    else:
        print("Unexpected message"+msg)
        disconnect_socket()
        return False

def update_mode(msg):
    if (is_pressed(msg) and button_prev==False ):
        button_prev=True
        state=(state+1)%num_state
    elif (is_pressed(msg)==False):
        button_prev


while 1:
    arduinoData=ser.readline()
    data=getInfo(arduinoData)
    #print("Sending: " + data)
    #s.sendto(data.encode('utf-8'), addr)
    #update_mode(msg)
    print("x: ",data[0]," y: ",data[1]," z: ",data[2], "mode: ", state)
    yaw=data[0]
    pitch=data[1]
    roll=data[2]
    # button=data[3]
    print("pitch: ",pitch," roll: ",roll," yaw: ",yaw)


    if state== 0:
        if roll>roll_threshold_L:
            speed=(roll-roll_threshold_L)/100
            robot.base.rotate_by(speed)
        elif roll<roll_threshold_R:
            speed=(roll-roll_threshold_R)/100
            robot.base.rotate_by(speed)
    
        if pitch>pitch_threshold_L:
            speed=(pitch-pitch_threshold_L)/100
            robot.base.translate_by(speed)
        elif pitch<pitch_threshold_R:
            speed=(pitch-pitch_threshold_R)/100
            robot.base.translate_by(speed)
    
    if state== 1:
        #speed0=(abs(roll)-25)/100
        #speed1=(abs(pitch)-25)/100

        if roll>roll_threshold_L:
            speed=(roll-roll_threshold_L)/100
            robot.arm.move_by(speed)
        elif roll<roll_threshold_R:
            speed=(roll-roll_threshold_R)/100
            robot.arm.move_by(speed)
    
        if pitch>pitch_threshold_L:
            speed=(pitch-pitch_threshold_L)/100
            robot.lift.move_by(-speed)
        elif pitch<pitch_threshold_R:
            speed=(pitch-pitch_threshold_R)/100
            robot.lift.move_by(-speed)

    if state== 2:
        wpitch=robot.end_of_arm.status['wrist_pitch']['pos']
        wyaw=robot.end_of_arm.status['wrist_yaw']['pos']
        #speed0=(abs(roll)-25)/10
        #speed1=(abs(pitch)-25)/10
        if roll>roll_threshold_L:
            speed=(roll-roll_threshold_L)/10
            robot.end_of_arm.move_by('wrist_yaw',float(-math.radians(speed)),v_des, a_des)
        elif roll<roll_threshold_R:
            speed=(roll-roll_threshold_R)/10
            robot.end_of_arm.move_by('wrist_yaw',float(-math.radians(speed)),v_des, a_des)
   
        if pitch>pitch_threshold_L:
            speed=(pitch-pitch_threshold_L)/10
            robot.end_of_arm.move_by('wrist_pitch',float(-math.radians(speed)),v_des, a_des)
        elif pitch<pitch_threshold_R:
            speed=(pitch-pitch_threshold_R)/10
            robot.end_of_arm.move_by('wrist_pitch',float(-math.radians(speed)),v_des, a_des)

    if state== 3:
        #speed0=(abs(roll)-10)/5
        if roll>roll_threshold_L:
            speed=(roll-roll_threshold_L)/5
            robot.end_of_arm.move_by('stretch_gripper',speed)
        elif roll<roll_threshold_R:
            speed=(roll-roll_threshold_R)/5
            robot.end_of_arm.move_by('stretch_gripper',speed)
    #if state== 4:
    #    robot.stop()

    robot.push_command()
    # robot.arm.move_by(-0.1)
    # robot.base.rotate_by(0.3)
    # robot.push_command()
    # time.sleep(2.0)

    # robot.base.translate_by(-0.3)
    # time.sleep(2.0)

