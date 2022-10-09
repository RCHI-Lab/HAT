# Todo: Make the calibration program automatic
# This version works with mouse_socket.py client 
import select
import socket
import time
import keyboard
import stretch_body.robot
import serial
import math
import numpy

ser=serial.Serial('/dev/rfcomm0',baudrate=115200,timeout=1)

# global variables
robot=stretch_body.robot.Robot()
robot.startup()
#robot.home()
# robot.stow()
robot.lift.set_soft_motion_limit_min(0.2,limit_type='user')
robot.lift.set_soft_motion_limit_max(0.98,limit_type='user')

# speed scaling factors
base_factor=70
arm_factor=100
wrist_factor=4
gripper_factor=4


head_control=True
v_des=stretch_body.wrist_yaw.WristYaw().params['motion']['default']['vel']
a_des=stretch_body.wrist_yaw.WristYaw().params['motion']['default']['accel']

state=0
num_state=4

LOW=15
HIGH=30
roll_threshold_L=LOW
roll_threshold_R=-LOW
pitch_threshold_L=LOW/2
pitch_threshold_R=-LOW
host = '172.26.163.219' #Server ip 1075
# host = '172.26.166.129' #Server ip 1082
port = 4000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(0.0)
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


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
  robot.head.move_to('head_tilt', math.radians(-90))
  robot.push_command()
  while 1:
    arduinoData=ser.readline()
    data=getInfo(arduinoData)
    print("x: ",data[0]," y: ",data[1]," z: ",data[2])
    X=data[0]
    Y=data[1]
    Z=data[2]
    if (X!=0 or Y!=0 or Z!=0):
      print("Connection established!\n")
      connect_socket()
      head_control=True
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

def update_mode(msg):
    global state, head_control
    if ( msg == "Switch"):
        state=(state+1)%num_state
        head_control=True
    elif (msg == "Client connected"):
        head_control=True
        print(msg)
    elif (msg=="disable"):
        head_control=False
    elif (msg=="enable"):
        head_control=True

while 1:
    arduinoData=ser.readline()
    data=getInfo(arduinoData)
    #s.listen(1)
    #msg, addr= s.accept()
    #timeout=0.00
    #ready_sockets, _, _=select.select([conn],[],[],timeout)
    #if ready_sockets:
    #msg=conn.recv(10)
    try:
        msg, addr = s.recvfrom(1024)
        msg = msg.decode('utf-8')
        print("Message from: " + str(addr))
        print("From connected user: " + msg)
        print("Sending: " + msg)
        s.sendto(msg.encode('utf-8'), addr)
        update_mode(msg)
    except socket.error:
        pass

    print("x: ",data[0]," y: ",data[1]," z: ",data[2], "mode: ", state)
    yaw=data[0]
    pitch=data[1]
    roll=data[2]
    # button=data[3]
    print("pitch: ",pitch," roll: ",roll," yaw: ",yaw)

    if (head_control==True):
        if state== 0:
            # base
            robot.head.move_to('head_pan', math.radians(0));
            if roll>roll_threshold_L:
                speed=(roll-roll_threshold_L)/base_factor
                robot.base.rotate_by(-speed)
            elif roll<roll_threshold_R:
                speed=(roll-roll_threshold_R)/base_factor
                robot.base.rotate_by(-speed)
    
            if pitch>pitch_threshold_L:
                speed=(pitch-pitch_threshold_L)/base_factor
                robot.base.translate_by(speed)
            elif pitch<pitch_threshold_R:
                speed=(pitch-pitch_threshold_R)/base_factor
                robot.base.translate_by(speed)
    
        if state== 1:
            # arm
            robot.head.move_to('head_pan', math.radians(90));
            if roll>roll_threshold_L:
                speed=(roll-roll_threshold_L)/arm_factor
                robot.arm.move_by(speed)
            elif roll<roll_threshold_R:
                speed=(roll-roll_threshold_R)/arm_factor
                robot.arm.move_by(speed)
    
            if pitch>pitch_threshold_L:
                speed=(pitch-pitch_threshold_L)/arm_factor
                robot.lift.move_by(-speed)
            elif pitch<pitch_threshold_R:
                speed=(pitch-pitch_threshold_R)/arm_factor
                robot.lift.move_by(-speed)

        if state== 2:
            # wrist
            robot.head.move_to('head_pan', math.radians(90));
            wpitch=robot.end_of_arm.status['wrist_pitch']['pos']
            wyaw=robot.end_of_arm.status['wrist_yaw']['pos']
            if roll>roll_threshold_L:
                speed=(roll-roll_threshold_L)/wrist_factor
                robot.end_of_arm.move_by('wrist_yaw',float(-math.radians(speed)),v_des, a_des)
            elif roll<roll_threshold_R:
                speed=(roll-roll_threshold_R)/wrist_factor
                robot.end_of_arm.move_by('wrist_yaw',float(-math.radians(speed)),v_des, a_des)
   
            if pitch>pitch_threshold_L:
                speed=(pitch-pitch_threshold_L)/wrist_factor
                robot.end_of_arm.move_by('wrist_pitch',float(-math.radians(speed)),v_des, a_des)
            elif pitch<pitch_threshold_R:
                speed=(pitch-pitch_threshold_R)/wrist_factor
                robot.end_of_arm.move_by('wrist_pitch',float(-math.radians(speed)),v_des, a_des)

        if state== 3:
            # gripper
            robot.head.move_to('head_pan', math.radians(90));
            if pitch>pitch_threshold_L:
                speed=(pitch-pitch_threshold_L)/gripper_factor
                robot.end_of_arm.move_by('stretch_gripper',speed)
            elif pitch<pitch_threshold_R:
                speed=(pitch-pitch_threshold_R)/gripper_factor
                robot.end_of_arm.move_by('stretch_gripper',speed)

    robot.push_command()
    # robot.arm.move_by(-0.1)
    # robot.base.rotate_by(0.3)
    # robot.push_command()
    # time.sleep(2.0)

    # robot.base.translate_by(-0.3)
    # time.sleep(2.0)

