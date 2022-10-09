import time
import stretch_body.robot
import serial
import math

ser=serial.Serial('/dev/ttyUSB3',baudrate=9600,timeout=1)
robot=stretch_body.robot.Robot()
robot.startup()

def getInfo(s):
  data=[0,0,0]
  sep=s.split()
  if len(sep)>4:
      data[0]=float(sep[1])
      data[1]=float(sep[3])
      data[2]=float(sep[5])
  return data

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

alpha=0
beta=0
theta=0
initialize()

while 1:
    arduinoData=ser.readline()
    data=getInfo(arduinoData)
    print("x: ",data[0]," y: ",data[1]," z: ",data[2])
    print("alpha: ",alpha," beta: ",beta," theta: ",theta)
    X=data[0]
    Y=data[1]
    Z=data[2]
    accX=X*3.9
    accY=Y*3.9
    accZ=Z*3.9
    if (abs(X)>0.05 or (abs(Y)>0.05) or (abs(Z)>0.05)):
      pitch=180*math.atan2(accX,math.sqrt(accY*accY+accZ*accZ))/math.pi
      roll=180*math.atan2(accY,math.sqrt(accX*accX+accZ*accZ))/math.pi
      yaw=180*math.atan2(accZ,math.sqrt(accX*accX+accY*accY))/math.pi
      alpha+=pitch
      beta+=roll
      theta+=yaw
        #xz_ang=math.degrees(math.atan(data[2]/data[0]))
        # robot.arm.move_by(-0.1)
        # robot.lift.move_by(0.1)
      
        #robot.push_command()
      #if alpha>150:
      #  robot.base.translate_by(0.05)
      #elif alpha<-150:
      #  robot.base.translate_by(-0.05)
      print("pitch: ",pitch," roll: ",roll," yaw: ",yaw)
      print("alpha: ",alpha," beta: ",beta," theta: ",theta)
      
    #time.sleep(0.01)
    if theta>200:
      robot.base.rotate_by(0.05)
    elif theta<-200:
      robot.base.rotate_by(-0.05)
    
    if beta>150:
      robot.base.translate_by(0.05)
    elif beta<-150:
      robot.base.translate_by(-0.05)
    robot.push_command()
    # robot.arm.move_by(-0.1)
    # robot.base.rotate_by(0.3)
    # robot.push_command()
    # time.sleep(2.0)

    # robot.base.translate_by(-0.3)
    # time.sleep(2.0)

robot.stop()
