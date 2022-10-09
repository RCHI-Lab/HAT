import time
import stretch_body.robot
import serial
import math
import pandas as pd
import numpy as np
import queue

ser=serial.Serial('/dev/ttyUSB3',baudrate=9600,timeout=1)
robot=stretch_body.robot.Robot()
robot.startup()

alpha=0
beta=0
theta=0
pitch=0
roll=0
yaw=0

def getInfo(s):
  data=[0,0,0]
  sep=s.split()
  if len(sep)>4:
      data[0]=float(sep[1])
      data[1]=float(sep[3])
      data[2]=float(sep[5])
  return data

def getAngV(accX, accY,accZ):
  pitch=180*math.atan2(accX,math.sqrt(accY*accY+accZ*accZ))/math.pi
  roll=180*math.atan2(accY,math.sqrt(accX*accX+accZ*accZ))/math.pi
  yaw=180*math.atan2(accZ,math.sqrt(accX*accX+accY*accY))/math.pi
  return (pitch,roll,yaw)

pitch_window=queue.Queue()
roll_window=queue.Queue()
yaw_window=queue.Queue()
aveP=0
aveR=0
aveY=0

def initialize(num):
  while 1:
    arduinoData=ser.readline()
    data=getInfo(arduinoData)
    print("x: ",data[0]," y: ",data[1]," z: ",data[2])
    accX=data[0]*3.9
    accY=data[1]*3.9
    accZ=data[2]*3.9
    if (accX!=0 or accY!=0 or accZ!=0):
      print("Connection established!\n")
      for i in range(num):
        data=getInfo(arduinoData)
        accX=data[0]*3.9
        accY=data[1]*3.9
        accZ=data[2]*3.9
        (pitch,roll,yaw)=getAngV(accX,accY,accZ) 
        aveP+=pitch
        aveR+=roll
        aveY+=yaw
        pitch_window.put(pitch)
        roll_window.put(roll)
        yaw_window.put(yaw)
      aveP/=num
      aveR/=num
      aveY/=num
      break

def update_ave(pitch,roll,yaw):
  p=pitch_window.get()
  r=roll_window.get()
  y=yaw_window.get()
  pitch_window.put(pitch)
  roll_window.put(roll)  
  yaw_window.put(yaw)
  size=pitch_window.qsize()
  aveP=(aveP*size-p+pitch)/size
  aveR=(aveR*size-r+roll)/size
  aveY=(aveY*size-y+yaw)/size

initialize(5)

while 1:
    arduinoData=ser.readline()
    data=getInfo(arduinoData)
    print("x: ",data[0]," y: ",data[1]," z: ",data[2])
    print("alpha: ",alpha," beta: ",beta," theta: ",theta)
    accX=data[0]*3.9
    accY=data[1]*3.9
    accZ=data[2]*3.9
    (pitch,roll,yaw)=getAngV(accX,accY,accZ) 
    update_ave(pitch,roll,yaw)
    alpha+=aveP
    beta+=aveR
    theta+=aveY
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
