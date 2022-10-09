import socket
import time
from gtts import gTTS
from playsound import playsound
import os
import select


import speech_recognition as sr

#print(sr.Microphone.list_microphone_names())



#set up socket communication
host = '______' #new laptop client ip
port = 4005
server = ('______', 4000)  # Fill in with <Robot IP>
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(5)


mode = int(input("Enter mode switching choice (1 for speech, 2 for cycle): "))
if mode == 1:
    print("Using Speech")
    detection_gap = 2.5
elif mode == 2:
    print("Using Cycle")
    detection_gap = 1.5

state = 0
max_state = 4 
start_pause = 1 #0 is start, 1 is pause

def Text_to_speech(message): #call this to save the audio file 
    speech = gTTS(text = message)
    speech.save('started1.mp3')
    playsound('started1.mp3')

def cycle_mode_switching(data):
    global start_pause, state
    if data == 'd2': #start-stop messages
        start_pause +=1
        if start_pause > 1:
            start_pause = 0
        if start_pause == 0:
            message= "start"
        elif start_pause == 1:
            message = "pause"
    elif data == 'd1': #state messages
        state +=1
        if state > max_state:
            state = 0
        if state == 0:
            message = "drive"
        elif state == 1:
            message = "arm"
        elif state == 2:
            message = "wrist"
        elif state == 3:
            message = "gripper"
        elif state == 4:
            message = "camera"
    
    send_msg(message)

def speech_to_text():
    with sr.Microphone(device_index = 1) as source:
        r = sr.Recognizer()
        r.adjust_for_ambient_noise(source, duration = 0.5)
        playsound("beep-2.wav")
        print("Say something!")
        #audio = r.listen(source)
        #audio = r.listen_in_background(source,callback)
        audio = r.record(source, duration = 2)
    # recognize speech using Google Speech Recognition
    try:
        text = (r.recognize_google(audio)).lower()
        t = text.split()
        print(t)
        #print("Google Speech Recognition thinks you said " + r.recognize_google(audio))
        drive = ['drive', 'tries', 'base']
        arm = ['arm', 'armed', 'ar', 'alarm','arms', 'charm']
        wrist = ['wrist', 'rest', 'risk']
        gripper = ['gripper', 'stripper', 'gerber', 'grandpa','ripper']
        pause = ['pause', 'paw', 'paws', 'cause']
        start = ['start']
        end = ['end', 'and']
        camera = ['camera']
        inlist = False
        for t in text.split():
            if t in drive:
                message = "drive"
                inlist = True
                break
            elif t in arm:
                message = "arm"
                inlist = True
                break
            elif t in wrist:
                message = "wrist"
                inlist = True
                break
            elif t in gripper:
                message = "gripper"
                inlist = True
                break
            elif t in camera:
                message = "camera"
                inlist = True
                break
            elif t in pause:
                message = "pause"
                inlist = True
                break
            elif t in start:
                message = "start"
                inlist = True
                break
            elif t in end:
                message = "end"
                inlist = True
                break
        if inlist == False:
            print("Repeat")
            playsound('repeat1.mp3')
        else:
            send_msg(message)
            pass
    except sr.UnknownValueError:
        #print("Google Speech Recognition could not understand audio")
        print("Repeat")
        playsound('repeat1.mp3')
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
    
def play_cofirmation_sound(message):
    filenamestart = "C:/Users/rchi/Desktop/GUI/"
    if message == "drive":
        print("Drive Mode")
        playsound(filenamestart + 'drive1.mp3')   
    elif message ==  "arm":
        print("Arm Mode")
        playsound(filenamestart + 'arm1.mp3')
    elif message == "wrist":
        print("Wrist Mode")
        playsound(filenamestart +'wrist1.mp3')
    elif message == "gripper":
        print("Gripper Mode")
        playsound(filenamestart +'gripper1.mp3')
    elif message == "camera":
        print("Camera Mode")
        playsound(filenamestart +'camera1.mp3')
    elif message == "pause":
        print("Pause Mode")
        playsound(filenamestart +'pause1.mp3')
    elif message == "start":
        print("Start/Calibrate Mode")
        playsound(filenamestart +'started1.mp3')
        time.sleep(0.5)
        playsound(filenamestart +'calibrated1.mp3')
    elif message == "end":
        print("Ending")
        playsound(filenamestart +'ending1.mp3')
    



def send_msg(message):
    global server, s
    try:
        s.sendto(message.encode('utf-8'), server)
        data, addr = s.recvfrom(1024)
        data = data.decode('utf-8')
        print("Received from server: " + data)
        start = time.time()        
        while (data!=message):
            data, addr = s.recvfrom(1024)
            data = data.decode('utf-8')
            s.sendto(message.encode('utf-8'),server)
            print("Received from server: " + data)
        if data == message:
            play_cofirmation_sound(message)

    except socket.error:
        pass
    #Text_to_speech(message)


while 1: 
    try:
        print('Sent connection message to robot')
        message = 'connecting'
        s.sendto(message.encode('utf-8'), server)
        msg, addr = s.recvfrom(1024)
        msg = msg.decode('utf-8')
        if msg == 'connecting':
            print("Connected")
            break
    except socket.error:
        pass

last_detection_time = time.time()
while 1:
    curr_time = last_detection_time+3
    try:
        data, addr = s.recvfrom(1024)
        data = data.decode('utf-8')
        if (data == 'd1' or data == 'd2') and curr_time-last_detection_time > detection_gap:
            if mode == 1:
                speech_to_text()
            elif mode == 2:
                print('here')
                cycle_mode_switching(data)
            last_detection_time = curr_time
    except socket.error:
        pass







'''
recycled GUI code using tkinter:

#Import tkinter library
#from curses.textpad import Textbox
from tkinter import *

def disconnect_socket(event=None):
    global s
    message="Client Disconnected"
    s.sendto(message.encode('utf-8'), server)
    Text_to_speech(message)
    s.close()

def connect_socket(event=None):
    global s
    s.bind((host,port))
    message="Client Connected"
    s.sendto(message.encode('utf-8'), server)
    Text_to_speech(message)
    add_highlighter()

def disable_control(event=None):
    send_msg("disable")
    text.tag_add("disable", "9.1", "9.100")
    text.tag_config("disable", background="red", foreground="white")

def enable_control(event=None):
    send_msg("enable")
    text.tag_remove("disable","9.1","9.100")

def switch_mode(event=None):
    send_msg("Switch")
    add_highlighter()

#Define a function to highlight the text
def add_highlighter(event=None):
    global mode,prev
    if prev==0:
        text.tag_remove("Mode1", "2.1","2.50")
    elif prev==1:
        text.tag_remove("Mode2", "3.1","3.50")
    elif prev==2:
        text.tag_remove("Mode3", "4.1","4.50")
    elif prev==3:
        text.tag_remove("Mode4", "5.1","5.50")

    if mode==0:
        text.tag_add("Mode1", "2.1","2.50")
        text.tag_config("Mode1", background= "black", foreground= "white")
    elif mode==1:
        text.tag_add("Mode2", "3.1","3.50")
        text.tag_config("Mode2", background= "black", foreground= "white")
    elif mode==2:
        text.tag_add("Mode3", "4.1","4.50")
        text.tag_config("Mode3", background= "black", foreground= "white")
    elif mode==3:
        text.tag_add("Mode4", "5.1","5.50")
        text.tag_config("Mode4", background= "black", foreground= "white")
    prev=mode
    mode=(mode+1)%num_mode
    
#Create a Tex Field
text= Text(win);
text.insert(INSERT, "Current mode:\n\
        Mode 1: Base movement\n\
        Mode 2: Arm movement\n\
        Mode 3: Wrist movement\n\
        Mode 4: Gripper movement\n\
        \n\n\n\
        Head movement control for the robot is temporarily disabled\n")
text.pack()

# uncomment to use spacebar for mode switching
# win.bind("<space>",add_highlighter)  

# Create a Button to highlight text
button1=Button(win, text= "Left click to switch Mode\n\nRight click to disable\nhead movement control for the \nrobot temporarily", height=10, width=30)#.pack(pady=20)
button1.pack(pady=20)
button1.bind('<Button-1>', switch_mode)
button1.bind('<Button-3>', disable_control)
button1.bind('<ButtonRelease-3>', enable_control)

#Create a button to connect socket
button2=Button(win, text="connect robot", command=connect_socket).pack(pady=20)

#Create a button to disconnect socket
button3=Button(win, text="disconnect robot", command=disconnect_socket).pack(pady=20)

win.mainloop()

'''
