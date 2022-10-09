#Import tkinter library
from tkinter import *
import socket
import time
from gtts import gTTS
from playsound import playsound

#Create an instance of tkinter frame
win= Tk()
win.geometry("1750x1450")
mode=0
prev=-1
num_mode=4

#set up socket communication
#host = '172.26.167.160' #client ip
host = '172.26.168.84' #new laptop client ip
port = 4005
#server = ('172.26.166.129', 4000) #robot 1082
server = ('172.26.163.219', 4000)  #robot 1075
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# timmer
time_pressed=0

def Text_to_speech(message):
    speech = gTTS(text = message)
    speech.save('Techarge.mp3')
    playsound('Techarge.mp3')
    

def connect_socket(event=None):
    global s
    s.bind((host,port))
    message="Client Connected"
    s.sendto(message.encode('utf-8'), server)
    Text_to_speech(message)
    add_highlighter()

def send_msg(message):
    global server, s
    s.sendto(message.encode('utf-8'), server)
    data, addr = s.recvfrom(1024)
    data = data.decode('utf-8')
    print("Received from server: " + data)
            
    while (data!=message):
        data, addr = s.recvfrom(1024)
        data = data.decode('utf-8')
        s.sendto(message.encode('utf-8'),server)
        print("Received from server: " + data)
    Text_to_speech(message)

def disconnect_socket(event=None):
    global s
    message="Client Disconnected"
    s.sendto(message.encode('utf-8'), server)
    Text_to_speech(message)
    s.close()

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

def disable_control(event=None):
    send_msg("disable")
    text.tag_add("disable", "9.1", "9.100")
    text.tag_config("disable", background="red", foreground="white")

def enable_control(event=None):
    send_msg("enable")
    text.tag_remove("disable","9.1","9.100")

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

