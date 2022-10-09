# Wearable-Headband-Interface
The code accompanies the submission: [HAT: Head-Worn Assistive Teleoperation of Mobile Manipulators](https://arxiv.org/abs/2209.13097).<br>
Akhil Padmanabha, Qin Wang, Daphne Han, Jashkumar Diyora, Kriti Kacker, Hamza Khalid, Liang-Jung Chen, Carmel Majidi, Zackory Erickson

This repository will contain the required software we are using for the hat interface. 
For video demos, visit https://sites.google.com/view/hat-teleop/home.

## Pre-requisities
### Hardware Architecture:
![Architecture](https://user-images.githubusercontent.com/66550924/194370025-66da0544-8f57-47f5-a286-899a2da01dfc.png).
- Hat: Follow the website [https://sites.google.com/view/hat-teleop/home](https://sites.google.com/view/hat-teleop/home/build-instructions) for hat assembly instruction.
- Stretch RE1 robot https://docs.hello-robot.com/0.2/. 
- [HDMI dongle](https://www.amazon.com/Headless-Display-Emulator-Headless-1920x1080-Generation/dp/B06XT1Z9TF/ref=asc_df_B06XT1Z9TF/?tag=hyprod-20&linkCode=df0&hvadid=309751315916&hvpos=&hvnetw=g&hvrand=1849427447759673039&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9005925&hvtargid=pla-547341237007&psc=1&tag=&ref=&adgrpid=67183599252&hvpone=&hvptwo=&hvadid=309751315916&hvpos=&hvnetw=g&hvrand=1849427447759673039&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9005925&hvtargid=pla-547341237007) plugged into the robot for remote desktop.
- Wireless earbuds with built-in microphone.
- Laptop 1: For remote control of the robot computer.
- Laptop 2: For speech recognition processing. (The "companion laptop" in the figure is Laptop 2). <br>
(Note: Laptop 1 and 2 can possibly be combined into using a single computer, using Remote Desktop to control the robot computer and using terminal to run the speech recognition script. Here we call them Laptop 1 and Laptop 2 for easy reference.)

### Software Installation
#### On laptop 1:
1. Remote desktop control of the robot using either [Getscreen.me](https://getscreen.me/) or [DWService](https://www.dwservice.net/).
2. On Stretch RE1, create a new user account and log into that account.
3. Install the required dependencies: 
```sh
pip install -r Requirements_laptop1.txt 
```
4. Clone the repository to your Stretch RE1 robot:
```sh
git clone https://github.com/Bread-wq/Wearable-Headband-Interface.git
```

#### On laptop 2:
1. Install the required dependencies: 
```sh
pip install -r Requirements_laptop2.txt 
```
2. Connect to the earbuds via bluetooth.
3. Clone the repository to laptop 2:
```sh
git clone https://github.com/Bread-wq/Wearable-Headband-Interface.git
```

#### On TinyPico ESP32:
1. Connect TinyPico on one of the computers.
2. Open Arduino and upload `IMU\_Button\_bluetooth.ino` to your TinyPico.


### Wireless communication
#### Bluetooth communication between TinyPico and robot:
1. Obtain the MAC address of your TinyPico ESP32 `<dev>`
2. Install [BlueZ](http://www.bluez.org/)
3. Run
```sh
rfkill unblock all
bluetoothctl
```
4. Pair using `bluetoothctl`:
```sh
power on
agent on
scan on
pair <dev>
```
Exit `bluetoothctl` by pressing `ctl d`.

5. Create serial device:

```sh
sudo rfcomm bind 0 <dev>
```
Now the TinyPico is connected to Stretch RE1.


#### Serial communication between robot (controled on laptop 1) and Laptop 2:
1. Obtain the IP address of your Stretch RE1 robot, `<Robot IP>`

- Go to main.py, replace the IP address on line 181 with ```<Robot IP>``` obtained.
- On Laptop 2, in `GUI/Voice_recognition.py`, replace the server IP address on line 23 with `<Robot IP>`.

2. Similarly, get the IP address of the Laptop 2 for running speech recognition, `<Comp IP>`.
- In `GUI/Voice_recognition.py`, replace the host IP address on line 20 with `<Comp IP>`.


## Start Experiment:
1. On the robot, open <terminal 1> and run the following command to home the robot and listen to the sign to stop data recording before running each experiment.
```sh
python home.py
sudo python keyboard.py
```

2. Open <terminal 2> and run: 
```sh
python main.py
```
- Enter task number and trial number as prompted.
- Enter 1 for speech recognition mode, or 2 for cycle mode.

3. On Laptop 2, run 
```sh
cd GUI
python3 Voice_recognition.py
```

- Enter 1 for speech recognition mode, or 2 for cycle mode.

The Laptop 2 will connect to the robot via socket communication, and <terminal 2> on robot should print "Connected".

## Instructions on how to switch mode [Video instruction](https://www.youtube.com/watch?v=v8wXM-cCss0) for more details.
### Method 1: Speech recognition
1. To send a speech command: 
- Wear the hat and shake the head left and right, after a "beep" sound, say the intended command. (Shake your head again if no "beep" sound is heard).
- After each head shake, the program will be listening to the input for 3s, so try to say it clearly once "beep" sound is heard
- If the speech recognition parses the word correctly, it play out the command heard to confirm that is correct.
- If the speech recognition fails to identify the phrase said, it will say "Repeat", and we need to repeat shaking the head and saying the command again.

2. Calibration command:
- Command phrase: "start".
- After laptop 2 capture the command correctly, hold the head still for 2 seconds to wait for the computer to say "calibrated". This is to record the natural head position.
- Note: The default mode will be "drive mode" when the robot first starts.

3. Mode switching commands: wear the hat and shake the head left and right, after a "beep" sound, say "start". <br>
- Commands are in the form of: "switch to <mode>"
- Modes include "drive", "arm", "wrist", "gripper"

### Method 2: Cycle 
If you choose Cycle mode, each time you shake your head, the mode will switch to the next one in the following order:
- "drive", "arm", "wrist", "gripper"

## Stop the robot
1. To stop the robot from moving temporarily, move your head to the calibrated position.
2. To stop the hat from controlling the robot in speech recognition mode, shake the head and say "pause".
3. If the researcher wants to stop the experiment and stop the robot, send 's' in the terminal with keyboard.py running or press the E-Stop button on the robot to fully stop it.
  
![EStop button: the white button shown](https://user-images.githubusercontent.com/66550924/194368128-14fd9672-23ec-4a38-b5bf-83271cb101be.png)
  
  *Emergency Stop Button*

## Saving Experiment Data
1. On line 15 in `main.py`, fill in participant number.
2. On line 85 in `main.py`, fill in `user_data_path= '_____'` with the intended directory to save data.
3. Enter 'e' in <terminal 1> (running `keyboard.py`), <terminal 2> (running `main.py`) should print 'Saved Data' 
4. `Ctrl-C` to terminate both `main.py` and `Voice_recognition.py`
5. Data saved: 
- IMU data: accelerometer data in all 3 axes  
- Mode data: the mode robot is in and the movement command sent to the robot
  - Mode includes: "drive", "arm", "wrist", "gripper"
  - Movement commands are encoded by 2 characters: e.g. 'wr' means "wrist right", 'wl' means "wrist left", and 'ww' means "wrist wait"
- Force data: the force applied by the robot lift

