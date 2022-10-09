import keyboard
import time


with open('keyboard_commands.txt', 'w') as f:
    f.write('n')

while True:
    command = raw_input("Enter command: ")
    if command == 's':
        print("stopping robot")
        with open('keyboard_commands.txt', 'w') as f:
            f.write('s')
    elif command == 'e':
        print("ending task")
        with open('keyboard_commands.txt', 'w') as f:
            f.write('e')
    else: 
        with open('keyboard_commands.txt', 'w') as f:
            f.write('n')
    time.sleep(5)
    with open('keyboard_commands.txt', 'w') as f:
        f.write('n')
'''
space = False

def space_pressed():
    global space
    print('pressed')
    space = True

keyboard.add_hotkey('space', space_pressed)

while True:
    print(space)
    if space == True:
        with open('keyboard_commands.txt', 'w') as f:
            f.write('s')
            space = False
'''