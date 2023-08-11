import serial
import time
from inputs import get_gamepad

ser = serial.Serial('/dev/ttyS0', 9600)  # Adjust the port if necessary

def send_command(command):
    ser.write(command.encode())

motor_direction = 0  # 0: Stopped, 1: Clockwise direction, 2: Counterclockwise direction

try:
    while True:
        events = get_gamepad()
        direction_updated = False  # Variable to track if the direction was updated

        for event in events:
            if event.ev_type == 'Key':
                if event.code == 'BTN_TR':
                    if event.state == 1:  # R1 pressed
                        if motor_direction != 1:
                            send_command('1')
                            motor_direction = 1
                            direction_updated = True
                            print("Rotating in clockwise direction.")
                elif event.code == 'BTN_TL':
                    if event.state == 1:  # L1 pressed
                        if motor_direction != 2:
                            send_command('2')
                            motor_direction = 2
                            direction_updated = True
                            print("Rotating in counterclockwise direction.")
        
        if not direction_updated and motor_direction != 0:
            send_command('0')
            motor_direction = 0
            print("Stopping the motor.")
        
        time.sleep(0.1)  # Small delay to prevent overly rapid gamepad readings
        
except KeyboardInterrupt:
    if motor_direction != 0:
        send_command('0')  # Stop the motor in case of interruption
    ser.close()
