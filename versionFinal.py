import RPi.GPIO as GPIO
import time
import serial
from inputs import get_gamepad

# Configuración de los pines GPIO
GPIO.setmode(GPIO.BCM)
GPIO_PIN_20 = 20  # Número del GPIO para el motor paso a paso
GPIO_PIN_16 = 16  # Número del otro GPIO para el motor paso a paso
GPIO_PIN_23 = 23  # Número del GPIO para el joystick derecho
GPIO_PIN_24 = 24  # Número del otro GPIO para el joystick derecho
ENDSTOP_PIN = 5   # Número del GPIO para el final de carrera

# Configuración de los pines GPIO como salida
GPIO.setup(GPIO_PIN_20, GPIO.OUT)
GPIO.setup(GPIO_PIN_16, GPIO.OUT)
GPIO.setup(GPIO_PIN_23, GPIO.OUT)
GPIO.setup(GPIO_PIN_24, GPIO.OUT)

# Configuración del pin del final de carrera como entrada
GPIO.setup(ENDSTOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Rango de movimiento de los servomotores
servo_min_angle = 0
servo_max_angle = 180

# Ajusta el puerto y la velocidad del puerto serie para el motor DC
ser = serial.Serial('/dev/ttyS0', 9600)

# Función para enviar comandos al motor DC
def send_motor_command(direction):
    ser.write(str(direction).encode())
    time.sleep(0.1)  # Pequeño retraso para permitir que el motor responda

# Función para mapear valores
def map_value(value, from_low, from_high, to_low, to_high):
    return int((value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low)

try:
    angle_servo1 = 90  # Ángulo inicial del primer servomotor
    angle_servo2 = 90  # Ángulo inicial del segundo servomotor
    motor_direction = 0  # Estado del motor DC

    while True:
        events = get_gamepad()
        
        for event in events:
            if event.ev_type == 'Absolute':
                if event.code == 'ABS_Y':
                    # Joystick izquierdo controla el primer servomotor (adelante y atrás)
                    angle_servo1 = map_value(event.state, -32768, 32767, servo_max_angle, servo_min_angle)
                    print(f"Ángulo del Servo1: {angle_servo1}")

                    if angle_servo1 > 135:
                        GPIO.output(GPIO_PIN_20, GPIO.HIGH)
                    elif angle_servo1 < 60:
                        GPIO.output(GPIO_PIN_16, GPIO.HIGH)
                    else:
                        GPIO.output(GPIO_PIN_20, GPIO.LOW)
                        GPIO.output(GPIO_PIN_16, GPIO.LOW)
                
                elif event.code == 'ABS_RX':
                    # Joystick izquierdo controla el segundo servomotor (izquierda y derecha)
                    angle_servo2 = map_value(event.state, -32768, 32767, servo_min_angle, servo_max_angle)
                    print(f"Ángulo del Servo2: {angle_servo2}")

                    if angle_servo2 > 135:
                        GPIO.output(GPIO_PIN_23, GPIO.HIGH)
                    elif angle_servo2 < 60:
                        GPIO.output(GPIO_PIN_24, GPIO.HIGH)
                    else:
                        GPIO.output(GPIO_PIN_23, GPIO.LOW)
                        GPIO.output(GPIO_PIN_24, GPIO.LOW)

            elif event.ev_type == 'Key':
                if event.code == 'BTN_TR':
                    if event.state == 1 and GPIO.input(ENDSTOP_PIN) == GPIO.HIGH:
                        new_direction = 1  # Sentido de las agujas del reloj
                        print("Girando en sentido de las agujas del reloj.")
                    else:
                        new_direction = 0
                elif event.code == 'BTN_TL':
                    if event.state == 1:
                        new_direction = 2  # Sentido contrario a las agujas del reloj
                        print("Girando en sentido contrario a las agujas del reloj.")
                    else:
                        new_direction = 0

        # Solo envía el comando al Arduino si hay un cambio en la dirección
        if new_direction != motor_direction:
            send_motor_command(new_direction)
            motor_direction = new_direction

        time.sleep(0.1)

except KeyboardInterrupt:
    if motor_direction != 0:
        send_motor_command(0)
    ser.close()
finally:
    GPIO.cleanup()
