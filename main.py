import cv2
import numpy as np
import serial
import time
import struct


arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=0.1)
time.sleep(2)

lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

Kp = 0.8
Ki = 0.02
Kd = 0.5

previous_error = 0
integral = 0

start_time = time.time()

class State:
    START = "START"
    DETECT_GREEN = "DETECT_GREEN"
    TRACK = "TRACK"
    FOLLOWPATH = "FOLLOW_PATH"

current_state = State.START

def compute_pid(error):
    global previous_error, integral

    P = Kp * error
    integral += error
    I = Ki * integral
    D = Kd * (error - previous_error)

    previous_error = error
    return P + I + D

def transition(new_state):
    global current_state, start_time
    print(f"Transitioning to {new_state}")
    current_state = new_state
    start_time = time.time()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

        if current_state == State.START:
            
            print("State: START")
            if cv2.countNonZero(green_mask) > 0:
                arduino.write(struct.pack('f', -1.0))
                print("Green detected, stopping the robot.")
                transition(State.DETECT_GREEN)

        elif current_state == State.DETECT_GREEN:
            
            print("State: DETECT_GREEN")
            if time.time() - start_time > 2:
                transition(State.TRACK)

        elif current_state == State.TRACK:
            
            print("State: TRACK")
            kernel = np.ones((15, 15), np.uint8)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)

                center_x = x + w // 2
                center_y = y + h // 2

                frame_center = frame.shape[1] // 2
                error = (center_x - frame_center) / 10
                print(f"Error: {error}")

                pid_output = compute_pid(error)
                print(f"PID Output: {pid_output}")

                arduino.write(struct.pack('f', pid_output))
                received_pid = arduino.readline().decode().strip()
                print(received_pid)
                # Listen for Arduino's state change message
                if arduino.in_waiting > 0:
                    message = arduino.readline().decode().strip()
                    print(f"Message from Arduino: {message}")
                    if message == "Grab Done":
                        print("Arduino requested state change, transitioning to START")
                        transition(State.FOLLOWPATH)
                        print("STATE: FOLLOW PATH")

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            else:
                print("No green object detected, switching to FOLLOW PATH state")
                transition(State.START)

        elif current_state == State.FOLLOWPATH:
            print("Follow A* Path")
            arduino.write(struct.pack('f', -1.0))
            time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user")

cap.release()
cv2.destroyAllWindows()
