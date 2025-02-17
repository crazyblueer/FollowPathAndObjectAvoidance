import cv2
import numpy as np
import serial
import time
import struct

arduino = serial.Serial(port='/dev/cu.usbserial-120', baudrate=115200, timeout=0.1)
time.sleep(2)
lower_green = np.array([45, 100, 100])
upper_green = np.array([75, 255, 255])

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# PID constants
Kp = 0.000023
Ki = 0.000000
Kd = 0.00000

previous_error = 0
integral_error = 0

start_time = time.time()
frame_last_processed_time = time.time()

class State:
    START = "START"
    DETECT_GREEN = "DETECT_GREEN"
    TRACK = "TRACK"
    FOLLOWPATH = "FOLLOW_PATH"

current_state = State.START


def compute_pid(error):
        global integral_error
        global previous_error 
        P = error 
        integral_error += error 
        D = error - previous_error
        pid_output = (Kp * P) + (Ki * integral_error) + (Kd * D)
        previous_error = error 

        return pid_output


def transition(new_state):
    global current_state, start_time
    print(f"Transitioning to {new_state}")
    current_state = new_state
    start_time = time.time()

def listen_for_arduino():
        message = arduino.readline().decode('utf-8').strip()
        if message == "done":
            
            arduino.write("0.0,0.0\n".encode('utf-8')) #stop
            print("Wrapped done, stop robot.")
            transition(State.FOLLOWPATH)    


try:
    while True:
        current_time = time.time()
        if current_time - frame_last_processed_time >= 0.0:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

            if current_state == State.START:
                print("State: START")
                arduino.write("0.0,0.15\n".encode('utf-8')) #spinning around until seeing the green bottle           
                if cv2.countNonZero(green_mask) > 0:
                    arduino.write("0.0,0.0\n".encode('utf-8'))
                    print("Green detected, stopping the robot.")
                    transition(State.DETECT_GREEN)

            elif current_state == State.DETECT_GREEN: 
                print("State: DETECT_GREEN")
                if time.time() - start_time > 1:
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
                    frame_center = frame.shape[1] // 2

                    error = (frame_center - center_x)
                    print(f"Error: {error}")

                    angular_velocity = compute_pid(error)
                    angular_velocity = np.clip(angular_velocity, -1.0, 1.0)
                    linear_velocity = 0.07
                    print(f"Angular velocity: {angular_velocity}")
                    # Differential steering
                    V_L = linear_velocity -  (angular_velocity*0.189)
                    V_R = linear_velocity + (angular_velocity*0.189)
                    V_L = round(V_L, 3)
                    V_R = round(V_R, 3)
                    print(f"Error: {error}")
                    print(f"Sending V_L: {V_L}, V_R: {V_R}")

                    # # Send velocities over serial to Arduino
                    arduino.write(f"{V_L},{V_R}\n".encode('utf-8'))
                    # # Optionally listen for Arduino's response
                    # if arduino.in_waiting > 0:
                    #     message = arduino.readline().decode().strip()
                    #     print(f"Message from Arduino: {message}")

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (center_x, y + h // 2), 5, (0, 0, 255), -1)
                else:
                    print("No green object detected.")
                    arduino.write("0.0,0.0\n".encode('utf-8')) 
                    transition(State.START)
                
                listen_for_arduino()
            frame_last_processed_time = current_time

except KeyboardInterrupt:
    print("Stopped by user")

cap.release()
cv2.destroyAllWindows()