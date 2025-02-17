import cv2
import numpy as np
import serial
import time
import struct

# arduino = serial.Serial(port='/dev/cu.usbserial-130', baudrate=9600, timeout=0.1)
time.sleep(2)

lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# PID constants
Kp = 0.8
Ki = 0.0
Kd = 0.0

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
    integral = max(min(integral, 10), -10)
    I = Ki * integral
    D = Kd * (error - previous_error)
    D = max(min(D, 5), -5)

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
                # arduino.write(struct.pack('ff', 0.0, 0.0))  # Stop robot on detection
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
                frame_center = frame.shape[1] // 2

                error = (center_x - frame_center) / 10  # Normalize error
                print(f"Error: {error}")

                # Compute the PID controller output
                pid_output = compute_pid(error)
                print(f"PID Output: {pid_output}")

                # Map PID output to left and right wheel velocities
                V_base = 0.2  # Base forward speed
                K = 0.5  # Steering adjustment gain

                # Differential steering
                V_L = V_base - K * pid_output
                V_R = V_base + K * pid_output
                print(f"Error: {error}")
                print(f"PID Output: {pid_output}")
                print(f"Sending V_L: {V_L}, V_R: {V_R}")

                # # Send velocities over serial to Arduino
                # arduino.write(struct.pack('ff', V_L, V_R))

                # # Optionally listen for Arduino's response
                # if arduino.in_waiting > 0:
                #     message = arduino.readline().decode().strip()
                #     print(f"Message from Arduino: {message}")

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, y + h // 2), 5, (0, 0, 255), -1)
            else:
                print("No green object detected.")
                # arduino.write(struct.pack('ff', 0.0, 0.0))  # Stop the robot when no target is detected

except KeyboardInterrupt:
    print("Stopped by user")

cap.release()
cv2.destroyAllWindows()
