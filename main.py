from picamera2 import Picamera2, Preview
import time
import cv2
from libcamera import Transform
import numpy as np
from ultralytics import YOLO
import serial
from datetime import datetime
import threading
import os
import sys
    
# Set up the serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust port as necessary

# Set up the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
	main={"size": (720,480), "format": "RGB888"},
	transform= Transform(hflip=True, vflip=True)
))

def send_msg(msg):
    try:
    # Send a message
        ser.write(msg.encode('ascii'))
        time.sleep(0.3)  # Wait for the connection to establish
    except Exception as e:
        print("Exception:", e) 

def wait_msg():
    print('Thread start - Waiting arduino message...')
    ser.flushInput()
    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode('ascii')
            time.sleep(1)
            print('Resetting...')
            cv2.destroyAllWindows()
            picam2.stop()
            picam2.close()
            ser.close()  # Close the connection
            os.system('python3 main.py')
            sys.exit()

def main():
    
    # Load a self-trained YOLO plant detection PyTorch model (Using Pre-trained YOLO11l.pt for base)
    model = YOLO("best.pt")

    # Open default camera
    picam2.start()
    
    # Reset variable
    plant_found = 0
        
    # Record State (0: Camera, 1: Car)
    moving = 0
    
    print('Waiting arduino message to start')
    ser.flushInput()
    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode('ascii')
            if message:
                send_msg("w")
                break

    while True:
        
        # Generate a timestamp for the filename
        datestamp = datetime.now().strftime("%Y-%m-%d")
        timestamp = datetime.now().strftime("%H%M")
        filename = f"Plant/{datestamp}_{timestamp}.jpg"
    
        frame = picam2.capture_array()

        # Run YOLO inference on the frame
        results = model(frame)

        # Draw results on the frame
        annotated_frame = results[0].plot()

        # Show the frame
        cv2.imshow("Smart Plant Rover Car", annotated_frame)
        
        # Detect Status
        detection = 0
        
        # Create threading for waiting arduino message
        t1 = threading.Thread(target=wait_msg)
        
        print(plant_found)
        print(moving)
        
        # Extract the detected class name set
        clist = results[0].boxes.cls
        cls = set()
        for cno in clist:
            detection = 1
            cls.add(model.names[int(cno)])
            
        # Extract bounding box dimensions
        x, y, w, h = 0,0,0,0
        boxes = results[0].boxes.xywh.cpu()
        for box in boxes:
            x, y, w, h = box
        
        # There are 4 types of labels: Vase, Flower, Lead and Potted Plant
        if "Potted Plant" in cls and "Vase" in cls and not plant_found:
            if not plant_found:
                t1.start()
                print(">>> Plant Found !!!")
                # Capture and save the image with the timestamp filename
                picam2.capture_file(filename)
                plant_found = 1
            send_msg("f") # Move Forward
            #time.sleep(1)
        
        if "Vase" in cls:
            if ("Leaf" in cls or "Flower" in cls):
                if not moving:
                    send_msg("w") 
                    moving += 1
                    send_msg("f") # Move Forward
                else:
                    if "Leaf" in cls or "Flower" in cls:
                        if not plant_found:
                            t1.start()
                            print(">>> Plant Found !!!")
                            # Capture and save the image with the timestamp filename
                            picam2.capture_file(filename)
                            plant_found = 1
                    if x < 210:
                        send_msg("l")
                    elif x > 500:
                        send_msg("f") # Move Forward
                    #time.sleep(1)
            else:
                detection = 0
        
        if "Leaf" in cls or "Flower" in cls:
            if moving:
                send_msg("s")
                moving -= 1
            else:
                send_msg("f")
                
        if detection == 0 and plant_found == 0:
            send_msg("i") # Rotate Offset
            send_msg("a")
            time.sleep(0.5)
            send_msg("d")
            time.sleep(0.5) 
        elif detection == 0 and plant_found == 1:
            if w > 350:
                send_msg("o")
                send_msg("f")
            else:
                send_msg("b")
                send_msg("i")
        
        print(x)
        
        # Exit on 'q' or ESC key
        key = cv2.waitKey(1)
        if key == 27 or key == ord('q'):
            break

    cv2.destroyAllWindows()
    picam2.stop()
    picam2.close()
    ser.close()  # Close the connection

if __name__ == "__main__":
    main()

