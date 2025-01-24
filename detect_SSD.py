import cv2
import numpy as np
from tensorflow.lite.python.interpreter import Interpreter
import time
import math
# import serial
# import threading
# import RPi.GPIO as GPIO

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)
time.sleep(2)

lblpath = "/home/ngocdz/Desktop/SSD TFlite/labelmap.txt"
modelpath = "/home/ngocdz/Desktop/SSD TFlite/detect-ver-24.tflite"
min_conf = 0.5

with open(lblpath, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

interpreter = Interpreter(model_path=modelpath)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

cap = cv2.VideoCapture(0)
cap.set(3, 720)
cap.set(4, 480)

arduino_data = [0, 0, 0, 0, 0, 0]
frame_count = 0
start_time = time.time()
full_data = False
received_data = True

def send_to_arduino(data):
    print("Bắt đầu gửi data tới Arduino")
    str_data = ','.join(map(str, data))
    ser.write(str_data.encode('utf-8'))
    print("Đã gửi data xong")

def process_frame(frame):
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (width, height))
    imH, imW, _ = frame.shape

    input_mean = 127.5
    input_std = 127.5

    if input_details[0]['dtype'] == np.float32:
        input_data = (np.float32(image_resized) - input_mean) / input_std
    else:
        input_data = image_resized

    interpreter.set_tensor(input_details[0]['index'], np.expand_dims(input_data, axis=0))
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[1]['index'])[0]
    classes = interpreter.get_tensor(output_details[3]['index'])[0]
    scores = interpreter.get_tensor(output_details[0]['index'])[0]

    for i in range(len(scores)):
        if scores[i] > min_conf and scores[i] <= 1:
            ymin, xmin, ymax, xmax = boxes[i]
            
            xmin = int(xmin * imW)
            xmax = int(xmax * imW)
            ymin = int(ymin * imH)
            ymax = int(ymax * imH)
            
#             print(ymin, xmin, ymax, xmax)

            class_id = int(classes[i]) 
#             print(class_id)
            class_name = labels[class_id]
#             print(class_name)
            score = scores[i]
            
#             arduino_data[0:4] = ymin, xmin, ymax, xmax

            if boxes[i].all() and class_name and not full_data:
                arduino_data[0:4] = ymin, xmin, ymax, xmax
                arduino_data[4] = class_name

            cv2.putText(frame, f'{class_name} {score:.2f}', (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 255, 0), 2)
#             cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

def detect_line(frame):
    # Xác định đường thẳng
    
    max_len_line = 0
    max_len_line_index = None

    # Convert to grayscale image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Use Canny to find edges
    edges = cv2.Canny(gray, 50, 200, None, 3)

    linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, None, 60, 60)

    if linesP is not None:
        for i, lineP in enumerate(linesP):
            # Tính chiều dài đường thẳng
            len_line = math.sqrt((lineP[0][2] - lineP[0][0])**2 + (lineP[0][3] - lineP[0][1])**2)

            if len_line > max_len_line:
                max_len_line = len_line
                max_len_line_index = i

            # Vẽ đoạn đường lên ảnh
            cv2.line(frame, (lineP[0][0], lineP[0][1]), (lineP[0][2], lineP[0][3]), (0, 0, 255), 3, cv2.LINE_AA)

        if max_len_line != 0 and max_len_line_index is not None:
            degree_theta = math.degrees(math.atan2(linesP[max_len_line_index][0][3] - linesP[max_len_line_index][0][1], linesP[max_len_line_index][0][2] - linesP[max_len_line_index][0][0]))
            degree_theta = - degree_theta
            if degree_theta < 0:
                degree_theta = 180 + degree_theta
            if degree_theta <= 180 and degree_theta >= 178:
                degree_theta = 0
            if degree_theta <= 2 and degree_theta >= 0:
                degree_theta = 0
            if not full_data:
                arduino_data[5] = degree_theta
                
def received_sign():
    sign = ser.readline().strip().decode('utf-8')
    return sign

def receive_from_arduino():
    global received_data
    global arduino_data
    global full_data
    while True:
        sign = received_sign()
        if sign != "":
            received_data = True
            full_data = False

receive_thread = threading.Thread(target=receive_from_arduino)
receive_thread.daemon = True

receive_thread.start()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    process_frame(frame)

    detect_line(frame)

    frame_count += 1
    elapsed_time = time.time() - start_time
    fps = frame_count / elapsed_time
    cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    if all(element != 0 for element in arduino_data):
        full_data = True
        if received_data: 
            print("arduino_data: ", arduino_data)
            send_to_arduino(arduino_data)
            received_data = False
            arduino_data = [0, 0, 0, 0, 0, 0]

    print("Arduino data: ", arduino_data)
 
    cv2.imshow('Object Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

