# Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino

## Description

This project involves a robotic arm capable of classifying objects using computer vision and sorting them accordingly. A Raspberry Pi 4 processes images captured from a camera, runs a trained machine learning model, and communicates with an Arduino to control the robotic arm’s movements.

<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/cb1debd6fb9ecf168e645abf7edaa6c06ce01490/Screenshot%202025-02-10%20085345.png" width="600">
</div>
<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/cb1debd6fb9ecf168e645abf7edaa6c06ce01490/Screenshot%202025-02-10%20085354.png" width="600">
</div>
<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/54c9f63219aa35e4910f3e36349a6c0ea116569f/z6263065847922_1003b7cafe9a5f4616814465f5f8eb61.jpg" width="600">
</div>





## System Overview
1. Computer Vision Subsystem: Identifies and classifies objects

```mermaid
graph TD;
  A(Start) --> B(Initialize Camera & Load Model)
  B --> C(Capture Image)
  C --> D(Preprocess Image)
  D --> E(Run Object Classification Model)
  E -->|Success| F(Identify Object & Determine Target)
  E -->|Failure| C
  F --> G(Convert to Movement Commands)
  G --> H{received_data = True?}
  H -->|No| M{Exit?}
  H -->|Yes| I(Send Command to Arduino)
  I --> J(Set received_data = False)
  J --> K{Wait for Acknowledgment?}
  K -->|No| M
  K -->|Yes| L(Set received_data = True)
  M -->|No| C
  M -->|Yes| N(End)
```
  
2. Robotic Control Subsystem: Moves the robotic arm based on classifications.

``` mermaid
graph TD;
  A(Start) --> B(Initialize Serial Communication & Motors)
  B --> C{Wait for Command?}
  C -->|No| C
  C -->|Yes| D(Decode Received Command)
  D --> E(Convert to Joint Angles)
  E --> F{Are Angles Valid?}
  F -->|No| G(Send Error & Return to Wait)
  F -->|Yes| H(Move Servos)
  H --> I{Movement Complete?}
  I -->|No| H
  I -->|Yes| J(Send Acknowledgment to Raspberry Pi)
  J --> K{Exit?}
  K -->|Yes| C
  K -->|No| L(End)
```

## Hardware Components

- Raspberry Pi 4: Runs object classification and sends movement commands.
- Arduino: Controls the robotic arm servos.
- Camera Module: Captures images for object recognition.
- Robotic Arm with Step Motors and Servo Motors: Moves objects based on classification.
- Drivers: 8825
- Power Supply: 24V 5A Provides power to components.
- Arm design
<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/203e2170ec728cdbb8de745ae4613ccea84c3a35/Screenshot%202025-02-10%20111221.png" width="600">
</div>

## Model Training Process

1. Data Collection: Capture images of objects using the camera module.
<div align="center"; display="flex">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/fbe2872c6891985c39d5b5bab8f1aaf8e47635ed/Screenshot%202025-02-10%20111242.png" height="300">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/fbe2872c6891985c39d5b5bab8f1aaf8e47635ed/Screenshot%202025-02-10%20111251.png" height="300">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/fbe2872c6891985c39d5b5bab8f1aaf8e47635ed/Screenshot%202025-02-10%20111303.png" height="300">
</div>
2. Preprocessing: Resize, normalize, and augment images.

<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/67e671b25cc8c84026e3b386a03e4a521a89dead/Screenshot%202025-02-10%20112149.png">
</div>

3. Model Selection: Use a convolutional neural network (CNN) for classification.
4. Training: Train the model using TensorFlow/Keras.

<div align="center"; display="flex">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/69e8f717c974c02693c91362188ec4e71f28926e/Screenshot%202025-02-10%20111443.png" height="300">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/69e8f717c974c02693c91362188ec4e71f28926e/Screenshot%202025-02-10%20111453.png" height="300">
</div>

5. Deployment: Convert the trained model to a format suitable for Raspberry Pi.

## Robotic Arm Kinematics

<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/5ade2921bac0592ef0fe79733c0ac4793d6177cc/Screenshot%202025-02-10%20125641.png" width="800">
</div>

The problem is to find 3 main angles: alpha, beta and gamma with fixed length a, b, c and h

<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/a2d12d7642d21b4da05c886abbbe4d683d9bc39f/Screenshot%202025-02-10%20125404.png" width="800">
</div>

The camera will be up side down like this, so we can base on the following picture to measure the coordinate of objects

<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/a2d12d7642d21b4da05c886abbbe4d683d9bc39f/Screenshot%202025-02-10%20125845.png" width="800">
</div>

## Operation

1. System Initialization: Start Raspberry Pi and Arduino.
2. Object Detection: Capture image and classify the object.

<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/3cfc5a853f931bac9c1f33cd3fd34bf08b185ccf/Screenshot%202025-02-10%20112744.png" width="800">
</div>
<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/3cfc5a853f931bac9c1f33cd3fd34bf08b185ccf/Screenshot%202025-02-10%20112810.png" width="800">
</div>
<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/3cfc5a853f931bac9c1f33cd3fd34bf08b185ccf/Screenshot%202025-02-10%20112831.png" width="800">
</div>

3. Robotic Arm Movement: Calculate and execute joint angles for object handling.
