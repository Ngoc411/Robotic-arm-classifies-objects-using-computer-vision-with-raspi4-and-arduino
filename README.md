# Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino

## Description

This project involves a robotic arm capable of classifying objects using computer vision and sorting them accordingly. A Raspberry Pi 4 processes images captured from a camera, runs a trained machine learning model, and communicates with an Arduino to control the robotic arm’s movements.

<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/cb1debd6fb9ecf168e645abf7edaa6c06ce01490/Screenshot%202025-02-10%20085345.png" width="500">
</div>
<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/cb1debd6fb9ecf168e645abf7edaa6c06ce01490/Screenshot%202025-02-10%20085354.png" width="500">
</div>
<div align="center">
  <img src="https://github.com/Ngoc411/Robotic-arm-classifies-objects-using-computer-vision-with-raspi4-and-arduino/blob/54c9f63219aa35e4910f3e36349a6c0ea116569f/z6263065847922_1003b7cafe9a5f4616814465f5f8eb61.jpg" width="500">
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
  J --> K{Next Command?}
  K -->|Yes| C
  K -->|No| L(End)
```

## Hardware Components

- Raspberry Pi 4: Runs object classification and sends movement commands.
- Arduino: Controls the robotic arm servos.
- Camera Module: Captures images for object recognition.
- Robotic Arm with Servo Motors: Moves objects based on classification.
- Power Supply: Provides power to components.

## Software Architecture

1. Object Detection & Classification: OpenCV and a trained model classify objects.
2. Communication Protocol: Serial communication between Raspberry Pi and Arduino.
3. Motor Control: Arduino processes received commands and moves servos.

## Model Training Process

1. Data Collection: Capture images of objects using the camera module.
2. Preprocessing: Resize, normalize, and augment images.
3. Model Selection: Use a convolutional neural network (CNN) for classification.
4. Training: Train the model using TensorFlow/Keras.
5. Deployment: Convert the trained model to a format suitable for Raspberry Pi.

## Robotic Arm Kinematics

## Operation

1. System Initialization: Start Raspberry Pi and Arduino.
2. Object Detection: Capture image and classify the object.
3. Robotic Arm Movement: Calculate and execute joint angles for object handling.
