#include <stdio.h>
#include <math.h> 
#include <Wire.h>
#include <Servo.h>
#include <AccelStepper.h>

#define bigServoPin 2
#define rightSmallServoPin 3
#define leftSmallServoPin 4

#define enPin42 5

#define dirPin42_1 6
#define stepPin42_1 7
#define SPR42_1 400

#define dirPin42_2 8
#define stepPin42_2 9
#define SPR42_2 400

#define dirPin42_3 10
#define stepPin42_3 11
#define SPR42_3 400

#define dirPin42_4 12
#define stepPin42_4 13
#define SPR42_4 400

#define dirPin57 14
#define stepPin57 15
#define SPR57 800

Servo bigServo;
Servo rightSmallServo;
Servo leftSmallServo;

AccelStepper stepper42_1(1, stepPin42_1, dirPin42_1);
AccelStepper stepper42_2(1, stepPin42_2, dirPin42_2);
AccelStepper stepper42_3(1, stepPin42_3, dirPin42_3);
AccelStepper stepper42_4(1, stepPin42_4, dirPin42_4);
AccelStepper stepper57(1, stepPin57, dirPin57);

float a = 153.2;
float b = 136.08;
float c = 41.17;
int h = 250;

float alpha = 0.0;
float beta = 0.0;
float gamma = 0.0;

bool ready = false;
bool homeReady = true;

#define STORAGE_SIZE 30
#define DATA_SIZE 6

uint8_t current_data[DATA_SIZE] = {0};  
bool received_data = false;  


// --------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  Wire.begin(1);
  Wire.onReceive(receiveEvent); 

  bigServo.attach(bigServoPin);
  bigServo.write(90);
  rightSmallServo.attach(rightSmallServoPin);
  leftSmallServo.attach(leftSmallServoPin);

  pinMode(dirPin42_1, OUTPUT);
  pinMode(stepPin42_1, OUTPUT);
  pinMode(dirPin42_2, OUTPUT);
  pinMode(stepPin42_2, OUTPUT);
  pinMode(dirPin42_3, OUTPUT);
  pinMode(stepPin42_3, OUTPUT);
  pinMode(dirPin42_4, OUTPUT);
  pinMode(stepPin42_4, OUTPUT);
  pinMode(dirPin57, OUTPUT);
  pinMode(stepPin57, OUTPUT);

  pinMode(enPin42, OUTPUT);


  digitalWrite(enPin42, LOW); // Kích hoạt động cơ 2

  stepper42_1.setMaxSpeed(20000); 
  stepper42_1.setAcceleration(4000);
  // stepper42_1.setSpeed(300); // Tốc độ ban đầu cho động cơ 1

  stepper42_2.setMaxSpeed(20000); 
  stepper42_2.setAcceleration(4000);
  // stepper42_2.setSpeed(500); // Tốc độ ban đầu cho động cơ 2

  stepper42_3.setMaxSpeed(20000); 
  stepper42_3.setAcceleration(4000);
  // stepper42_3.setSpeed(700); // Tốc độ ban đầu cho động cơ 3

  stepper42_4.setMaxSpeed(20000); 
  stepper42_4.setAcceleration(4000);
  // stepper42_4.setSpeed(900); // Tốc độ ban đầu cho động cơ 4

  stepper57.setMaxSpeed(30000); 
  stepper57.setAcceleration(8000);
}

void loop() {
  if(!homeReady) {
    home(-2482, -2868, 0, 280, 0, 90, 90, 90);
  }
  if (received_data) {
    unitArm();
    received_data = false;
  }
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------

// -------------------- //

void actPhase(
  int deg42_1_grip,
  int deg42_2_grip, 
  int deg42_3_grip, 
  int deg42_4_grip,
  int degServo_obj,
  int deg57_obj,
  int deg57_class
) { // thực hiện nhiệm vụ gắp và phân loại

  // bước 1: di chuyển cánh tay để gắp vặt thể
  spinObjDeg(90 - degServo_obj + deg57_obj);
  rightSmallServo.write(0); 
  leftSmallServo.write(180); 
  
  stepper42_1.moveTo(deg42_1_grip); // come to grip object
  stepper42_2.moveTo(deg42_2_grip); 
  stepper42_3.moveTo(deg42_3_grip); 
  stepper42_4.moveTo(deg42_4_grip); 
  stepper57.moveTo(deg57_obj);
  while (stepper42_1.distanceToGo() || stepper42_2.distanceToGo() || stepper42_3.distanceToGo() || stepper42_4.distanceToGo() || stepper57.distanceToGo()) {
    stepper42_1.run(); // Chạy động cơ 1
    stepper42_2.run(); // Chạy động cơ 2
    stepper42_3.run(); // Chạy động cơ 3
    stepper42_4.run(); // Chạy động cơ 4
    stepper57.run(); // Chạy động cơ 57
  }
  stepper42_1.setCurrentPosition(0);
  stepper42_2.setCurrentPosition(0);
  stepper42_3.setCurrentPosition(0);
  stepper42_4.setCurrentPosition(0);
  stepper57.setCurrentPosition(0);

  delay(500);

  // bước 2: gắp vật thể
  grip();

  delay(1000);

  // bước 3: chuyển cánh tay đến vùng phân loại
  stepper42_1.moveTo(deg42_1_grip - 1860); 
  stepper42_2.moveTo(-1086 - deg42_2_grip); 
  stepper42_3.moveTo(0); 
  stepper42_4.moveTo(-90 - deg42_4_grip); 
  stepper57.moveTo(deg57_class - deg57_obj); 
  while (stepper42_1.distanceToGo() || stepper42_2.distanceToGo() || stepper42_3.distanceToGo() || stepper42_4.distanceToGo() || stepper57.distanceToGo()) {
    stepper42_1.run(); // Chạy động cơ 1
    stepper42_2.run(); // Chạy động cơ 2
    stepper42_3.run(); // Chạy động cơ 3
    stepper42_4.run(); // Chạy động cơ 4
    stepper57.run(); // Chạy động cơ 57
  }
  stepper42_1.setCurrentPosition(0);
  stepper42_2.setCurrentPosition(0);
  stepper42_3.setCurrentPosition(0);
  stepper42_4.setCurrentPosition(0);
  stepper57.setCurrentPosition(0);

  delay(1000);

  // bước 4: thả vật thể
  drop();

  delay(1000);

  // bước 5: chuyển hình dạng cánh tay về dạng mặc định 

  home(1860, 1086, 0, 90, -deg57_class, 90, 90, 90);
}

void home(int degHome42_1, int degHome42_2, int degHome42_3, int degHome42_4, int degHome57, int degHomeBigSer, int degHomeRightSmallSer, int degHomeLeftSmallSer) {
  spinObjDeg(degHomeBigSer);
  rightSmallServo.write(degHomeRightSmallSer); 
  leftSmallServo.write(degHomeLeftSmallSer); 

  stepper42_1.moveTo(degHome42_1); // turn back the arm to home position
  stepper42_2.moveTo(degHome42_2); 
  stepper42_3.moveTo(degHome42_3); 
  stepper42_4.moveTo(degHome42_4); 
  stepper57.moveTo(degHome57); 
  while (stepper42_1.distanceToGo() || stepper42_2.distanceToGo() || stepper42_3.distanceToGo() || stepper42_4.distanceToGo() || stepper57.distanceToGo()) {
    stepper42_1.run(); // Chạy động cơ 1
    stepper42_2.run(); // Chạy động cơ 2
    stepper42_3.run(); // Chạy động cơ 3
    stepper42_4.run(); // Chạy động cơ 4
    stepper57.run(); // Chạy động cơ 57
  }
  stepper42_1.setCurrentPosition(0);
  stepper42_2.setCurrentPosition(0);
  stepper42_3.setCurrentPosition(0);
  stepper42_4.setCurrentPosition(0);
  stepper57.setCurrentPosition(0);

  homeReady = true;
}

// -------------------- //

void spinObjDeg(int deg_obj) {
  bigServo.write(deg_obj);
}

// -------------------- //

void grip() { // gắp vật thể
  rightSmallServo.write(90); 
  leftSmallServo.write(90); 
  Serial.write("Done");
}

// -------------------- //

void drop() { // thả vật thể
  rightSmallServo.write(0); 
  leftSmallServo.write(90); 
}

// -------------------- //

void unitArm() {
    if (ready){
      stepper42_1_deg(a, b, c, cal_d(current_data, h, stepper57_deg(current_data, h)), cal_e(c, cal_d(current_data, h, stepper57_deg(current_data, h))));
      stepper42_2_deg(a, b, c, cal_d(current_data, h, stepper57_deg(current_data, h)), cal_e(c, cal_d(current_data, h, stepper57_deg(current_data, h))));
      stepper42_4_deg(alpha, beta);

      actPhase(
        convert_deg_to_phase(42, 27, alpha),
        convert_deg_to_phase(42, 27, beta),
        0,
        convert_deg_to_phase(42, 3, gamma),
        (int)(current_data[5]),
        convert_deg_to_phase(57, 1, stepper57_deg(current_data, h)),
        convert_deg_to_phase(57, 1, cal_deg_57_class(current_data))
      );
    }
}

// -------------------- //

void receiveEvent(int numBytes) {
  while (Serial.available() < DATA_SIZE) {
  }

  for (int i = 0; i < DATA_SIZE; i++) {
    current_data[i] = Serial.read();
    received_data = true;
  }
}

// -------------------- //

int convert_from_pixel_to_mm(int pixel) {
  return (5 * pixel) / 24;  
}

float stepper57_deg(uint8_t current_data[6], int h) {
  int ymin = (int)current_data[0];
  int xmin = current_data[1];
  int ymax = current_data[2];
  int xmax = current_data[3];

  float deg_57 = round(atan(((xmax - xmin) / 2.0) / (h + ymin + (ymax - ymin) / 2.0)) * (180.0 / M_PI) * 1000.0) / 1000.0;

  return deg_57; 
}

int cal_d(uint8_t current_data[6], int h, float deg_57) { 
  int ymin = current_data[0];
  int xmin = current_data[1];
  int ymax = current_data[2];
  int xmax = current_data[3];

  float deg_57_radians = deg_57 * (M_PI / 180.0);

  float sin_deg_57 = sin(deg_57_radians);

  if (sin_deg_57 == 0) {
      printf("Error: sin(alpha) is zero, cannot divide by zero.\n");
      return -1;
  }

  int d = (convert_from_pixel_to_mm(xmax) - convert_from_pixel_to_mm(xmin)) / 2 / sin_deg_57;

  return d; 
}

float cal_e(float c, int d) {
  float e = round(sqrt(pow(c, 2) + pow(d, 2)) * 1000.0) / 1000.0;
  return e;
}

void stepper42_1_deg(float a, float b, float c, int d, float e) {
  float alpha_1 = round(acos((pow(a, 2) + pow(e, 2) - pow(b, 2)) / (2 * a * e)) * 1000.0) / 1000.0;
  float alpha_2 = round(atan(c / d) * 1000.0) / 1000.0;

  alpha = (alpha_1 + alpha_2) - 90.0;
}
void stepper42_2_deg(float a, float b, float c, int d, float e) {
  beta = 113.61 - (round(acos((pow(a, 2) + pow(b, 2) - pow(e, 2)) / (2 * a * b)) * 1000.0) / 1000.0);
}
// float stepper42_3_deg(float a, float b, float c, int d, float e) {
// }
void stepper42_4_deg(float alpha, float beta) {
  gamma = 66.39 - (round(360.0 - 90.0 - alpha - beta) * 1000.0) / 1000.0;
}


float cal_deg_57_class(uint8_t current_data[6]) {
  uint8_t id = current_data[4];
  if(id == 0) {
    return -60.0;
  } if(id == 1) {
    return 60.0;
  } if(id == 2) {
    return 120.0;
  }
}


int convert_deg_to_phase(int type, int ratio, float deg) {
  if (type == 42) {
    return (int) ((deg * 400.0 * ratio) / 360.0);
  } 
  if (type == 57) {
    return (int) ((deg * 1600.0 * ratio) / 360.0);
  }
  return 0; 
}
