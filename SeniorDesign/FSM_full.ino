#include <AccelStepper.h>

//---------------------
// 电机驱动部分
//---------------------

// 电机引脚定义
const int s1dirPin    = 46;
const int s1stepPin   = 47;
const int edirPin     = 48;
const int estepPin    = 49;
const int basedirPin  = 52; 
const int basestepPin = 53; 
const int s3dirPin    = 50;
const int s3stepPin   = 51;

// 创建步进电机对象
AccelStepper s1Stepper   (AccelStepper::DRIVER, s1stepPin, s1dirPin);
AccelStepper elbowStepper(AccelStepper::DRIVER, estepPin, edirPin);
AccelStepper baseStepper (AccelStepper::DRIVER, basestepPin, basedirPin);
AccelStepper s3Stepper   (AccelStepper::DRIVER, s3stepPin, s3dirPin);

// 将角度转换为步数（步进电机的1.8°/步）
float rot(float deg) {
  return deg / 1.8;
}

// 电机动作函数（仅保留 blockFront、blockDown、blockSide5，jab()已删除）
void blockFront() {
  Serial.println("blocking front");

  s1Stepper.moveTo(rot(-45));    // 45°
  s3Stepper.moveTo(rot(90));       // 90°
  elbowStepper.moveTo(rot(15));    // 15°
  baseStepper.moveTo(0);           // 0°

  while ( s1Stepper.distanceToGo() != 0 ||
          s3Stepper.distanceToGo() != 0 ||
          elbowStepper.distanceToGo() != 0 ||
          baseStepper.distanceToGo() != 0 )
  {
    s1Stepper.run();
    s3Stepper.run();
    elbowStepper.run();
    baseStepper.run();
  }
  delay(500);
}

void blockDown() {
  Serial.println("blocking below");

  s1Stepper.moveTo(rot(15));    // 15°
  s3Stepper.moveTo(rot(90));    // 90°
  elbowStepper.moveTo(rot(15)); // 15°
  baseStepper.moveTo(rot(-30));

  while ( s1Stepper.distanceToGo() != 0 ||
          s3Stepper.distanceToGo() != 0 ||
          elbowStepper.distanceToGo() != 0 ||
          baseStepper.distanceToGo() != 0 )
  {
    s1Stepper.run();
    s3Stepper.run();
    elbowStepper.run();
    baseStepper.run();
  }
  delay(500);
}

void blockSide5() {
  Serial.println("blocking side");

  s1Stepper.moveTo(rot(-45));    // 45°
  s3Stepper.moveTo(rot(15));      // 15°
  elbowStepper.moveTo(rot(-45));  // 45°
  baseStepper.moveTo(rot(30));    // 30°

  while ( s1Stepper.distanceToGo() != 0 ||
          s3Stepper.distanceToGo() != 0 ||
          elbowStepper.distanceToGo() != 0 ||
          baseStepper.distanceToGo() != 0 )
  {
    s1Stepper.run();
    s3Stepper.run();
    elbowStepper.run();
    baseStepper.run();
  }
  delay(500);
  
}
bool active() {
    if (s1Stepper.isRunning() || s3Stepper.isRunning() || baseStepper.isRunning() || elbowStepper.isRunning()) {
        return true;
    } else {
        return false;
    }
}

//---------------------
// 状态机部分（RobotArm_SM 与 UltrasoundSM）
//---------------------

//FLAGS 与 TIMERs
int player_ready = 1; // 假设玩家已经准备好
bool moving  = true;
int contact = 0;

unsigned long timer1 = 0; // Position0 状态计时（等待3秒）
unsigned long timer2 = 0; // Reading_input 状态计时（5秒超时）
unsigned long timer3 = 0; // Show_results 状态计时（5秒显示结果）
unsigned long timer4 = 0; // Waiting_4_Player 状态计时（10秒超时）

const long interval1 = 3000;  // 3秒
const long interval2 = 5000;  // 5秒
const long interval3 = 10000; // 10秒

int ledPin1 = 2; // 指示机器人臂动作结束，玩家出拳
int ledPin2 = 3; // 指示程序开始

// 按钮引脚定义
//  int buttonPin1 = 4;
//  int buttonPin2 = 5;
//  int buttonPin3 = 6;
//  int buttonPin4 = 7;
// // // 原代码中 buttonPin5-7 分别为 8,9,10，但 9 和 10 与 ultrasound 冲突，故修改为：
//  int buttonPin5 = 8;   // Routine option 1
//  int buttonPin6 = 11;  // Routine option 2（原9）
//  int buttonPin7 = 12;  // Routine option 3（原10）

// // 按钮状态变量
//  int buttonState1 = 0;
//  int buttonState2 = 0;
//  int buttonState3 = 0;
//  int buttonState4 = 0;
//  int buttonState5 = 0;
//  int buttonState6 = 0;
//  int buttonState7 = 0;

const int jsX = A0; 
const int jsY = A1; 

int reset = 13; 

// 定义 routines 数组：行数为不同例程，列数为例程中每一步动作的索引
int routines[3][3] = {
  {0, 1, 2},
  {0, 1, 2},
  {2, 0, 1}
};

int idx = 0;         // 当前例程的步骤索引（0~2）
int end_idx = 3;     // 未使用
int routine = 1;     // 当前选择的例程（默认1）
int movement_idx = 0; // 根据例程矩阵调用相应动作
int x1 = 0;
int x2 = 0;

// 将动作函数数组指针指向电机动作函数
void (*movements[])() = { blockFront, blockDown, blockSide5 };

//---------------------
// Ultrasound（超声波传感器）状态机
//---------------------

enum SensorState { TRIGGER_SENSOR, WAIT_FOR_ECHO, READ_SENSOR, PROCESS_DATA };
SensorState currentSensorState = TRIGGER_SENSOR;

const int trigPin = 9;
const int echoPin = 10;
unsigned long echoStartTime = 0;
unsigned long echoDuration = 0;

float previousDistance = 0; // 上一次测量的距离
float currentDistance = 0;  // 当前测量的距离

const int movementThreshold = 10; // 检测到运动的距离阈值（cm）

void UltrasoundSM() {
  switch (currentSensorState) {
    case TRIGGER_SENSOR:
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      echoStartTime = micros();
      currentSensorState = WAIT_FOR_ECHO;
      break;

    case WAIT_FOR_ECHO:
      if (digitalRead(echoPin) == HIGH) {
        echoStartTime = micros();
        currentSensorState = READ_SENSOR;
      } else if (micros() - echoStartTime > 30000) {
        currentSensorState = TRIGGER_SENSOR;
      }
      break;

    case READ_SENSOR:
      if (digitalRead(echoPin) == LOW) {
        echoDuration = micros() - echoStartTime;
        currentSensorState = PROCESS_DATA;
      }
      break;

    case PROCESS_DATA:
      currentDistance = echoDuration * 0.034 / 2;
      if (abs(currentDistance - previousDistance) > movementThreshold) {
        Serial.println("Movement detected!");
        Serial.print("Previous Distance: ");
        Serial.println(previousDistance);
        Serial.print("Current Distance: ");
        Serial.println(currentDistance);
        digitalWrite(ledPin2, HIGH);
      } else {
        digitalWrite(ledPin2, LOW);
      }
      previousDistance = currentDistance;
      currentSensorState = TRIGGER_SENSOR;
      break;
  }
}

//---------------------
// 机器人手臂状态机
//---------------------

enum RobotArm_States {Choose_mode, Waiting_4_Player, Position0, Arm_moves_next, Reading_input, Button_pressed, Show_results, Reset_State} RobotArm_State;

void RobotArm_SM(unsigned long currentMillis) {
  switch(RobotArm_State) {
    case Choose_mode:
      Serial.println("routine: ");
      Serial.println(routine);
      if (routine != 10) {
        RobotArm_State = Waiting_4_Player;
      } else {
        RobotArm_State = Choose_mode;
      }
      break;

    case Waiting_4_Player:
      if (player_ready) {
        RobotArm_State = Position0;
        timer1 = currentMillis;
        Serial.println("Player Ready");
      } else {
        RobotArm_State = Waiting_4_Player;
      }
      break;
      
    case Position0:
      if (currentMillis - timer1 >= interval1) {
        RobotArm_State = Arm_moves_next;
        moving = true;
      } else {
        RobotArm_State = Position0;
      }
      break;

    case Arm_moves_next:
      moving = active();
      if (moving == true) {
        RobotArm_State = Arm_moves_next;
      } else {
        Serial.println("Movement executed");
        RobotArm_State = Reading_input; 
        timer2 = currentMillis;
      }
      break;

    case Reading_input:
      if (contact == 1) {
        Serial.println("PUNCHHHHH");
        Serial.println(idx);
        if (idx >= 2) { // 例程执行完毕
          RobotArm_State = Show_results;
          idx = 0;
          timer3 = currentMillis;
          Serial.println("ROUTINE DONE");
        } 
        else {
          RobotArm_State = Arm_moves_next;
          moving = 1; // 标记电机开始运动
          idx += 1;        
        }   
      } 
      else {
        if ((currentMillis - timer2) < interval2) {
          RobotArm_State = Reading_input; 
        } 
        else {
          RobotArm_State = Show_results;
          Serial.println("TIME-OUT");
          idx = 0;
          timer3 = currentMillis;
        }
      }  
      break;

    

    case Show_results:
      if (currentMillis - timer3 >= interval2) {
        RobotArm_State = Choose_mode;
      } else {
        RobotArm_State = Show_results; 
      }
      break;
  }

  // 状态下的动作
  switch(RobotArm_State) {
    case Choose_mode:
      Serial.println("Choosing routine");
      // buttonState5 = digitalRead(buttonPin5);
      // buttonState6 = digitalRead(buttonPin6);
      // buttonState7 = digitalRead(buttonPin7);
      // if (buttonState5) {
      //   routine = 0;
      // } else if (buttonState6) {
      //   routine = 1;
      // } else if (buttonState7) {
      //   routine = 2;
      // }
      break;

    case Waiting_4_Player:
      break;

    case Position0:
      idx = 0;
      if (x1 != int((currentMillis - timer1) / 1000)) {
        Serial.println("The robot moves to position 0 and waits three seconds");
        Serial.println(3 - int((currentMillis - timer1) / 1000));
        x1 = int((currentMillis - timer1) / 1000);
      }
      break;
      
    case Arm_moves_next:
      Serial.println("routine :");
      Serial.print(routine);

      Serial.println("idx : ");
      Serial.println(idx);

      Serial.println("Executing movement");
      movement_idx = routines[routine][idx];
      movements[movement_idx](); // 调用电机动作函数      
      //moving = 0; // 假定电机运动已结束
      break;
      
    case Reading_input:
      Serial.println("Reading input");

      int leftThresh = 256; 
      int rightThresh = 768; 
      int upThresh = 768; 
      int downThresh = 256; 

      int moveX = analogRead(jsX); 
      int moveY = analogRead(jsY); 

      if (moveX >= rightThresh || moveX <= leftThresh || moveY >= upThresh || moveY < downThresh) 
      {
          contact = 1;  // Joystick moved, register hit
          digitalWrite(ledPin1, HIGH);
          Serial.println("Hit detected!");
      } 
      else {
          contact = 0;
          digitalWrite(ledPin1, LOW);
      }

    break;
      
    case Show_results:
      if (x2 != int((currentMillis - timer3) / 1000)) {
        Serial.println("The results go here");
        Serial.println(5 - int((currentMillis - timer3) / 1000));
        x2 = int((currentMillis - timer3) / 1000);
      }
      break;

      case Reset_State: 
        Serial.println("Resetting Robot Arm...");
      
        s1Stepper.moveTo(0);
        s3Stepper.moveTo(0);
        elbowStepper.moveTo(0);
        baseStepper.moveTo(0);

        while (active()) {
            s1Stepper.run();
            s3Stepper.run();
            elbowStepper.run();
            baseStepper.run();
        }

        Serial.println("Reset Complete");
        RobotArm_State = Choose_mode;  // Go back to the start
      break; 
  }
}

//---------------------
// setup 与 loop
//---------------------

void setup() {
  // 状态机相关引脚初始化
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(reset, INPUT_PULLUP); 

  // pinMode(buttonPin1, INPUT);
  // pinMode(buttonPin2, INPUT);
  // pinMode(buttonPin3, INPUT);
  // pinMode(buttonPin4, INPUT);
  // pinMode(buttonPin5, INPUT);
  // pinMode(buttonPin6, INPUT);
  // pinMode(buttonPin7, INPUT);

  // 超声波传感器引脚
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // 电机步进驱动初始化
  s1Stepper.setMaxSpeed(80);
  s1Stepper.setAcceleration(100);
  
  s3Stepper.setMaxSpeed(50);
  s3Stepper.setAcceleration(50);
  
  elbowStepper.setMaxSpeed(50);
  elbowStepper.setAcceleration(100);
  
  baseStepper.setMaxSpeed(50);
  baseStepper.setAcceleration(100);
  
  s1Stepper.setCurrentPosition(0);
  elbowStepper.setCurrentPosition(0);
  s3Stepper.setCurrentPosition(0);
  baseStepper.setCurrentPosition(0);
  
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();

  if(!(digitalRead(reset)))
  {
    RobotArm_State = Reset_State;
  }

  RobotArm_SM(currentMillis);
  UltrasoundSM();
}