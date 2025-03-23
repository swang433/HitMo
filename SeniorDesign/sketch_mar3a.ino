#include <AccelStepper.h>

//---------------------
// Motor Control Section
//---------------------

// Motor pin definitions
const int s1dirPin    = 46;
const int s1stepPin   = 47;
const int edirPin     = 48;
const int estepPin    = 49;
const int basedirPin  = 52; 
const int basestepPin = 53; 
const int s3dirPin    = 50;
const int s3stepPin   = 51;

// Create stepper motor objects
AccelStepper s1Stepper   (AccelStepper::DRIVER, s1stepPin, s1dirPin);
AccelStepper elbowStepper(AccelStepper::DRIVER, estepPin, edirPin);
AccelStepper baseStepper (AccelStepper::DRIVER, basestepPin, basedirPin);
AccelStepper s3Stepper   (AccelStepper::DRIVER, s3stepPin, s3dirPin);

// Convert angles to steps (Stepper motor moves 1.8° per step)
float rot(float deg) {
  return deg / 1.8;
}

// Motor movement functions (only blockFront, blockDown, blockSide5 are retained, jab() has been removed)
void blockFront() {
  Serial.println("Blocking front");

  s1Stepper.moveTo(rot(-45));   // 45°
  s3Stepper.moveTo(rot(90));    // 90°
  elbowStepper.moveTo(rot(15)); // 15°
  baseStepper.moveTo(0);        // 0°

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
  Serial.println("Blocking below");

  s1Stepper.moveTo(rot(15));   // 15°
  s3Stepper.moveTo(rot(90));   // 90°
  elbowStepper.moveTo(rot(15));// 15°
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
  Serial.println("Blocking side");

  s1Stepper.moveTo(rot(-45));   // 45°
  s3Stepper.moveTo(rot(15));    // 15°
  elbowStepper.moveTo(rot(-45));// 45°
  baseStepper.moveTo(rot(30));  // 30°

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
void motors_stopped() {
  s1Stepper.stop();   
  s3Stepper.stop();    
  elbowStepper.stop();
  baseStepper.stop();  
}

void hold_0() {
  //Serial.println("Blocking side");

  s1Stepper.moveTo(0);   // 45°
  s3Stepper.moveTo(0);    // 15°
  elbowStepper.moveTo(0);// 45°
  baseStepper.moveTo(0);  // 30°

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
// State Machine Section (RobotArm_SM & UltrasoundSM)
//---------------------

// FLAGS and TIMERS
int player_ready = 1; // Assume the player is ready
bool moving  = true;
int contact = 0;

unsigned long timer1 = 0; // Position0 state timer (waits for 3 seconds)
unsigned long timer2 = 0; // Reading_input state timer (5-second timeout)
unsigned long timer3 = 0; // Show_results state timer (5-second result display)
unsigned long timer4 = 0; // Waiting_4_Player state timer (10-second timeout)

const long interval1 = 3000;  // 3 seconds
const long interval2 = 5000;  // 5 seconds
const long interval3 = 15000; // 10 seconds

int ledPin1 = 2; // Indicates when the robotic arm is done moving, player can make a move
int ledPin2 = 3; // Indicates program start

// Joystick pins
const int jsX = A0; 
const int jsY = A1; 

// Define movement routines: rows represent different routines, columns represent each step in the routine
int routines[3][12] = {
  {0, 1, 0,0, 0, 0, 0 , 0, 2,0,0,2},
  {0, 1, 2,0, 1, 2, 0 , 1, 2,0,1,2},
  {2, 0, 1, 0, 0, 0, 0 , 0, 2,0,0,2}
};

int idx = 0;         // Current step index of the routine (0~2)
int end_idx = 11;    // Not used
int routine = 1;     // Currently selected routine (default: 1)
bool routine_chosen = false; //Flag to check if the routine has been chosen 
int movement_idx = 0;// Movement index from routine matrix
int x1 = 0;
int x2 = 0;

// Array of movement function pointers pointing to motor movement functions
void (*movements[])() = { blockFront, blockDown, blockSide5 };

//---------------------
// Ultrasound Sensor State Machine
//---------------------

enum SensorState { TRIGGER_SENSOR, WAIT_FOR_ECHO, READ_SENSOR, PROCESS_DATA };
SensorState currentSensorState = TRIGGER_SENSOR;

const int trigPin = 9;
const int echoPin = 10;
unsigned long echoStartTime = 0;
unsigned long echoDuration = 0;

float previousDistance = 0; // Previous distance measurement
float currentDistance = 0;  // Current distance measurement

const int movementThreshold = 10; // Distance threshold for detecting movement (cm)




//---------------------
// Robotic Arm State Machine
//---------------------

enum RobotArm_States {Choose_mode, Arm_moves_next, Reading_input, Show_results} RobotArm_State;
void RobotArm_SM(unsigned long currentMillis) {
  switch(RobotArm_State) {
    // case Wait:
    // if ((currentMillis - timer1) < interval1) {
    //       RobotArm_State = Wait; 
    //       Serial.println("Waiting");
    //     }
    // else {
    //       RobotArm_State = Choose_mode; 
    //       Serial.println("Done waiting");     
    // } 
        
    //break;
    case Choose_mode:  
      if (routine_chosen) {
        RobotArm_State = Arm_moves_next;
        routine_chosen = false; //reset FLAG
        idx = 0; //The routine is chosen so set the idx to first step        
        //timer1 = currentMillis; // Start 3 seconds count down 
        Serial.println("Player Ready AND routine chosen");
      }
      else {
        RobotArm_State = Choose_mode;
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
        Serial.print("STRIKE: ");
        Serial.println(contact);
               
        if (idx >= end_idx) { // routine completed
          RobotArm_State = Show_results;
          idx = 0;
          timer3 = currentMillis;
          Serial.println("ROUTINE DONE");
        } 
        else {
          RobotArm_State = Arm_moves_next;
          moving = true; // motors start moving
          idx += 1;        
        }   
      } 
      else {
        if ((currentMillis - timer2) < interval3) {
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
        routine_chosen = false; // reset flag        
      } else {
        RobotArm_State = Show_results; 
      }
      break;
  }

  // 状态下的动作
  switch(RobotArm_State) {
    // case Wait:
    // break;
    s1Stepper.setSpeed(0);
    s3Stepper.setSpeed(0);
    elbowStepper.setSpeed(0);
    baseStepper.setSpeed(0);
    case Choose_mode:
      //hold_0()
      motors_stopped();       
      if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      if (input.startsWith("START")) {
        // Extract the integer value after "START"
        int routine = input.substring(6).toInt(); // this value contains routine number chosen by user
        routine_chosen = true;
        
        // Blink onboard LED as confirmation
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(200);
          digitalWrite(LED_BUILTIN, LOW);
          delay(200);
        }
        
        Serial.print("STATUS: Routine "); 
        Serial.print(routine);
        Serial.println(" received and LED blinked");
      } else {
        // Handle other commands (e.g., countdown numbers)
        Serial.print("STATUS: Received ");
        Serial.println(input);
      }
    }
      //delay(3000);
      break;
      
    case Arm_moves_next:
      Serial.println("Executing movement");
      movement_idx = routines[routine][idx];
      movements[movement_idx](); // call the motor funtcion    
      //SEND TO RASBERRY PI
        Serial.print("Robot position: ");
        Serial.println(movement_idx) ;  
      break;
      
    case Reading_input:
      //Serial.println("Reading input");
      motors_stopped();

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
          //Serial.println("Hit detected!");
      } 
      else {
          contact = 0;
          digitalWrite(ledPin1, LOW);
      }

    break;
      
    
      
    case Show_results:
      motors_stopped();
      if (x2 != int((currentMillis - timer3) / 1000)) {
        Serial.println("The results go here");
        Serial.println(5 - int((currentMillis - timer3) / 1000));
        x2 = int((currentMillis - timer3) / 1000);
      }
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
  
  Serial.begin(115200);
  //timer1 = currentMillis; // Start 3 seconds count down 

}

void loop() {
  unsigned long currentMillis = millis();
  RobotArm_SM(currentMillis);
}