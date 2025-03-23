// STEP and DIRECTION pins
const int stepPin = 3;
const int dirPin = 4;

// Number of steps per full revolution
const float stepsPerRevolution = 200.0;  // 200 full steps = 360° (1.8° per step)

void setup() {
  // Initialize 
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Set the motor direction 
  digitalWrite(dirPin, HIGH);

  // Start serial 
  Serial.begin(9600);
  Serial.println("Stepper Motor Maximum Speed Test");
}

void loop() {
  // Test speeds from a low value to a high value in degrees/sec
  // Here we choose speeds corresponding to motor RPM from 10 to 200 in steps of 10.
  // degrees per second = RPM * 6 
  // these RPM values correspond to speeds from 60°/sec to 1200°/sec.
  for (int rpm = 10; rpm <= 200; rpm += 10) {
    // Calculate steps per second: (RPM / 60) * stepsPerRevolution
    float stepsPerSec = (rpm / 60.0) * stepsPerRevolution;
    
    // Calculate delay between steps in microseconds.

    unsigned long delayMicrosBetweenSteps = 1000000.0 / stepsPerSec;

   
    float degPerSec = rpm * 6.0;  // 360°/60 sec = 6° per RPM
    Serial.print("Testing speed: ");
    Serial.print(degPerSec);
    Serial.println(" deg/sec");

    // Run the motor at this speed for 2 seconds
    unsigned long testDuration = 2000; 
    unsigned long startTime = millis();
    while (millis() - startTime < testDuration) {
      // Send one step pulse
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(delayMicrosBetweenSteps / 2);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(delayMicrosBetweenSteps / 2);
    }
    
    
    delay(2000);
  }
  
  // Once all speeds have been tested, stop running the loop.
  while (true) {
  }
}
