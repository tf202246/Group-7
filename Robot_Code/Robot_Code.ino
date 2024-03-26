// Define constant values and global variables.

// Define the pin numbers on which the outputs are generated.
#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 9
#define MOT_B2_PIN 10
#define LEFT_IND_PIN 12
#define RIGHT_IND_PIN 13
#define IND_THRESHOLD 5
#define BRAKE_PIN 11

boolean reverseL = 0; 
boolean reverseR = 0;

/* variables to keep track of current speed of motors */
int leftServoSpeed = 0;
int rightServoSpeed = 0;

/* variables to keep track of consistent turning*/
int leftind = 0;
int rightind = 0;

int turn = 0;

int objectCounter = 0;

/* Define the pins for the IR sensors */
const int irPins[3] = {A2, A1, A0};

/* Define values for the IR Sensor readings */
int irSensorDigital[3] = {0, 0, 0};

int threshold = 550; // IR sensor threshold value for line detection


const int maxSpeed = 180; // the range for speed is(0,255)

// binary representation of the sensor reading
// 1 when the sensor detects the line, 0 otherwise
int irSensors = B000;

// A score to determine deviation from the line [-180 ; +180].
// Negative means the robot is left of the line.
int error = 0;

int errorLast = 0;  //  store the last value of error

/* variables for distance sensors, define pins */
int trigPin = 3;    // TRIG pin
int echoPin = 2;    // ECHO pin

float duration_us, distance_cm;

// ================================================================================
void setup(void)
{
  // Initialize the stepper driver control pins to output drive mode.
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);

  // Initialize the serial UART at 9600 bits per second.
  Serial.begin(9600);

   // configure the trigger pin to output mode
  pinMode(trigPin, OUTPUT);
  // configure the echo pin to input mode
  pinMode(echoPin, INPUT);

  delay(3000);
}
// ================================================================================

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}
// ================================================================================

void set_motor_currents(int pwm_A, int pwm_B)
{
  set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);

  // Print a status message to the console.
  Serial.print("Set motor A PWM = ");
  Serial.print(pwm_A);
  Serial.print(" motor B PWM = ");
  Serial.println(pwm_B);
}

// ================================================================================

void spin_and_wait(int pwm_A, int pwm_B, int duration)
{
  set_motor_currents(pwm_A, pwm_B);
  delay(duration);
}

// ================================================================================

void Scan() {
  // Initialize the sensors
    irSensors = B000;

  for (int i = 0; i < 3; i++) {
      int sensorValue = analogRead(irPins[i]);
      if (sensorValue >= threshold) {
        irSensorDigital[i] = 1;
      }

    else {
      irSensorDigital[i] = 0;
    }

    int b = 2-i;
    irSensors = irSensors + (irSensorDigital[i]<<b);
    }
}

// ================================================================================

void UpdateDirection() {

  errorLast = error;

  switch (irSensors) {

    case B000: // no sensor detects the line
       if (errorLast < 0) { error = -130; leftind = leftind + 1; rightind = 0;}
       else if (errorLast > 0) {error = 130; rightind = rightind + 1; leftind = 0;}
       break;

     case B100: // left sensor on the line
       error = -120;
       leftind = leftind + 1;
       rightind = 0;
       break;

     case B110:
       error = -60;
        leftind = leftind + 1;
        rightind = 0;
       break;

     case B010: // Straight
       error = 0;
        leftind = 0;
        rightind = 0;
       break;

     case B011:
       error = 60;
        rightind = rightind + 1;
        leftind = 0;
       break;

     case B001: // right sensor on the line
       error = 120;
        rightind = rightind + 1;
        leftind = 0;
       break;

     case B111:
       error = 1;
        leftind = 0;
        rightind = 0;
       break;

     default:
       error = errorLast;
  }

    if (error >= 0) {
      leftServoSpeed = maxSpeed;          //180
      rightServoSpeed = maxSpeed - error; //100
    }

    else if (error < 0) {
      leftServoSpeed = maxSpeed + error;  //100
      rightServoSpeed = maxSpeed;         //180
    }
}

// ================================================================================

void distance_sensor() {
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(echoPin, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

  // if certian distance fron object stop 
  if(distance_cm < 20){
    objectCounter = objectCounter + 1;
    if (objectCounter==5){
    digitalWrite(BRAKE_PIN, HIGH);
    if(turn<0){
    leftServoSpeed = -100;          
    rightServoSpeed = 100;
    }
    else{
    leftServoSpeed = 100;          
    rightServoSpeed = -100;
    }
    spin_and_wait(leftServoSpeed,rightServoSpeed,1000); // sets speed for 0.01 sec
    Scan();
    while(irSensors == B000){
      Scan();
      spin_and_wait(leftServoSpeed,rightServoSpeed,10); // sets speed for 0.01 sec
    }
    turn = 0;
    digitalWrite(BRAKE_PIN, LOW);
    objectCounter = 0;
  }
  }
} 

// ================================================================================

void indicators() {
if (leftind > 10) {
digitalWrite(LEFT_IND_PIN, HIGH);
turn = turn - 1;
}
else if (rightind > 10) {
digitalWrite(RIGHT_IND_PIN, HIGH);
turn = turn + 1;
}
else {
digitalWrite(RIGHT_IND_PIN, LOW);
digitalWrite(LEFT_IND_PIN, LOW);
}
}

// ================================================================================
  
void loop()
{
Scan(); 
UpdateDirection();
distance_sensor();
spin_and_wait(leftServoSpeed,rightServoSpeed,10); // sets speed for 0.01 sec
indicators();  
}
