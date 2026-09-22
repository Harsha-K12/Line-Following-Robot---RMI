
//IR sensor pins
#define S1_PIN 34
#define S2_PIN 35
#define S3_PIN 32
#define S4_PIN 33
#define S5_PIN 25
#define S6_PIN 26
#define S7_PIN 27
#define S8_PIN 14
#define CALIBRATE_BUTTON 19
#define AIN2 16 //A02 - M2 LEFT MOTOR
#define AIN1 17 //AO1 - M1 LEFT MOTOR
#define BIN1 18 //BO1 - M2 RIGHT MOTOR
#define BIN2 21 //BO2 - M1 RIGHT MOTOR
#define STBY 2
#define PWMA 4
#define PWMB 5

const int SENSOR_COUNT= 8; //number of sensors
const int sensorPins[SENSOR_COUNT] = {S1_PIN, S2_PIN, S3_PIN, S4_PIN, S5_PIN, S6_PIN, S7_PIN, S8_PIN};
volatile bool calibrateState = false;
const int weight[SENSOR_COUNT] = {-210*3/2, -150*3/2, -90, -30, 30, 90, 150*3/2, 210*3/2}; //Initial Weights assumption approx 6mm pitch
unsigned long calibCheck=0;
unsigned long calibDuration=150;

//Calibration
int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT];
const int LINE_LOST_THRESHOLD = 300;               // sum of normalized readings below this = line lost
int startNum=0;

//PID
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.0;

int BASE_SPEED = 140;   //cruising PWM
int MAX_SPEED  = 255;

float integral = 0.0;
float lastError = 0.0;
int lastPosition = 0.0;
unsigned long lastTime=0;

void IRAM_ATTR calibrateButton(){
  if ((millis() - calibCheck)>calibDuration){
    calibCheck=millis();
    calibrateState=!calibrateState;
    if (startNum==0) startNum=1;
  }

}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < SENSOR_COUNT; i++) pinMode(sensorPins[i], INPUT);
  pinMode(CALIBRATE_BUTTON,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CALIBRATE_BUTTON),calibrateButton,FALLING);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
}
 pinMode(STBY,OUTPUT);
 pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
 pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
 pinMode(PWMA,OUTPUT); pinMode(PWMB,OUTPUT);
}


//Normalizing one channel to 0-1000 
int readNormalized(int i) {
  int raw = analogRead(sensorPins[i]);
  raw = constrain(raw, sensorMin[i], sensorMax[i]);
  if (sensorMax[i] == sensorMin[i]) return 0; // avoiding zero division error
  return map(raw, sensorMin[i], sensorMax[i], 0, 1000);
}
void stopMotors(){
  digitalWrite(AIN1,0);
  digitalWrite(AIN2,0);
  digitalWrite(BIN1,0);
  digitalWrite(BIN2,0);
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  digitalWrite(STBY,0);
}
//Weighted position estimate
bool computePosition(int &positionOut) {
  long num = 0;
  long den = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int n = readNormalized(i);
    num += (long)weight[i] * n;
    den += n;
  }
  if (den < LINE_LOST_THRESHOLD) return false; // no sensor sees enough line
  positionOut = num / den;
  return true;
}

//function for controlling inidividual motor
void setMotor(int pinIn1, int pinIn2, int pinPWM, int speed) {
  digitalWrite(STBY,1);
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) {
    digitalWrite(pinIn1, 1);
    digitalWrite(pinIn2, 0);
  } else {
    digitalWrite(pinIn1, 0);
    digitalWrite(pinIn2, 1);
    speed = -speed;
  }
  analogWrite(pinPWM, speed);
}

// ---------------- Main loop ----------------
void loop() {
if (!calibrateState&&(startNum==1)){
  int position;
  bool onLine = computePosition(position);

  if (onLine) {
    lastPosition = position;
  } else {
    position = lastPosition;
  }

  unsigned long now = millis();
  float dt = (float)(now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  float error = position/4.0; //4 is picked aritrarily
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  lastTime = now;
  Serial.println(position);
  delay(100);
  int leftSpeed  = constrain((int)(BASE_SPEED - output), -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain((int)(BASE_SPEED + output), -MAX_SPEED, MAX_SPEED);

  setMotor(AIN1, AIN2, PWMA, leftSpeed);
  setMotor(BIN1, BIN2, PWMB, rightSpeed);

}
else if(startNum==1) {
  stopMotors();
  Serial.println("Calibrating - sweep the sensor bar across the line now...");

  for (int i = 0; i < SENSOR_COUNT; i++) {
      int v = analogRead(sensorPins[i]);
      if (v < sensorMin[i]) sensorMin[i] = v;
      if (v > sensorMax[i]) sensorMax[i] = v;
  }
  delay(1000);
  for (int i = 0; i < SENSOR_COUNT; i++) {
  Serial.print(i+1); Serial.print(" min="); Serial.print(sensorMin[i]);
  Serial.print(" max="); Serial.println(sensorMax[i]);  
  }
}
}
