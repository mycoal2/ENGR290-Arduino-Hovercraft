// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "HCSR04.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define INTERRUPT_PIN 2

Servo servo0;
MPU6050 mpu;
void calculate_IMU_error();

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; 
VectorFloat gravity; 
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
int c = 0;
int j = 0;
float correct;



// US SENSOR SPECIFIC
// FRONT = P10
int frontTrigPin = 12; // yellow jumper --> PB4: pin 12
int frontEchoPin = 8;// blue jumper --> PB0 : pin 8
// Right = P6
int rightTrigPin = 11; // yellow jumper --> PB3 : pin 11
int rightEchoPin = 2; // blue jumper --> PD2 : pin 2

// LEFT = P13   
//int leftTrigPin = 13; // yellow jumper --> PB5 : pin 13 
//int leftEchoPin = 3;// blue jumper --> PD3 : pin 3 

// lift fan P17
int liftPin = 7; // PD7
// thrust fan P4
int thrustPin = 6; // PD6

UltraSonicDistanceSensor frontSensor(frontTrigPin, frontEchoPin);
UltraSonicDistanceSensor rightSensor(rightTrigPin, rightEchoPin);
//UltraSonicDistanceSensor leftSensor(leftTrigPin, leftEchoPin);
    
    
int frontSensorDistance;
int rightSensorDistance;
//int leftSensorDistance;

int turnDirection = 0; // 0=>forward --- 1=>TURN RIGHT --- 2=>TURN LEFT
int currentYaw;
int lol = 0;
const int ledPin = 13;      // activity LED pin
bool blinkState = false; // state of the LED
int counter = 0;

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    pinMode(liftPin, OUTPUT);
    pinMode(thrustPin, OUTPUT);

    servo0.attach(9);
    
    
    Serial.begin(115200);
    while (!Serial);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  calculate_IMU_error();
  delay(20);

  // COM2A1 --> clear OC2A on compare match
  // COM2B1 --> clear OC2B on compare match
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // WGM --> WaveForm Generation Mode
  // set prescaler to 64 (i.e. divide clock speed of timer 1)
  TCCR2B = (1 << CS22);

//    Wire.setWireTimeout(3000,true);
    delay(500);
    digitalWrite(liftPin, HIGH);
    analogWrite(thrustPin, 220);
}

void loop() {

  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      ypr[0] = ypr[0] * 180 / M_PI;
      ypr[1] = ypr[1] * 180 / M_PI;
      ypr[2] = ypr[2] * 180 / M_PI;
      if (j <= 300) {
        correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
        j++;
      } else {
        ypr[0] = ypr[0] - correct;
      }
    Serial.print("Yaw: ");
    Serial.println(ypr[0]);
  }

  
    if(turnDirection == 1) {
      //TURN LEFT
      servo0.write(25);
      analogWrite(thrustPin, 240);
      int diff = abs(abs(ypr[0]-currentYaw)-90);
      Serial.print("Diff left: ");
      Serial.println(diff);
      
      if(diff > 45) {
        if (lol % 15 == 0) {
          blinkState = !blinkState;
          digitalWrite(ledPin, blinkState);
        } 
      lol++;
      return;
      }
      turnDirection = 0;
      servo0.write(90);
      delay(500);

    } else if(turnDirection == 2) {
      //TURN RIGHT
      servo0.write(155);
      analogWrite(thrustPin, 240);
      int diff = abs(abs(ypr[0]-currentYaw)-90);
      Serial.print("Diff right: ");
      Serial.println(diff);
      
       if(diff > 45) {
        if (lol % 15 == 0) {
           blinkState = !blinkState;
          digitalWrite(ledPin, blinkState);
        } 
      lol++;
        return;
      }
      turnDirection = 0;
      servo0.write(90);
      delay(500);

    } else {
      //GO STRAIGHT
      servo0.write(90);
      analogWrite(thrustPin, 220);
    }


while(lol < 5)
{
    frontSensorDistance = frontSensor.measure_distance_cm();
    rightSensorDistance = rightSensor.measure_distance_cm();
    lol++;
    delay(500);
    return;
}

    frontSensorDistance = frontSensor.measure_distance_cm();
    rightSensorDistance = rightSensor.measure_distance_cm();
//    leftSensorDistance  = leftSensor.measure_distance_cm();

 
     Serial.print("Front Distance: ");
     Serial.println(frontSensorDistance);

     Serial.print("Right Distance: ");
     Serial.println(rightSensorDistance);
//
//    if(turnDirection == 0 && frontSensorDistance < 100 ) {
//      analogWrite(thrustPin, 0);
//    } else {
//      analogWrite(thrustPin, 220);
//    }
    
    Serial.print(ypr[0]);
      if(frontSensorDistance < 50) {
        if(rightSensorDistance > 32) { 
          // NEED TO TURN RIGHT
          turnDirection = 2;
          currentYaw = ypr[0];
        } else {  
          // NEED TO TURN LEFT
          currentYaw = ypr[0];
          turnDirection = 1;
        }
      } else {
        turnDirection = 0;
      }



  if (lol % 15 == 0) {
    blinkState = !blinkState;
    digitalWrite(ledPin, blinkState);
  } 
  lol++;
  if(lol > 1505) {
    lol = 5; 
  }
}








void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;  
    
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

}
