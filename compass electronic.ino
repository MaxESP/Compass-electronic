#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>
#include <Adafruit_LSM303DLH_Mag.h>

Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);


//#include <MPU6050_tockn.h>
#include <Wire.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif


MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//MPU6050 mpu6050(Wire);


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




int deg= 0;
String text= "";
 String text1 = "";
String message1="";
String message="";
float Xhorizontal;
float Yhorizontal;


float roll =0;
float pitch=0;
float yaw=0;


long previousMillis = 0;
long previousMillis1 = 0;
long interval = 100;


void setup(void) {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
       Wire.begin();
  // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
//    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//        Fastwire::setup(400, true);
    #endif

  
    Serial.begin(115200);
   mpu.initialize();
pinMode(INTERRUPT_PIN, INPUT);

if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(40);
    mpu.setYGyroOffset(99);
    mpu.setZGyroOffset(79);
    mpu.setXAccelOffset(-897); 
    mpu.setYAccelOffset(-1969); 
    mpu.setZAccelOffset(1705); 
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
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
      
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



void loop(){

// if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
      
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    
       

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


            
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[2] * 180/M_PI);
//          
//            Serial.println();

        #endif

  

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }  
unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;
      
compass1();
compass(); 

}

}


void compass() {
sensors_event_t event;
 mag.getEvent(&event);


 
float roll =(ypr[2] * 180/M_PI);
float pitch=(ypr[1] * 180/M_PI);

float yaw=(ypr[0] * 180/M_PI);

//float roll =(ypr[1] * 180/M_PI);
//float pitch=(ypr[2] * 180/M_PI);


//float roll1= constrain(roll,-90.00,90.00);
//float pitch1= constrain(pitch,-90.00,90.00);

Serial.println (roll);
Serial.println (pitch);

float X=(event.magnetic.x);
float Y=(event.magnetic.y);
float Z=(event.magnetic.z);
//float X=(event.magnetic.x);
//float Y=(event.magnetic.y);
//float Z=(event.magnetic.z);


float Xhorizontal = X*cos(pitch*-0.017453) + Y*sin(roll*0.017453)*sin(pitch*-0.017453) - Z*cos(roll*0.017453)*sin(pitch*-0.017453); 
float Yhorizontal = Y*cos(roll*0.017453) + Z*sin(roll*0.017453) ;
//
//float Xhorizontal = X*cos(pitch*0.017453) + Y*sin(roll*0.017453)*sin(pitch*0.017453) - Z*cos(roll*0.017453)*sin(pitch*0.017453); 
//float Yhorizontal = Y*cos(roll*0.017453) + Z*sin(roll*0.017453) ;
//float Xhorizontal= X*cos(pitch*0.017453) + Z*sin(pitch*0.017453) ;
//1float Yhorizontal = X *sin(roll*0.017453)*sin(pitch*-0.017453)+ Y*cos(roll*0.017453)-Z *sin(roll*-0.017453)*cos(pitch*-0.017453);

//


//float heading = atan2(Xhorizontal, Yhorizontal);
float heading = (atan2(Yhorizontal, Xhorizontal)*(180/3.14));

float declinationAngle =  -0.015708;
  heading += declinationAngle;


if(heading < 0)
      heading += 2*PI;
      
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
//if (Yhorizontal< 0)heading=180+(180+((atan2(Yhorizontal,Xhorizontal))*(180/3.14)));
//else heading=(atan2(Yhorizontal,Xhorizontal))*(180/3.14);

//float declinationAngle =  -0.015708;
//  heading += declinationAngle;


     
float headingDegrees1 = heading;
    
//  if(heading > 2*PI)
//    heading -= 2*PI;   
    
//float declinationAngle =  0.0174;
// heading += declinationAngle;



//float headingDegrees1 = heading * 180/M_PI;

 int deg1 = (headingDegrees1);

String tilt = String(deg1);
  Serial.println(tilt);  

}
void compass1(){
sensors_event_t event;
 mag.getEvent(&event);





 

 float heading = (atan2(event.magnetic.y, event.magnetic.x)*(180/3.14));
 
 float declinationAngle =  -0.015708;
  heading += declinationAngle;

if(heading < 0)
    heading += 2*PI;

 if(heading > 2*PI)
    heading -= 2*PI;
float headingDegrees = heading ; 
int deg = (headingDegrees);
    
    String text = String(deg);

 String text1 = "compass_N";
  String compass = String(text1)+":"+String(deg);
   
 Serial.println(compass);
 
  
}
