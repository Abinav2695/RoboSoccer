#include "I2Cdev.h"
#include "var.h"
#include "TeensyThreads.h"


#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;


/***************MPU PIN AND VARIABLE DEFINITIONS*******************/
#define MPU_INTERRUPT_PIN 17     


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
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup()
{
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Soccer Robot Ready");

  delay(STARTUP_DELAY);
  indicationLed_setup();
  esp_setup();
  _serial_init();

  mpu_setup();
  pwm_setup();
  dribbler_setup();
  kicker_setup();

  //threads.addThread(read_gyrodata);
  threads.addThread(kick);
  delay(100);
  esp_enable(true);
  led_enable(true);

}

void loop()
{
  // put your main code here, to run repeatedly:
  esp_data();
  //rasp_pi_data();
  read_gyrodata();
  //manage_graphics();
}







void mpu_setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(MPU_INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
#ifdef BLUE_BOTS

  mpu.setXAccelOffset(calibrationValuesBlue[BLUE_BOTS][0]);
  mpu.setYAccelOffset(calibrationValuesBlue[BLUE_BOTS][1]);
  mpu.setZAccelOffset(calibrationValuesBlue[BLUE_BOTS][2]);
  mpu.setXGyroOffset(calibrationValuesBlue[BLUE_BOTS][3]);
  mpu.setYGyroOffset(calibrationValuesBlue[BLUE_BOTS][4]);
  mpu.setZGyroOffset(calibrationValuesBlue[BLUE_BOTS][5]);


#elif TURQUOISE_BOTS

  mpu.setXAccelOffset(calibrationValuesTurquoise[TURQUOISE_BOTS][0]);
  mpu.setYAccelOffset(calibrationValuesTurquoise[TURQUOISE_BOTS][1]);
  mpu.setZAccelOffset(calibrationValuesTurquoise[TURQUOISE_BOTS][2]);
  mpu.setXGyroOffset(calibrationValuesTurquoise[TURQUOISE_BOTS][3]);
  mpu.setYGyroOffset(calibrationValuesTurquoise[TURQUOISE_BOTS][4]);
  mpu.setZGyroOffset(calibrationValuesTurquoise[TURQUOISE_BOTS][5]);

#endif


  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(MPU_INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
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

}



void read_gyrodata()
{
  if (!dmpReady)return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    current_angle = (ypr[0] * 180 / M_PI);

    //    Serial.println(current_angle);
    if (rotate)
    {
      if (current_angle < (target_angle + 3) && current_angle > (target_angle - 3))
      {
        _stop();
        rotate = false;
        prev_angle = current_angle;
      }
    }
    String data = String(int(ypr[0] * 180 / M_PI))+';'; //+ ',' + String(int(aaWorld.x)) + ',' + String(int(aaWorld.y)) + ';';
    
    if(millis()-lastROSUpdateTime>20)
    {
      ESP.print(data);
      lastROSUpdateTime=millis();  
    }
    
    
#ifdef DEBUG_MPUDATA
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);

#endif
  }
  //threads.delay(1);
  //threads.yield();

}


void manage_graphics()
{
  if(millis()-lastGraphicsUpdateTime>5000)
  {
    RASP_PI.println(rpiDisplayCommands[graphicsRollCount]);
    RASP_PI.println(rpiAudioCommands[audioRollCount]);
    graphicsRollCount++;
    audioRollCount++;
    if(graphicsRollCount==5)graphicsRollCount=0;
    if(audioRollCount==10)audioRollCount=0;
    lastGraphicsUpdateTime=millis();
   
  }
}
