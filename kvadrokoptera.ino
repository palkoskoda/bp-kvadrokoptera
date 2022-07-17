#define CAL_SAMPLES     2000
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#include <EEPROM.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

//int cyklus=0;
//bool pocitaj=false;
//float timestart;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus = 0;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float clanok1, clanok2, clanok3;

// orientation/motion vars
Quaternion q_raw, q, q0;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float sroll, spitch, syaw;
bool newSampleReady = false;
VectorInt16  gyro;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

float value[17];

#define KPY value[0]
#define KIY value[1]
#define KDY value[2]
#define KPP value[3]
#define KIP value[4]
#define KDP value[5]
#define KPR value[6]
#define KIR value[7]
#define KDR (value[8])
#define KD2Y value[9]
#define KD2P value[10]
#define KD2R value[11]
#define CAL (value[12])
#define YAW value[13]
#define PITCH (-value[14])
#define ROLL value[15]
#define THROTTLE value[16]

#define MAX_PID 50.0
#define DFILTER 0.20

long lc_time;
long s_time;
boolean bateria_stop = false;
bool fpitch=0;


class PID  {
    //float sample;
    float Dsample, Dsample_buf=0;
    float lastSample, DDlastSample;
    float P, I, D,DD;
    float kP, kI, kD, kD2;
    float output;
    float lastInput=0, lastOutput;
    float Imax;
public:
    long lastProcess;

    PID() {
      kP = 1;
      kI = 0.001;
      kD = 0.001;
      I = 0;
      Imax = MAX_PID / kI;
      P = 0;
      D = 0;
      lastSample = 0;
      DDlastSample = 0;
      lastProcess = millis();
      output = 0;
    }

    void addNewSample(float Sample, float sample, float angvel) {

      if (Sample < (lastInput-1))
        Sample = (lastInput-1);
      else if (Sample > (lastInput+1))
        Sample = (lastInput+1);
        
      long t = millis();

      if (t == lastProcess) return;

      float deltaTime = (float)(t - lastProcess) / 1000.0;
            
      lastProcess = t;

      //P
      P = (Sample-sample) * kP;

      //I
      I = I + ((Sample-sample) * kI) * deltaTime;
      if (I < (-Imax))
        I = -Imax;
      else if (I > Imax)
        I = Imax;

      //D
      Dsample = ((-(Sample-lastSample) + DFILTER * Dsample ) / (deltaTime + DFILTER));
        
      D=-kD*(Dsample-angvel);
      lastSample = Sample;
      
      //DD
      DD =kD2*(-(Dsample-angvel)+DDlastSample + DFILTER * DD ) / (deltaTime + DFILTER);
      DDlastSample=(Dsample-angvel);
      /*
if (fpitch){
       Serial.print("sample\t");
       Serial.print(Dsample-angvel);
       Serial.print("\t");
       Serial.print("dsample\t");
       Serial.print(DD);
       Serial.print("\t");
       Serial.println(angvel);}*/
      

        
      if (deltaTime>0.025)
        Serial.println(deltaTime*1000);
        
//        Serial.print("PID\t");
//        Serial.print(P);
//        Serial.print("\t");
//        Serial.print(I);
//        Serial.print("\t");
//        Serial.println(D);
        
      output = P + I + D +DD;
      if (output < (-MAX_PID))
        output = -MAX_PID;
      else if (output > MAX_PID)
        output = MAX_PID;
        
      if (output < (lastOutput-5))
        output = (lastOutput-5);
      else if (output > (lastOutput+5))
        output = (lastOutput+5);
        
      lastOutput=output;
      lastInput=Sample;
    }

    void newPidConst(float _kP, float _kI, float _kD, float _kD2) {
      kP = _kP;
      kI = _kI;
      kD = _kD;
      kD2= _kD2;
      Imax = MAX_PID*kI; // / kI;
    }

    inline int getOutput() {return output;}
};

PID yawPid;
PID pitchPid;
PID rollPid;

#define MOTORBIAS 20
#define ADC_GAIN 0.0157707986880
#define BATTERY_MIN 0
double bat_time;

void riadenie()
{ 
  double t=millis();
  
    if (bateria_stop)
        bat_time=t;
    bateria_stop=false;
    
//    Serial.println(THROTTLE);    
    if (THROTTLE == 0 || t > (lc_time + 500) || t < (bat_time + 2000)||t>(s_time+500))
    {
      motor1.write(MOTORBIAS);
      motor2.write(MOTORBIAS);
      motor3.write(MOTORBIAS);
      motor4.write(MOTORBIAS);
      yawPid.lastProcess = t;
      pitchPid.lastProcess = t;
      rollPid.lastProcess = t;
      return;
    }
    else if (newSampleReady)
    {
      yawPid.addNewSample(YAW, syaw,-gyro.z);
      fpitch=1;
      pitchPid.addNewSample(PITCH, spitch,gyro.y );
      fpitch=0;
      rollPid.addNewSample(ROLL, sroll,-gyro.x);

   
      
      
      motor1.write(MOTORBIAS +transformujvystup( THROTTLE + pitchPid.getOutput() - rollPid.getOutput()));
      motor2.write(MOTORBIAS +transformujvystup( THROTTLE + pitchPid.getOutput() + rollPid.getOutput()));
      motor3.write(MOTORBIAS +transformujvystup( THROTTLE - pitchPid.getOutput() + rollPid.getOutput()));
      motor4.write(MOTORBIAS +transformujvystup( THROTTLE - pitchPid.getOutput() - rollPid.getOutput()));
      newSampleReady = false;
    }

}

int transformujvystup(int cislo)
{
  if (cislo<0)
  return 0;

  float vysledok=(float)cislo/200.0;
  vysledok=sqrt(vysledok)*200;
  return (int)vysledok;
}
void nacitaj_konstanty()
{
  EEPROM.get(0, KPY);
  EEPROM.get(4, KIY);
  EEPROM.get(8, KDY);
  EEPROM.get(12, KPP);
  EEPROM.get(16, KIP);
  EEPROM.get(20, KDP);
  EEPROM.get(24, KPR);
  EEPROM.get(28, KIR);
  EEPROM.get(32, KDR);
  EEPROM.get(36, KD2Y);
  EEPROM.get(40, KD2P);
  EEPROM.get(44, KD2R);
}

void setup()
{
  THROTTLE = 30;
  pinMode(15, INPUT_PULLUP);
  Serial.begin(115200);
  Serial3.begin(57600);

//  motor1.attach(10);
//  motor2.attach(11);
//  motor3.attach(12);
//  motor4.attach(9);
  motor1.attach(9);
  motor2.attach(12);
  motor3.attach(11);
  motor4.attach(10);
  motor1.write(MOTORBIAS);
  motor2.write(MOTORBIAS);
  motor3.write(MOTORBIAS);
  motor4.write(MOTORBIAS);
  nacitaj_konstanty();
  inicializaciasnimacov();


}
void loop()
{
  clanok1 = ADC_GAIN * (float)analogRead(A0);
  clanok2 = ADC_GAIN * (float)analogRead(A1) - clanok1;
  clanok3 = ADC_GAIN * (float)analogRead(A2) - clanok2 - clanok1;
  
//  Serial.print("aktualizacia snimacov");
//  Serial.println(THROTTLE);  
  aktualizaciasnimacov();
//  Serial.print("nacitaj");
  //Serial.println(THROTTLE);  
  nacitaj();
          
  yawPid.newPidConst(KPY, KIY, KDY, KD2Y);
  pitchPid.newPidConst(KPP, KIP, KDP, KD2P);
  rollPid.newPidConst(KPR, KIR, KDR, KD2R);
//  Serial.print("pid");
//  Serial.println(THROTTLE);  
//    Serial.print("prvý článok: ");
//    Serial.print(clanok1);
//    Serial.println("V");
//    Serial.print("druhý článok: ");
//    Serial.print(clanok2);
//    Serial.println("V");
//    Serial.print("tretí článok: ");
//    Serial.print(clanok3);
//    Serial.println("V");

  if (clanok1 < BATTERY_MIN)
  {
    Serial.print("vybitý prvý článok: ");
    Serial.print(clanok1);
    Serial.println("V");
    bateria_stop = true;
  }
  if (clanok2 < BATTERY_MIN)
  {
    Serial.print("vybitý druhý článok: ");
    Serial.print(clanok2);
    Serial.println("V");
    bateria_stop = true;
  }
  if (clanok3 < BATTERY_MIN)
  {
    Serial.print("vybitý tretí článok: ");
    Serial.print(clanok3);
    Serial.println("V");
    bateria_stop = true;
  }
//   Serial.print("riadenie");
//  Serial.println(THROTTLE);  
  riadenie();
}

void inicializaciasnimacov()
{

  /*devStatus = mpu.dmpInitialize();
    Serial.println(mpu.getXGyroOffset());
    Serial.println(mpu.getYGyroOffset());
    Serial.println(mpu.getZGyroOffset());
    Serial.println(mpu.getZAccelOffset());*/


  /* mpu.setXGyroOffset(mpu.getXGyroOffset());
    mpu.setYGyroOffset(mpu.getYGyroOffset());
    mpu.setZGyroOffset(mpu.getZGyroOffset());
    mpu.setZAccelOffset(mpu.getZAccelOffset()); // 1688 factory default for my test chip*/
// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(200000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(200, true);
    #endif
  
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
//    mpu.setXGyroOffset(220);
//    mpu.setYGyroOffset(76);
//    mpu.setZGyroOffset(-85);
//    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(0); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready!"));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    s_time = millis();
}
void aktualizaciasnimacov() 
{
  if (THROTTLE==0 && CAL==1)
  {
    mpuCalibrate();
    CAL=0;
  }
   else if (CAL==1)
  {
    q0=q_raw.getConjugate();
    //mpuCalibrate();
    CAL=0;
  }
  
  if (!mpuInterrupt && fifoCount < packetSize)
    return;

    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Read all packets out of FIFO
        while(fifoCount >= packetSize)
        {
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
        }
        
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q_raw, fifoBuffer);
        q = q0.getProduct(q_raw);
        //q = q0.q_raw;
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(euler, &q, &gravity);
        mpu.dmpGetGyro(&gyro,fifoBuffer);
        syaw = euler[0] * 180/M_PI;
        spitch = euler[1] * 180/M_PI;
        sroll = euler[2] * 180/M_PI;
//        Serial.print("ypr\t");
//        Serial.print(syaw);
//        Serial.print("\t");
//        Serial.print(spitch);
//        Serial.print("\t");
//        Serial.println(sroll);
//        
//        Serial.print("gyro\t");
//        Serial.print(gyro.z);
//        Serial.print("\t");
//        Serial.print(gyro.y);
//        Serial.print("\t");
//        Serial.println(gyro.x);
        newSampleReady = true;
        s_time =millis();
    }
    else if (millis()>(s_time+100))
    {
       int16_t offx = mpu.getXGyroOffset();
       int16_t offy = mpu.getYGyroOffset();
       int16_t offz = mpu.getZGyroOffset();

       inicializaciasnimacov();

       mpu.setXGyroOffset(offx);
       mpu.setYGyroOffset(offy);
       mpu.setZGyroOffset(offz); 
    }
}




void dmpDataReady() 
{
    mpuInterrupt = true;
}


void mpuCalibrate()
{
  Serial3.end();
  Serial.print(F("Calibrating DMP..."));
  mpu.setDMPEnabled(false);

  // Clear previous biases
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  // measure gyro and accelerometer biases
  int32_t g_x = 0, g_y = 0, g_z = 0;
    for (int i = 0; i < CAL_SAMPLES; i++)
  {
    // wait for sample
    while ( !mpu.getIntDataReadyStatus() )
      ;
  }
  
  for (int i = 0; i < CAL_SAMPLES; i++)
  {
    // wait for sample
    while ( !mpu.getIntDataReadyStatus() )
      ;

    // read sample
    g_x += mpu.getRotationX();
    g_y += mpu.getRotationY();
    g_z += mpu.getRotationZ();
  }

  // Assign new biases
  mpu.setXGyroOffset(g_x / CAL_SAMPLES);
  mpu.setYGyroOffset(g_y / CAL_SAMPLES);
  mpu.setZGyroOffset(g_z / CAL_SAMPLES);

  // Flush FIFO
  mpu.resetFIFO();

  // reset state in DMP
  //mpu.resetDMP();

  // start DMP
  mpu.setDMPEnabled(true);
  Serial.println(F("done."));
  Serial3.begin(57600);
  s_time = millis();
}




