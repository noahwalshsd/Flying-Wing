#include <Servo.h>
#define pitch 8
#define roll 7
#define lr 6
#define switch1 9
Servo left;
Servo right;
Servo rudder;

//elevon mixing consts
const float pitchcmd=.5;
const float rollcmd=.5;
const float mixconst=.02;

//servo consts vals
const int low_bound_lr=1000;
const int up_bound_lr=2000;
const int low_bound_rudder=70;
const int up_bound_rudder=110;
const int switch_barrier=1500;
const int elevon_low_bound = 40;
const int elevon_up_bound = 140;
float l = 90;
float r = 90;
float p = 90;
float ro = 90;
//basic inputs manual
int x=1500;
int y=1500;
int z=1500;

//everything needed for balanced
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
float Kp = 2.0; 
float Ki = 0.0;
float Kd = 0.2;
Adafruit_MPU6050 mpu;
const float dt=.045;
//basic inputs balanced
float desroll=0;
float despitch=0;
float actpitch = 0;
float actroll = 0;
float pitchdiff = 0;
float rolldiff = 0;
// Global variables for tracking integrals
float integralR=0;
float integralP=0;
// Sensors data structures
sensors_event_t accel, gyro, temp;

void setup() {
  // put your setup code here, to run once:
  pinMode(pitch, INPUT);
  pinMode(roll, INPUT);
  pinMode(lr, INPUT);
  Serial.begin(9600);
  rudder.attach(16);
  left.attach(15);
  right.attach(14);
  Wire.begin();
  if (!mpu.begin()) {
    Serial.print("failed to recognise");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // put your main code here, to run repeatedly:
  //check values first
  float s1=pulseIn(switch1,HIGH);
  y=pulseIn(pitch, HIGH)-1500;
  z=pulseIn(roll, HIGH)-1500;
  //manual flight mode
  if (s1<switch_barrier){
    //aeleron vals
    p=y;
    ro=z;
  }
  if (s1>1500){
    //get desired inputs
    desroll=z*.09;
    despitch=y*.09;
    //get sensor inputs for accel and gyro in pitch & roll
    mpu.getEvent(&accel, &gyro, &temp);
    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;
    float accelPitch = atan2(ax, az) * 180.0 / PI;
    float accelRoll  = atan2(ay, az) * 180.0 / PI;
    float cpitch = gyro.gyro.x * 180.0 / PI;
    float croll  = gyro.gyro.y * 180.0 / PI;
    actpitch = 0.98 * (actpitch + cpitch * dt) + .02 * accelPitch;
    actroll = 0.98 * (actroll + croll * dt) + .02 * accelRoll;

    // define difference calculations
    pitchdiff=despitch-actpitch;
    rolldiff=desroll-actroll;
    //round similar
    if (abs(pitchdiff) < 0.5) pitchdiff = 0;
    if (abs(rolldiff) < 0.5) rolldiff = 0;
    //Define integral
    integralP += pitchdiff*dt;
    integralR += rolldiff*dt;
    // Constrain integrals to prevent windup
    integralP = constrain(integralP, -40, 40);
    integralR = constrain(integralR, -40, 40);
    p=(pitchdiff * Kp) + (integralP * Ki)-(cpitch*Kd);
    ro=(rolldiff * Kp) + (integralR * Ki)-(croll*Kd);
    Serial.print("actroll = "); Serial.print(actroll); Serial.print("  |  actpitch = "); Serial.print(actpitch); Serial.print("  |  desroll = "); Serial.print(desroll); Serial.print("   |   pitch = "); Serial.println(despitch);
  }
  //rudder vals
  x=pulseIn(lr, HIGH);
  int rd=map(x, low_bound_lr, up_bound_lr, low_bound_rudder, up_bound_rudder);
  rudder.write(rd);
  //elevon mix
  l=90+(pitchcmd*p-rollcmd*ro)*mixconst;
  r=90+(pitchcmd*p+rollcmd*ro)*mixconst;
  //write and repeat
  l=constrain(l, elevon_low_bound, elevon_up_bound);
  r=constrain(r, elevon_low_bound, elevon_up_bound);
  left.write(180-l); right.write(r);
  Serial.print(" = rudder"); Serial.print(rd); Serial.print("  |  l = "); Serial.print(l); Serial.print("  |  r = "); Serial.println(r); 
  delay((int)(dt*1000));
}
