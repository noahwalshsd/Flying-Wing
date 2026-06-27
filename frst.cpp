#include <Servo.h>
#define pitch 8
#define roll 7
#define lr 6
Servo left;
Servo right;
Servo rudder;
//rudder vals
const int low_bound_lr=1000;
const int up_bound_lr=2000;
const int low_bound_rudder=70;
const int up_bound_rudder=110;
//elevon vals
int x=90;
int y=90;
void setup() {
  // put your setup code here, to run once:
  pinMode(pitch, INPUT);
  pinMode(roll, INPUT);
  pinMode(lr, INPUT);
  Serial.begin(9600);
  rudder.attach(13);
  left.attach(12);
  right.attach(11);
}

void loop() {
  // put your main code here, to run repeatedly:
  //check values first
  Serial.print(lr);
  Serial.print(" = rudder   |   roll = ");
  Serial.print(roll);
  Serial.print("   |   pitch = ");
  Serial.println(pitch);
  //calculate rudder value
  x=pulseIn(lr, HIGH);
  long y=map(x, low_bound_lr, up_bound_lr, low_bound_rudder, up_bound_rudder);
  rudder.write(y);
  //aeleron vals - need ratio
  int u=pulseIn(pitch, HIGH)-1500;
  int v=pulseIn(roll, HIGH)-1500;
  int l=90+(.65*u-.35*v)*.020;
  int r=90+(.65*u+.35*v)*.020;
  Serial.print(l);
  Serial.print(" = l   |   r = ");
  Serial.print(r);
  left.write(l);
  right.write(r);
}