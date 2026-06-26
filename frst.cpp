#define rudder 13
#define left 12
#define right 11
#define pitch 8
#define roll 7
#define lr 6
//rudder vals
const int low_bound_lr=0;
const int up_bound_lr=100;
const int low_bound_rudder=70;
const int up_bound_rudder=110;
//aeleron vals
int x=90;
int y=90;
void setup() {
  // put your setup code here, to run once:
  pinMode(rudder, OUTPUT);
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
  pinMode(pitch, INPUT);
  pinMode(roll, INPUT);
  pinMode(lr, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //check values first
  Serial.print(rudder);
  Serial.print(" = rudder   |   left = ");
  Serial.print(left);
  Serial.print("   |   right = ");
  Serial.println(right);
  //calculate rudder value
  x=analogRead(lr);
  long y=(x, low_bound_lr, up_bound_lr, low_bound_rudder, up_bound_rudder);
  analogWrite(rudder, y);
  //aeleron vals - need ratio
  int u=analogRead(pitch)-50;
  int v=analogRead(roll)-50;
  int l=90+(.65*u-.35*v)*.020;
  int r=90+(.65*u+.35*v)*.020;
  Serial.print(l);
  Serial.print(" = l   |   r = ");
  Serial.print(r);
  analogWrite(l, left);
  analogWrite(r, right);
}