#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

int anglepot0= 34;
int anglepot1=35;
int anglepot2=32;

int a1=19;
int b1=18;
int a2=5;
int b2=4;
int a3=36;
int b3=39;
int motor1Pin1 = 14; 
int motor1Pin2 = 12; 
int motor2Pin1=33;
int motor2Pin2=25;
int motor3Pin1=26;
int motor3Pin2=27;
int pos[3];
int setpos[3];
float kp=15,kd=0.010,ki=0;
long prevt[3];
float eprev[3],einteg[3],deltaT[3],dervative[3],pidOutput[3];
const int freq = 30000;
const int pwmChannel_11 = 0;
const int pwmChannel_12 = 1;
const int pwmChannel_21 = 2;
const int pwmChannel_22 = 3;
const int pwmChannel_31 = 4;
const int pwmChannel_32 = 5;
const int resolution = 8;
float pre;


void readencoder1()
{
  int f= digitalRead(b1);
  if(f>0)
  pos[0]++;
  else
  pos[0]--;
}
void readencoder2()
{
  int f= digitalRead(b2);
  if(f>0)
  pos[1]++;
  else
  pos[1]--;
}
void readencoder3()
{
  int f= digitalRead(b3);
  if(f>0)
  pos[2]++;
  else
  pos[2]--;
}

float pid(int i)
{
 float calc;
 long currt = micros();
 deltaT[i]= ((float)(currt-prevt[i]))/1.0e6;
 prevt[i]=currt;
 int e=setpos[i]-pos[i];
 dervative[i]= (e-eprev[i])/(deltaT[i]);
 einteg[i] = einteg[i]+ e*deltaT[i];
 eprev[i]=e;
 return calc= kp*e+kd*dervative[i]+ki*einteg[i];
}

void motorwrite(int channel1,int channel2,int val)
{
 int dutyc = constrain(fabs(val),0,255);
 if(val<0)
 {ledcWrite(channel2,0);
 ledcWrite(channel1,dutyc);}
 else
 {ledcWrite(channel1,0);
 ledcWrite(channel2,dutyc);
 }
}

void setup()
{
 pinMode(anglepot0,INPUT);
 pinMode(anglepot1,INPUT); 
 pinMode(anglepot2,INPUT); 
 pinMode(b1,INPUT);
 pinMode(b2,INPUT);
 pinMode(b3,INPUT);
 pinMode(motor1Pin1,OUTPUT);
 pinMode(motor1Pin2,OUTPUT);
 pinMode(motor2Pin1,OUTPUT);
 pinMode(motor2Pin2,OUTPUT);
 pinMode(motor3Pin1,OUTPUT);
 pinMode(motor3Pin2,OUTPUT);

 ledcSetup(pwmChannel_11, freq, resolution);
 ledcSetup(pwmChannel_12, freq, resolution);
 ledcSetup(pwmChannel_21, freq, resolution);
 ledcSetup(pwmChannel_22, freq, resolution);
 ledcSetup(pwmChannel_31, freq, resolution);
 ledcSetup(pwmChannel_32, freq, resolution);

 ledcAttachPin(motor1Pin1, pwmChannel_11);
 ledcAttachPin(motor1Pin2, pwmChannel_12);
 ledcAttachPin(motor2Pin1, pwmChannel_21);
 ledcAttachPin(motor2Pin2, pwmChannel_22);
 ledcAttachPin(motor3Pin1, pwmChannel_31);
 ledcAttachPin(motor3Pin2, pwmChannel_32);

 attachInterrupt(digitalPinToInterrupt(a1), readencoder1, RISING);
 attachInterrupt(digitalPinToInterrupt(a2), readencoder2, RISING);
 attachInterrupt(digitalPinToInterrupt(a3), readencoder3, RISING);
 lcd.init();
 lcd.backlight();
 Serial.begin(115200);
}

void loop() {

setpos[0]=map(analogRead(anglepot0),0,4096,-187,187);
setpos[1]=map(analogRead(anglepot1),0,4096,-187,187);
setpos[2]=map(analogRead(anglepot2),0,4096,-187,187);
pidOutput[0]= pid(0);
pidOutput[1]= pid(1);
pidOutput[2]= pid(2);
motorwrite(pwmChannel_11,pwmChannel_12,pidOutput[0]);
motorwrite(pwmChannel_21,pwmChannel_22,pidOutput[1]);
motorwrite(pwmChannel_31,pwmChannel_32,pidOutput[2]);


long t=millis();
if((t-pre)>400){

lcd.setCursor(0, 0);
lcd.print("M1:");
lcd.print(map(pos[0],-187,187,-135,135));
lcd.print("  ");
lcd.print("M2:");
lcd.print(map(pos[1],-187,187,-135,135));
lcd.print("  ");
lcd.setCursor(0, 1);
lcd.print("M3:");
lcd.print(map(pos[2],-187,187,-135,135));
lcd.print("  ");
pre=t;
}


}
