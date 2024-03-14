#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

int anglepot1= 34;
int dirpot= 35;
int a1=19;
int b1=18;
int motor1Pin1 = 25; 
int motor1Pin2 = 26; 
int pos[3];
int setpos[3];
float kp=15,kd=0.015,ki=0;
long prevt[3];
float eprev[3],einteg[3],deltaT[3],dervative[3],pidOutput[3];
const int freq = 30000;
const int pwmChannel = 0;
const int pwmChannel2 = 1;
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
 pinMode(anglepot1,INPUT);
 pinMode(dirpot,INPUT);
 pinMode(b1,INPUT);
 pinMode(motor1Pin1,OUTPUT);
 pinMode(motor1Pin2,OUTPUT);

 ledcSetup(pwmChannel, freq, resolution);
 ledcSetup(pwmChannel2, freq, resolution);
 ledcAttachPin(motor1Pin1, pwmChannel);
 ledcAttachPin(motor1Pin2, pwmChannel2);
 attachInterrupt(digitalPinToInterrupt(a1), readencoder1, RISING);
 lcd.init();
 lcd.backlight();
 Serial.begin(115200);
}

void loop() {
int dir;
int dirv=analogRead(dirpot);
//kd=0.01*map(analogRead(dirpot),0,4096,0,100);
setpos[0]=map(analogRead(anglepot1),0,4096,-187,187);


pidOutput[0]= pid(0);
motorwrite(pwmChannel,pwmChannel2,pidOutput[0]);
/*int dutyc = constrain(fabs(u),0,255);
if(u<0)
{ledcWrite(pwmChannel2,0);
ledcWrite(pwmChannel,dutyc);}
else
{ledcWrite(pwmChannel,0);
ledcWrite(pwmChannel2,dutyc);
}*/
long t=millis();
if((t-pre)>400){
Serial.print(kd,6);
Serial.print(" ");
Serial.print(setpos[0]);
Serial.print(" ");
Serial.print(pos[0]);
Serial.print(" ");
Serial.print(240);
Serial.print(", ");
Serial.println(-240);
lcd.setCursor(0, 0);
lcd.print("Motor1:");
lcd.print(map(pos[0],-187,187,-135,135));
lcd.print("  ");
pre=t;
}


}
