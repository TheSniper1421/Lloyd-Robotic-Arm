int anglepot= 34;
int dirpot= 35;
int a=19;
int b=18;
int motor1Pin1 = 25; 
int motor1Pin2 = 26; 
int pos[3];
int setpos[3];
float kp=15,kd=0.015,ki=0;
long prevt[3];
float eprev[3],einteg[3],deltaT[3],dervative[3];
const int freq = 30000;
const int pwmChannel = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
float pre;


void readencoder1()
{
  int f= digitalRead(b);
  if(f>0)
  pos[0]++;
  else
  pos[0]--;
}

float pid(int i)
{
 float u;
 long currt = micros();
 deltaT[i]= ((float)(currt-prevt[i]))/1.0e6;
 prevt[i]=currt;
 int e=setpos[i]-pos[i];
 dervative[i]= (e-eprev[i])/(deltaT[i]);
 einteg[i] = einteg[i]+ e*deltaT[i];
 eprev[i]=e;
 return u= kp*e+kd*dervative[i]+ki*einteg[i];
}

void setup()
{
 pinMode(anglepot,INPUT);
 pinMode(dirpot,INPUT);
 pinMode(b,INPUT);
 pinMode(motor1Pin1,OUTPUT);
 pinMode(motor1Pin2,OUTPUT);

 ledcSetup(pwmChannel, freq, resolution);
 ledcSetup(pwmChannel2, freq, resolution);
 ledcAttachPin(motor1Pin1, pwmChannel);
 ledcAttachPin(motor1Pin2, pwmChannel2);
 attachInterrupt(digitalPinToInterrupt(a), readencoder1, RISING);
 Serial.begin(115200);
}

void loop() {
int dir;
int dirv=analogRead(dirpot);
//kd=0.01*map(analogRead(dirpot),0,4096,0,100);
setpos[0]=map(analogRead(anglepot),0,4096,-187,187);


float u= pid(0);
int dutyc = constrain(fabs(u),0,255);
if(u<0)
{ledcWrite(pwmChannel2,0);
ledcWrite(pwmChannel,dutyc);}
else
{ledcWrite(pwmChannel,0);
ledcWrite(pwmChannel2,dutyc);
}
long t=millis();
if((t-pre)>20){
Serial.print(kd,6);
Serial.print(" ");
Serial.print(setpos[0]);
Serial.print(" ");
Serial.print(pos[0]);
Serial.print(" ");
Serial.print(240);
Serial.print(", ");
Serial.println(-240);
pre=t;
}


}
