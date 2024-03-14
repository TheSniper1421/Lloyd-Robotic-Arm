int anglepot= 34;
int dirpot= 35;
int a=19;
int b=18;
int motor1Pin1 = 25; 
int motor1Pin2 = 26; 
int pos=0;
float kp=15,kd=0.015,ki=0;
long prevt=0;
float eprev=0,einteg=0;
const int freq = 30000;
const int pwmChannel = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
float pre;


void readencoder()
{
  int f= digitalRead(b);
  if(f>0)
  pos++;
  else
  pos--;
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
 attachInterrupt(digitalPinToInterrupt(a), readencoder, RISING);
 Serial.begin(115200);
}

void loop() {
int dir;
int dirv=analogRead(dirpot);
//kd=0.01*map(analogRead(dirpot),0,4096,0,100);
int setpos=map(analogRead(anglepot),0,4096,-187,187);

/*if(dirv<2000)
dir=-1;
else if(dirv>2500)
dir=1;
else
dir=0;*/

/*Serial.print(dir);
Serial.print("  ");
Serial.print(dirv);
Serial.print(" ");
Serial.print(mspeed);
Serial.print(" ");
Serial.println(pos);*/

/*if(dir==1)
{ledcWrite(pwmChannel,0);
ledcWrite(pwmChannel2,mspeed);}
else if (dir==-1)
{
ledcWrite(pwmChannel2,0);
ledcWrite(pwmChannel,mspeed);
}
else if (dir==0)
{ledcWrite(pwmChannel,0);
ledcWrite(pwmChannel2,0);}*/
long currt = micros();
float deltaT= ((float)(currt-prevt))/1.0e6;
prevt=currt;
int e=setpos-pos;
float dervative= (e-eprev)/(deltaT);
einteg = einteg+ e*deltaT;
eprev=e;
float u= kp*e+kd*dervative+ki*einteg;
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
Serial.print(setpos);
Serial.print(" ");
Serial.print(pos);
Serial.print(" ");
Serial.print(240);
Serial.print(", ");
Serial.println(-240);
pre=t;
}


}
