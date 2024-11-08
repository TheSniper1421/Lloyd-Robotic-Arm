#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

int anglepot0= A0;
int anglepot1=A1;

const float alpha = 0.025;  // Adjust this value to set the cutoff frequency
double filteredOutput[2];
double filteredInput[2];

int a1=3;
int fre=10;
int b1=9;
int a2=2;
int b2=7;
int motor1Pin1 = 5; 
int motor1Pin2 = 6; 
int motor2Pin1=11;
int motor2Pin2=10;
int pos[2];
int setpos[2];
float kp=50,kd=0.00505,ki=0.800;
long prevt[2];
double eprev[2],einteg[2],deltaT[2],dervative[2],pidOutput[2];
float rpm[2],prevTime[2];
float pre;
float calc=0;

void readencoder1()   // Function to read Encoder
{
  int f= digitalRead(b1);
  if(f>0)
  pos[0]++;
  else
  pos[0]--;
}
void readencoder2()   // Function to read Encoder
{
  int f= digitalRead(b2);
  if(f>0)
  pos[1]++;
  else
  pos[1]--;
}

float pid(int i)    // PID Function
{
 
 long currt = micros();
  if (currt - prevt[i] >= fre)
  {
 deltaT[i]= ((double)(currt-prevt[i]))/1.0e6;
 prevt[i]=currt;
 double e=filteredInput[i]-filteredOutput[i];
 dervative[i]= (e-eprev[i])/(deltaT[i]);
 einteg[i] = einteg[i]+ e*deltaT[i];
 eprev[i]=e;
 calc= kp*e+kd*dervative[i]+ki*einteg[i];
  }
return calc;
}

void motorwrite(int channel1,int channel2,int val)    // Function to give motors PWM signal
{
 int dutyc = constrain(fabs(val),0,255);
 if(val<0)
 {analogWrite(channel2,0);
 analogWrite(channel1,dutyc);}
 else
 {analogWrite(channel1,0);
 analogWrite(channel2,dutyc);
 }
}
void ENCODER_RPM(int i)   // Function to calculate RPM
{
  // Calculate RPM every second
  float currentTime = millis();
  if (currentTime - prevTime[i] >= fre)
  {
    // Calculate RPM
    rpm[i] =  (float)pos[i] * (60000.0 / 493.9) / (currentTime - prevTime[i]); //493.9 number of pulses per rotaion
    pos[i] = 0;
    prevTime[i] = currentTime;
    filteredOutput[i]=alpha * rpm[i] + (1 - alpha) * filteredOutput[i];   // Low Pass Filter
  }
}

void setup()
{
 pinMode(anglepot0,INPUT);
 pinMode(anglepot1,INPUT); 
 pinMode(b1,INPUT);
 pinMode(b2,INPUT);
 pinMode(a1,INPUT);
 pinMode(a2,INPUT);
 pinMode(motor1Pin1,OUTPUT);
 pinMode(motor1Pin2,OUTPUT);
 pinMode(motor2Pin1,OUTPUT);
 pinMode(motor2Pin2,OUTPUT);


 attachInterrupt(digitalPinToInterrupt(a1), readencoder1, RISING);
 attachInterrupt(digitalPinToInterrupt(a2), readencoder2, RISING);
 lcd.init();
 lcd.backlight();
 Serial.begin(115200);
}

void loop() {

setpos[0]=map(analogRead(anglepot0),0,1024,-110,110);
setpos[1]=map(analogRead(anglepot1),0,1024,-110,110);
ENCODER_RPM(0);
ENCODER_RPM(1);
filteredInput[0]=alpha * setpos[0] + (1 - alpha) * filteredInput[0];    // Low Pass Filter
filteredInput[1]=alpha * setpos[1] + (1 - alpha) * filteredInput[1];    // Low Pass Filter

pidOutput[0]= pid(0);
pidOutput[1]= pid(1);
motorwrite(motor1Pin1,motor1Pin2,(setpos[0]+pidOutput[0]));
motorwrite(motor2Pin1,motor2Pin2,(setpos[1]+pidOutput[1]));
long t=millis();
if((t-pre)>400){    // Only Writing to Screen once per 400ms

lcd.setCursor(0, 0);
lcd.print("S M1:");
lcd.print((int)filteredInput[0]);
lcd.print("  ");
lcd.setCursor(9, 0);
lcd.print("M2:");
lcd.print((int)filteredInput[1]);
lcd.print("  ");
lcd.setCursor(0, 1);
lcd.print("A M1:");
lcd.print((int)filteredOutput[0]);
lcd.print("  ");
lcd.setCursor(9, 1);
lcd.print("M2:");
lcd.print((int)filteredOutput[1]);
lcd.print("  ");
pre=t;
}


}