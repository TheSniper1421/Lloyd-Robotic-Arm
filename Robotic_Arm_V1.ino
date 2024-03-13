int speedpot= 34;
int dirpot= 35;
int a=19;
int b=18;
int motor1Pin1 = 25; 
int motor1Pin2 = 26; 
int pos=0;

const int freq = 30000;
const int pwmChannel = 0;
const int pwmChannel2 = 1;
const int resolution = 8;


void setup() {

pinMode(speedpot,INPUT);
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
void readencoder()
{
  int f= digitalRead(b);
  if(f>0)
  pos++;
  else
  pos--;
}
void loop() {
int dir;
int dirv=analogRead(dirpot);

int mspeed=map(analogRead(speedpot),0,4096,0,255);

if(dirv<2000)
dir=-1;
else if(dirv>2500)
dir=1;
else
dir=0;

Serial.print(dir);
Serial.print("  ");
Serial.print(dirv);
Serial.print(" ");
Serial.print(mspeed);
Serial.print(" ");
Serial.println(pos);
delay(50);

if(dir==1)
{ledcWrite(pwmChannel,0);
ledcWrite(pwmChannel2,mspeed);}
else if (dir==-1)
{
ledcWrite(pwmChannel2,0);
ledcWrite(pwmChannel,mspeed);
}
else if (dir==0)
{ledcWrite(pwmChannel,0);
ledcWrite(pwmChannel2,0);}

}
