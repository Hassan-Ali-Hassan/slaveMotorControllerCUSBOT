#include<Wire.h>
#define SIZE 8
#define SIZEAUX 3
#define SLAVEADDRESS 5
#define MOTORADDRESS 2
#define DRIVER 6
#define IN1 5
int led = 13;
byte message[2] = {0,0};
unsigned long counter = 0;
unsigned long time = 0;
unsigned long oldTime = 0;
unsigned long timeMain = 0;
unsigned long oldTimeMain = 0;
unsigned int uRPM = 0;
float RPM = 0;
float filteredRPM = 0;
float speedRequired = 0;
int filterIndex = 0;
float Ts = 10; //Sampling time in milli seconds
char a = 0;
bool state = false;
bool sendFeed = false;
bool motorON = false;
bool motorOFF = true;

void setup() {
  Serial.begin(115200);
  Wire.begin(SLAVEADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  attachInterrupt(0,disp,FALLING);
  pinMode(led,OUTPUT);
  pinMode(DRIVER,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(7,OUTPUT);
  digitalWrite(IN1,HIGH);
  digitalWrite(7,LOW);
//  Serial.println('a');
//  while(a != 'a')
//  {
//    //wait for a specific character from pc
//    a = Serial.read();
//  }
  
}

void loop() 
{  
  if(Serial.available())
  {
    a = Serial.read();
    if(a == 'r')
    {
      Serial.println(uRPM);
    }
    else if(a =='w')
    {
      speedRequired = 250;
    }
    else if(a =='s')
    {
      speedRequired = 200;
    }
    else if(a =='d')
    {
      speedRequired = 100;
    }
    else if(a =='f')
    {
      speedRequired = 150;
    }
  }
  timeMain = millis();
  
  //rejecting erratic readings
  
  if((timeMain - oldTimeMain) >= Ts)
  {
    filteredRPM = rejectOutlier2(RPM);
//    filteredRPM = filterButter(RPM);
//    filteredRPM = filterFIR(RPM);
//    analogWrite(MOTORADDRESS,255);
    oldTimeMain = timeMain;
    
//    MIT(filteredRPM,speedRequired);

    Serial.print(timeMain);
    Serial.print("\t");
    Serial.print(filteredRPM);
    Serial.print("\t");
    Serial.print((int)RPM); 
    Serial.print("\t");
    PID(filteredRPM,speedRequired);
//    delay(1);
  }
}

void requestEvent()
{
  //converting the RPM value into two bytes describing the integer part
  breakDownRPM();
  Wire.write(message,2);
}

void receiveEvent(int bytes)
{
  int i = 0;
  byte message[2];
  
  while(Wire.available())
  {
    message[i] = Wire.read();
    delay(10);
    i++;
  }  
  speedRequired = (float)(message[0] + message[1] * 10);
}

void breakDownRPM()
{
  message[1] = (int)filteredRPM / 10;
  message[0] = (int)(filteredRPM - message[1] * 10);
  //  Serial.print(message[0]);
  //  Serial.print("\t");
  //  Serial.print(message[1]);
  //  Serial.print("\t");
  //  Serial.println(RPM);
}

void disp()
{
  time = micros();
  RPM = 60/((float)(time - oldTime) *0.000016);  
//  RPM = 60/((float)(time - oldTime) *0.000020);
  state = !state;
  digitalWrite(led,state);
  oldTime = time;
}


void PID(float rpm, float requiredRPM)
{
  float Kp = 5;
  float Ki = 0.1;
  float Kd = 0.15;
  float error = 0;
  static float errorIntegral[2]= {0,0};
  float errorDifferential = 0;
  static float errorOld = 0;
  int PWMvalue = 0;
  float timePID = (float)millis()/1000;
  static float oldTimePID = 0;
  float dt = timePID - oldTimePID;
  boolean gainSchedulingOn = false;
  float voltage = 0;
  
  // P part
  error = (requiredRPM - rpm);
  // I part
  if(motorON)
  {
    errorIntegral[1] = errorIntegral[0] + (errorOld + error) * 0.5 * dt; //watch out!! when this term increases too much, it becomes negative and the motor suddenly stops and becomes sluggish
    errorIntegral[0] = errorIntegral[1];
  }
  else
  {
//    errorIntegral[1] += 0;
    errorIntegral[1] = 0;
    errorIntegral[0] = 0;
  }
  // D part
  errorDifferential = (error - errorOld)/dt; 
  errorOld = error;

  // Using feedback from battery level to adjust the controller gains
  if(gainSchedulingOn)
  {
    voltage = (float)analogRead(A0) * 5.0 / 1023.0 * 3;
    Kp = 15.0882 - 1.323*voltage;
    Ki = 10.4117 - 1.176*voltage;
  }
  else
  {
  }
  
  // PID part
//  PWMvalue = Kp * error + Ki * errorIntegral[1] + Kd * errorDifferential;
  if(abs(error) < 20)
  {
    PWMvalue = Kp * error + Ki * errorIntegral[1] + Kd * errorDifferential;
  }
  else
  {
    PWMvalue = Kp * error + Ki * errorIntegral[1] + Kd * errorDifferential;
  }

  Serial.print(error);
  Serial.print("\t");
  Serial.print(PWMvalue);
  Serial.println("\t");
  if(PWMvalue > 255){PWMvalue = 255;}
  else if(PWMvalue < 0){PWMvalue = 0;}
//  PWMvalue = 55;
  analogWrite(5,PWMvalue);
  
  oldTimePID = timePID;
}

float filterButter(float rpm) //the value supplied to this function should be the rpm value after ommitting the erronous values 
{
  int i = 0;
  static float oldFiltered[3] = {0,0,0}; //we are using only oldFiltered[1] and oldFiltered[2], with 2 being the older value
  static float rpmOld[3] = {0,0,0};
  float filteredRPM = 0;
  rpm = rejectOutlier2(rpm);
//  checkStop(rpm);
  if(motorOFF)
  {
//    return 0;
  }
  else
  {
//    filteredRPM = 1.454*oldFiltered[1]-0.5741*oldFiltered[2]+0.02995*rpm + 0.0599 * rpmOld[1] + 0.02995*rpmOld[2];
//    oldFiltered[2] = oldFiltered[1];
//    oldFiltered[1] = filteredRPM;
//    rpmOld[2] = rpmOld[1];
//    rpmOld[1] = rpm;

//     filteredRPM = 0.1659*(rpm + rpmOld[1]) + 0.6682 * oldFiltered[1];
    filteredRPM = 0.1871*(rpm + rpmOld[1]) + 0.6258* oldFiltered[1]; //9hz
//    filteredRPM = 0.2841*(rpm + rpmOld[1]) + 0.4318* oldFiltered[1]; //15hz
//     filteredRPM = 0.09163*(rpm + rpmOld[1]) + 0.8167 * oldFiltered[1]; //4hz
    rpmOld[1] = rpm;
    oldFiltered[1] = filteredRPM;  
    return filteredRPM;
  }
}

void checkStop(float rpm)
{
  static float rpm_old = 0;
  static float lastRPM = 0;
  static int repetitionCounter = 0;
  if(rpm == rpm_old)
  {
    repetitionCounter++;
    if(repetitionCounter > 10) //this means the motor has stopped
    {
      motorON = false;
      motorOFF = true;
      RPM = 0;
      filteredRPM = 0;
      lastRPM = rpm;
      repetitionCounter = 0;
//      Serial.println("the motor has stopped");
    }  
  }
  else
  {
    repetitionCounter--;
  }
  repetitionCounter = constrain(repetitionCounter,0,15);
  rpm_old = rpm;
  
  if(motorOFF)
  {
    if(rpm != lastRPM) //motion happens
    {
      motorON = true;
      motorOFF = false;
//      Serial.println("the motor has started again!");
    }
  }
}



float rejectOutlier2(float rpm)
{
  static float readings[SIZE];
  static float auxReadings[SIZEAUX];
  float sortedReadings[SIZE];
  static int counter = 0;
  static int auxCounter = 0;
  static bool allow = false;
  static bool sorted = false;
  float Q1 = 0;
  float Q2 = 0;
  float upperBound = 0;
  float lowerBound = 0;
  int i = 0;
  float k = 1.5;
  checkStop(rpm);
  if(motorON)
  {
    if(counter < SIZE)
    {
      readings[counter] = rpm;
      counter++;
    }
    else
    {
      // adding the new RPM to the queue of numbers we have
      for(i = 0; i < SIZE - 1; i++)
      {
        readings[i] = readings[i+1];
        sortedReadings[i] = readings[i];
      }
      readings[SIZE-1] = rpm;
      sortedReadings[SIZE-1] = rpm;
      
      // Now sorting the readings
      bubble_sort(sortedReadings);
//      for(i = 0; i < SIZE; i++)
//      {
//        Serial.print(sortedReadings[i]);
//        Serial.print("\t");
//      }
//      Serial.println();
//      for(i = 0; i < SIZE; i++)
//      {
//        Serial.print(readings[i]);
//        Serial.print("\t");
//      }
//      Serial.println();
      // Checking the interquartile difference
      Q1 = sortedReadings[1];
      Q2 = sortedReadings[6];
      upperBound = Q2 + (Q2 - Q1) * 1.5;
      lowerBound = Q1 - (Q2 - Q1) * 1.5;
//      Serial.print("upper and lower");
//      Serial.print("\t"); 
//      Serial.print(Q2 - Q1);
//      Serial.print("\t");
//      Serial.print(Q1);
//      Serial.print("\t");
//      Serial.print(Q2);
//      Serial.print("\t");
//      Serial.print(upperBound);
//      Serial.print("\t"); 
//      Serial.println(lowerBound);
      if((rpm > upperBound || rpm < lowerBound) && !allow) //in this case the rpm value  is an outlier
      {        
        readings[SIZE-1] = readings[0]; // overwritting the last rpm value and make it equal to another acceptable value
        auxReadings[auxCounter++] = rpm;
//        Serial.println(readings[SIZE-1]);
      }
      else
      {
        auxCounter = 0;
        allow = false;
      }
      if(auxCounter >= SIZEAUX) //which means we have a sequence of other readings out of box and consistent
      {
        for(i = 0; i < SIZEAUX; i++)
        {
          readings[SIZE-SIZEAUX+i] = auxReadings[i];
        }
        auxCounter = 0;
        allow = true;
      }  
    }
  }
  return readings[SIZE-1];
}

float mean(float* a,int n)
{
  int i = 0;
  float sum = 0;
  for(i = 0; i < n; i++)
  {
    sum += a[i];
  }
  sum /= (float)n;
  return sum;
}
void bubble_sort(float* a)
{
  int j = 0;
  int i = 0;
  for(j = 0; j < SIZE; j++)
  {
    for(i = 0; i < SIZE-j-1; i++)
    {
      if(a[i]>a[i+1])
      {
        swap(a[i],a[i+1]);
      }
    }
  }
}

void swap(float&a, float&b)
{
  float temp = a;
  a = b;
  b = temp;
}

