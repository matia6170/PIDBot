/*
MIT License

Copyright July 2023 Hyunwoo Choi

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
//MAX RPM 250

const int ENCR_CLK = 2;
const int ENCR_Dt = 4;
const int ENCL_CLK = 3;
const int ENCL_Dt = 7;

const int MOTR_FW = 10;
const int MOTR_BK = 9;
const int MOTL_FW = 5;
const int MOTL_BK = 6;

volatile long MOTR_ENC_CNT = 0;
volatile long MOTL_ENC_CNT = 0;

//MOTOR SPEED
float RPM_R = 0;
float RPM_L = 0;

//Motor RPM calculation related
long currT=micros();
long prevT=0; 
long MOTL_ENC_CNT_PREV = 0;
long MOTR_ENC_CNT_PREV = 0;
float deltaT = 0;

//PID
float eintegral_L = 0;
float prevE_L = 0;
float eintegral_R = 0;
float prevE_R = 0;

//cascading
float eintegral_DL = 0;
float prevE_DL = 0;
float eintegral_DR = 0;
float prevE_DR = 0;

/* Physical Properties of the Robot */
const int circumference = 2*3.14*3.5;
const int ticksPerRev = 400;
const float ticksPerCm = ticksPerRev/circumference;

long startTimer = micros();

void ISR_R() {
  if (digitalRead(ENCR_CLK) == digitalRead(ENCR_Dt)) {
    MOTR_ENC_CNT++;
  } else {
    MOTR_ENC_CNT--;
  }
}

void ISR_L() {
  if (digitalRead(ENCL_CLK) == digitalRead(ENCL_Dt)) {
    MOTL_ENC_CNT--;
  } else {
    MOTL_ENC_CNT++;
  }
}

void initRobot(){
  Serial.begin(9600);

  //motorpins
  pinMode(MOTR_FW, OUTPUT);
  pinMode(MOTR_BK, OUTPUT);
  pinMode(MOTL_FW, OUTPUT);
  pinMode(MOTL_BK, OUTPUT);

  //encoders
  pinMode(ENCR_CLK,INPUT);
  pinMode(ENCR_Dt,INPUT);
  pinMode(ENCL_CLK,INPUT);
  pinMode(ENCL_Dt,INPUT);
  
}

void setMotorR(int dir, int speed){
  switch(dir){
    case 1:
      analogWrite(MOTR_BK, 0);
      analogWrite(MOTR_FW, speed);
      break;
    case -1:
      analogWrite(MOTR_FW, 0);
      analogWrite(MOTR_BK, speed);
      break;
    default:
      analogWrite(MOTR_FW, 0);
      analogWrite(MOTR_BK, 0);
      break;
  }
}
void setMotorL(int dir, int speed){
  switch(dir){
    case 1:
      analogWrite(MOTL_BK, 0);
      analogWrite(MOTL_FW, speed);
      break;
    case -1:
      analogWrite(MOTL_FW, 0);
      analogWrite(MOTL_BK, speed);
      break;
    default:
      analogWrite(MOTL_FW, 0);
      analogWrite(MOTL_BK, 0);
      break;
  }
}
void stop(){
    setMotorR(0,0);
    setMotorL(0,0);
}

int CMToTicks(int cm) {
  return ticksPerCm * cm;
}


void calcRPM(){
  //analogWrite(MOTL_FW, 100/3.0*micros()/1.0e6);
  currT = micros();
  deltaT = ((float) (currT-prevT)/1.0e6);

 //Calculate Right Motor RPM
  float vel1 = (MOTR_ENC_CNT - MOTR_ENC_CNT_PREV)/deltaT;
  RPM_R = vel1/400*60;
  MOTR_ENC_CNT_PREV = MOTR_ENC_CNT;

  //Calculate Left Motor RPM
  float vel2 = (MOTL_ENC_CNT - MOTL_ENC_CNT_PREV)/deltaT;
  RPM_L = vel2/400*60;
  MOTL_ENC_CNT_PREV = MOTL_ENC_CNT;
  
  prevT = currT;
}

void PID_L(float vt, float rpm, int *pwr, int *dir){
  float kp = 1;
  float ki = 10;
  float kd = 0.05;

  float e = vt-rpm;

  eintegral_L = eintegral_L + e * deltaT;

  float u = kp*e + kd*prevE_L + ki*eintegral_L;

  prevE_L = e;

  if (u<0)
    *dir=-1;

  *pwr = (int) fabs(u);
  if(*pwr>255)
    *pwr = 255;
  
}
void PID_R(float vt, float rpm, int *pwr, int *dir){
  float kp = 2;
  float ki = 10;
  float kd = 0.01;

  float e = vt-rpm;

  eintegral_R = eintegral_R + e * deltaT;

  float eDeriv = ((abs(prevE_R)-abs(e))/deltaT);

  float u = kp*e + kd*eDeriv + ki*eintegral_R;

  prevE_R = e;

  if (u<0)
    *dir=-1;

  *pwr = (int) fabs(u);
  if(*pwr>255)
    *pwr = 255;
  
}

float rpmLimit=100;
void PID_DL(float target, float *rpmOut){
  float kp = 0.7;
  float ki = 0.001;
  float kd = 0.01;

  float e = target-MOTL_ENC_CNT;

  eintegral_DL = eintegral_DL + e * deltaT;

  float eDeriv = ((abs(e)-abs(prevE_DL))/deltaT);

  float u = kp*e + kd*eDeriv + ki*eintegral_DL;

  prevE_DL = e;

  *rpmOut = u;
  if(*rpmOut>rpmLimit)
    *rpmOut = rpmLimit;
  else if(*rpmOut<-rpmLimit)
    *rpmOut=-rpmLimit;
  
}
void PID_DR(float target, float *rpmOut){
  float kp = 0.7;
  float ki = 0.001;
  float kd = 0.01;

  float e = target-MOTR_ENC_CNT;

  eintegral_DR = eintegral_DR + e * deltaT;

  float eDeriv = ((abs(e)-abs(prevE_DR))/deltaT);

  float u = kp*e + kd*eDeriv + ki*eintegral_DR;

  prevE_DR = e;

  *rpmOut = u;
  if(*rpmOut>rpmLimit)
    *rpmOut = rpmLimit;
  else if(*rpmOut<-rpmLimit)
    *rpmOut=-rpmLimit;
  
}

void setup() {

  initRobot();

  attachInterrupt(digitalPinToInterrupt(ENCR_CLK), ISR_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCL_CLK), ISR_L, RISING);

}

long CURRENT_TIME = millis();
long PAST_TIME = millis();

float target_rpm_L=0;
float target_rpm_R=0;

float targetDist = CMToTicks(50);
void loop() {

  CURRENT_TIME=millis();
  long DT;

  
  if ((currT - startTimer) < 10*1000000){
    
    calcRPM();

    // //set a target
    //vt = 200 *(sin(currT/1e6)>0)-100;
    //vt=150*sin(currT/1e6);
    
    int pwr_l = 0;
    int dir_l = 1;
    PID_L(target_rpm_L, RPM_L, &pwr_l, &dir_l);
    setMotorL(dir_l, pwr_l);

    int pwr_r = 0;
    int dir_r = 1;
    PID_R(target_rpm_R, RPM_R , &pwr_r, &dir_r);
    setMotorR(dir_r, pwr_r);


    //Cascading PID
    DT=CURRENT_TIME-PAST_TIME;
    if(DT>100){

      PID_DR(targetDist, &target_rpm_R);
      PID_DL(targetDist, &target_rpm_L);


      PAST_TIME=CURRENT_TIME;

    }

  }else{
    stop();
  }


  // Serial.print("dealtaT: ");
  // Serial.print(DT);


  // Serial.print("2100 1500 ");
  // Serial.print(vt);
  // Serial.print(" ");
  Serial.print(targetDist);
  Serial.print(" ");
  Serial.print(MOTL_ENC_CNT);
  Serial.print(" ");
  Serial.print(MOTR_ENC_CNT);
  Serial.println("");



  


}