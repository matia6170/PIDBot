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
long currT=millis();
long prevT=0; 
long MOTL_ENC_CNT_PREV = 0;
long MOTR_ENC_CNT_PREV = 0;
float deltaT = 0;

//PID
float eintegral = 0;
float prevE = 0;

/* Physical Properties of the Robot */
const int circumference = 70;
const int ticksPerRev = 400;
const float ticksPerCm = ticksPerRev/circumference;

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

int CMtoTicks(int cm) {
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

void PID(float vt, float rpm, int *pwr, int *dir){
  float kp = 1;
  float ki = 10;
  float kd = 0.05;

  float e = vt-rpm;

  eintegral = eintegral + e * deltaT;

  float u = kp*e + kd*prevE + ki*eintegral;

  prevE = e;

  if (u<0)
    *dir=-1;

  *pwr = (int) fabs(u);
  if(*pwr>255){
    *pwr = 255;
  }
  

}

void setup() {

  initRobot();

  attachInterrupt(digitalPinToInterrupt(ENCR_CLK), ISR_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCL_CLK), ISR_L, RISING);

  

}






void loop() {

  calcRPM();

  // //set a target
  float vt = 100 *(sin(currT/1e6)>0)+250;
  


  int pwr_l = 0;
  int dir_l = 1;
  PID(vt, RPM_L, &pwr_l, &dir_l);
  setMotorL(dir_l, pwr_l);

  // int pwr_r = 0;
  // int dir_r = 1;
  // PID(vt, RPM_R, &pwr_r, &dir_r);
  // setMotorR(dir_r, pwr_r);

  Serial.print("0 ");
  Serial.print("500 ");
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(RPM_L);
  Serial.print(" ");
  Serial.print(pwr_l);


  Serial.println();
  // currT = millis();
  // if(currT - start > 2000){
  //   analogWrite(MOTR_FW, 0);
  //   analogWrite(MOTL_FW, 0);
  // }

}