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

//This number will overflow after approximately 70 minutes
volatile unsigned long currentTimeR = 0;
volatile unsigned long lastTimeR = 0;
volatile unsigned long deltaTimeR = 0;

volatile unsigned long currentTimeL = 0;
volatile unsigned long lastTimeL = 0;
volatile unsigned long deltaTimeL = 0;



/* Physical Properties of the Robot */
const int circumference = 70;
const int ticksPerRev = 400;
const float ticksPerCm = ticksPerRev/circumference;

void ISR_R() {
  currentTimeR = micros();


  if (digitalRead(ENCR_CLK) == digitalRead(ENCR_Dt)) {
    MOTR_ENC_CNT++;
  } else {
    MOTR_ENC_CNT--;
  }

  deltaTimeR = currentTimeR - lastTimeR;
  lastTimeR = currentTimeR;

}

void ISR_L() {
  currentTimeL = micros();


  if (digitalRead(ENCL_CLK) == digitalRead(ENCL_Dt)) {
    MOTL_ENC_CNT--;
  } else {
    MOTL_ENC_CNT++;
  }

  deltaTimeL = currentTimeL - lastTimeL;
  lastTimeL = currentTimeL;

}

void initRobot(){
  Serial.begin(9600);

  
  lastTimeR = micros();
  currentTimeR = micros();
  lastTimeL = micros();
  currentTimeL = micros();

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

void setMotorL(int dir, int speed){

  if(dir == 1){ 
    // go foward
    analogWrite(MOTL_BK, 0);
    analogWrite(MOTL_FW, speed);
  }
  else if(dir == -1){
    // go back
    analogWrite(MOTL_FW, 0);
    analogWrite(MOTL_BK, speed);
  }
  else{
    // Or dont turn
    analogWrite(MOTL_FW, 0);
    analogWrite(MOTL_BK, 0);
  }

}

int CMtoTicks(int cm) {
  return ticksPerCm * cm;
  
}






void setup() {

  initRobot();

  attachInterrupt(digitalPinToInterrupt(ENCR_CLK), ISR_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCL_CLK), ISR_L, RISING);

  int speed = 255;
  //analogWrite(MOTR_FW, speed);
  analogWrite(MOTL_FW, speed);

}


long prevT=0; 
long posPrev = 0;
float eintegral = 0;
float prevE = 0;

void loop() {

  //analogWrite(MOTL_FW, 100/3.0*micros()/1.0e6);

  
  



  long currT = micros();
  float deltaT = ((float) (currT-prevT)/1.0e6);
  float vel1 = (MOTL_ENC_CNT - posPrev)/deltaT;
  vel1 = vel1/400*60;
  posPrev = MOTL_ENC_CNT;
  prevT = currT;

  //set a target
  float vt = 50 *(sin(currT/1e6)>0)+250;
  
  float kp = 1;
  float ki = 10;
  float kd = 0.05;
  float e = vt-vel1;
  eintegral = eintegral + e * deltaT;

  float u = kp*e + kd*prevE + ki*eintegral;

  prevE = e;


  
  int dir = 1;
  if (u<0)
    dir=-1;

  int pwr = (int) fabs(u);
  if(pwr>255){
    pwr = 255;
  }
  setMotorL(dir, pwr);


  Serial.print("0 ");
  Serial.print("500 ");
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(vel1);
  Serial.print(" ");
  Serial.print(dir*10);
  Serial.println();
  // currT = millis();
  // if(currT - start > 2000){
  //   analogWrite(MOTR_FW, 0);
  //   analogWrite(MOTL_FW, 0);
  // }

}