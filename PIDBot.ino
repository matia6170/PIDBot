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

unsigned long currT = millis();
unsigned long start = millis();

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
    MOTL_ENC_CNT++;
  } else {
    MOTL_ENC_CNT--;
  }

  deltaTimeL = currentTimeL - lastTimeL;
  lastTimeL = currentTimeL;

}

void initRobot(){
  //Serial.begin(9600);

  
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




void setup() {

  initRobot();

  attachInterrupt(digitalPinToInterrupt(ENCR_CLK), ISR_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCL_CLK), ISR_L, RISING);

  int speed = 100;
  analogWrite(MOTR_FW, speed);
  analogWrite(MOTL_FW, speed);

}

void loop() {
  //Serial.println(deltaTimeR);
  currT = millis();

  if(currT - start > 2000){
    analogWrite(MOTR_FW, 0);
    analogWrite(MOTL_FW, 0);
  }

}