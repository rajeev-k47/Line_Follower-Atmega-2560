#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define POS_LINE 4
#define RIGHT_TURN 5
#define LEFT_TURN 6
#define L 0
#define NL 1
int STBY = 10; //standby

int AIN1=39;  // Motor A direction pin 1
int AIN2=38 ; // Motor A direction pin 2
int PWMA=3 ;// Motor A PWM speed cint
int BIN1=41  ;// Motor B direction pin 1
int BIN2=42 ;// Motor B direction pin 2
int PWMB=5 ;// Motor B PWM speed control
#define ActiveRun 53 // button for fast run
#define LedPin 52
int mode = 0;
int status;
int pathIndex = 0;
float Kp=35;
float Kd=55;
float Ki=0;
float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;
int sensorPins[] = {25, 24, 23, 22 ,26, 27, 28, 29};
int weights[] = {-3, -2, -1, 0, 0, 1, 2, 3};
char path[100] = "";
unsigned char pathLength = 0;


int sensorValues[8] = {0};
int baseSpeed=130;
int extra_motorSpeed = 90;
int rotateSpeed =80 ;
int extraInch = 300;

int adjGoAndTurn = 500;

bool pass =false;

int sidesensor = 35;

int extra_after_cont = 0;

int scndloop = 0;
int greenLed = 51;

void setup() {

  Serial.begin(9600);
  pinMode(STBY, OUTPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(greenLed,OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  pinMode(sidesensor, INPUT);

  pinMode(ActiveRun, INPUT_PULLUP); 
  mode = STOPPED;
  status=0;
  checkPIDvalues();
  delay(2000);
}  
void loop(){
   Serial.println("Start First Pass");
  readLFSsensors();
  mazeSolve();  
  Serial.println("End First Pass"); 

  while(!pass){
  digitalWrite(LedPin, HIGH); 
//  for(int i =0 ; i<5;i++){
//     Serial.println(path[i]);}
//     Serial.println("**********");
  if(!digitalRead(ActiveRun)){
  digitalWrite(LedPin, LOW); 
    pass=true;
    }
  }
 
  delay(2000);

    Serial.println("Starting 2nd Pass"); 
    pathIndex = 0;
    status = 0;
    if(scndloop==0){mazeOptimization();}
    else{digitalWrite(greenLed, HIGH);}
    Serial.println("End 2nd Pass");
    status =1;
    scndloop=1;
  // mode = STOPPED;U
  // status = 0; // 1st pass
  // pathIndex = 0;
  // pathLength = 0;

}

void readLFSsensors() {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  } 
    if (allZeros(sensorValues)) {
    mode = CONT_LINE;  // All sensor values are 0
 
  } else if (sensorValues[0] == 0 && sensorValues[7] == 1) {
    mode = RIGHT_TURN;  // Right turn
  } else if (sensorValues[0] == 1 && sensorValues[7] == 0) {
    mode = LEFT_TURN;  // Left turn
  } else if (allOnes(sensorValues)) {
    mode = NO_LINE;  // All sensor values are 1
  } else {
      error = calculateError(sensorValues, weights, 8);
    //  Serial.println(error);
        mode=FOLLOWING_LINE;
  }
       

  if(mode == 3){
    extra_after_cont+=1;
  }else{
    extra_after_cont=0;
  }

  Serial.print("Current mode: ");
  Serial.println(mode);
  // Serial.print("Error: ");
  // Serial.println(error);
}
bool allZeros(int arr[]) {
  for (int i = 0; i < 8; i++) {
    if (arr[i] != 0) {
      return false;
    }
  }
  return true;
}

bool allOnes(int arr[]) {
  for (int i = 0; i < 8; i++) {
    if (arr[i] != 1) {
      return false;
    }
  }
  return true;
}

void checkPIDvalues(){
  Serial.print("PID: ");
  Serial.print(Kp);
  Serial.print(" - ");
  Serial.print(Ki);
  Serial.print(" - ");
  Serial.println(Kd);  
  
}
float calculateError(int sensorValues[], int weights[], int numSensors) {
  float numerator = 0;
  float denominator = 0;

  for (int i = 0; i < numSensors; i++) {
    numerator += sensorValues[i] * weights[i];
    denominator += sensorValues[i]==L;
  }

  if (denominator == 0) {
    return 0; // Default to 0 if no sensor detects the line
  }

  return numerator / denominator;
}
void calculatePID(){
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;

  //  Serial.print("P: "); Serial.print(P);
  // Serial.print(" I: "); Serial.print(I);
  // Serial.print(" D: "); Serial.print(D);
  // Serial.print(" PID: "); Serial.println(PIDvalue);
}
void motorPIDcontrol(){
  int leftSpeed = constrain(baseSpeed+PIDvalue, 0, 255);
  int rightSpeed = constrain(baseSpeed-PIDvalue, 0, 255);

  move(1, rightSpeed, 0); //motor 1, full speed, left
  move(2, leftSpeed, 1); //motor 2, full speed, left  
}
void recIntersection(char direction) {
  path[pathLength] = direction;//Store the intersection in the path variable.
  Serial.println(direction);
  pathLength++;
  simplifyPath();  //Simplify the learned path.
}
void simplifyPath() {
  if (pathLength < 3 || path[pathLength - 2] != 'B') {
    return;
  }

  int totalAngle = 0;
  for (int i = 1;i <= 3; i++) {
    switch (path[pathLength - i]) {
      case 'R':
        totalAngle+= 90;
        break;
      case 'L':
        totalAngle+= 270;
        break;
      case 'B':
        totalAngle+= 180;
        break;
    }
  }

  totalAngle =totalAngle % 360;

  switch(totalAngle){
    case 0:
      path[pathLength - 3]= 'S';
      break;
    case 90:
      path[pathLength - 3]= 'R';
      break;
    case 180:
      path[pathLength - 3]= 'B';
      break;
    case 270:
      path[pathLength - 3]= 'L';
      break;
  }

  pathLength-= 2;
}
void mazeSolve() {
  while (!status) {
    readLFSsensors();  
    switch (mode) {   
      case NO_LINE:
        runExtraInch(extra_motorSpeed);
        motorStop();
        recIntersection('B');

        rotate(rotateSpeed,0);//180 turn
        break;
      
      case CONT_LINE: 
        runExtraInch(extra_motorSpeed);
        readLFSsensors();

        if (mode != CONT_LINE) {
          recIntersection('L');  // Assuming "T" or "Cross" goes left

          rotate(rotateSpeed,0);
        } else {
          mazeEnd();
        }
        break;
        
      case RIGHT_TURN: 

        runExtraInch(extra_motorSpeed);
        readLFSsensors();
        if (mode == NO_LINE) {
          recIntersection('R');

          rotate(rotateSpeed,1);
        } else {
          recIntersection('S');
          Serial.println("S");
        }
        break;   
        
      case LEFT_TURN:
        runExtraInch(extra_motorSpeed); 
        // readLFSsensors();
        recIntersection('L');

        rotate(rotateSpeed,0);
        break;   
     
      case FOLLOWING_LINE: 

        followingLine();
        // Serial.println("Following line");
      
        break;      
    }
  }
}
void motorStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);


}
void followingLine(void){
  // move(1, 150, 0); //motor 1, full speed, left
  // move(2, 150, 1); //motor 2, full speed, left
   //readLFSsensors(); 
   calculatePID();
   motorPIDcontrol();
   
}
void runExtraInch(int extra_motorSpeed){
  // motorPIDcontrol();
  move(1, extra_motorSpeed, 0); //motor 1,150 speed, left
  move(2, extra_motorSpeed, 1); //motor 2, 150 speed, left
  delay(extraInch);
  motorStop();
}

void rotate(int motorSpeed,int direction) {
  int st = 1;
  motorPIDcontrol();
  //////// delay(adjGoAndTurn);
  // direction  =0 to turn left and 1 for right
  while(true){
      move(1, motorSpeed, direction); //turn
      move(2, motorSpeed, direction); //turn
  
      readLFSsensors(); 
      if(extra_after_cont>=10){
        mazeEnd();
        break;
      }


      if(allOnes(sensorValues)) {
          st=0;
      }
    if (sensorValues[3] == 0 && sensorValues[4] == 0 && st==0) {
      break; 
   }
}
}
void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 2 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }

}

void mazeEnd(void){
  motorStop();
  for(int i=0;i<pathLength;i++)
  Serial.print(path[i]);
  Serial.print("~pathLenght ==> ");
  Serial.println(pathLength);
  status = 1;
  mode = STOPPED;
}
void mazeOptimization (void){
  while (!status)
  {
    readLFSsensors();  
    switch (mode)
    {
      case FOLLOWING_LINE:
        followingLine();
        break;    
      case CONT_LINE:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn(path[pathIndex]); pathIndex++;}
        break;  
      case LEFT_TURN:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn(path[pathIndex]); pathIndex++;}
        break;  
      case RIGHT_TURN:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn(path[pathIndex]); pathIndex++;}
        break;   
    }    
   }  
}

//-----------------------------------------------------
void mazeTurn (char dir) {
  switch(dir)
  {
    case 'L': // Turn Left

      runExtraInch(extra_motorSpeed); 
      rotate(rotateSpeed,0);      
       break;   
    
    case 'R': // Turn Right
      runExtraInch(extra_motorSpeed); 
      rotate(rotateSpeed,1);     
       break;   
       
    case 'B': // Turn Back
      runExtraInch(extra_motorSpeed); 
       rotate(rotateSpeed,0);     
       break;   
       
    case 'S': // Go Straight
       runExtraInch(extra_motorSpeed);
       followingLine(); 
       break;
  }
}


