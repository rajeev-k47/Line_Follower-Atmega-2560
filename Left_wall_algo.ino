#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2
#define CONT_LINE 3
#define POS_LINE 4
#define RIGHT_TURN 5
#define LEFT_TURN 6
int STBY = 10; //standby
int Light = 52;

int AIN1=39;  // Motor A direction pin 1
int AIN2=38 ; // Motor A direction pin 2
int PWMA=3   ;// Motor A PWM speed cint
int BIN1=41  ;// Motor B direction pin 1
int BIN2=42  ;// Motor B direction pin 2
int PWMB=5   ;// Motor B PWM speed control
int mode = 0;
int sensorPins[] = {25, 24, 23, 22,26, 27, 28, 29};
unsigned char dir;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         

// The path variable will store the p  ath that the robot has taken:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)

char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
unsigned int status = 0; // solving = 0; reach end = 1
int sensorValues[8] = {0};
// int back_motorSpeed = 150;
int extra_motorSpeed = 115;
int rotate_motorSpeed = 80;
int motor_Speed = 150;
int extraInch = 300;
int Uturn = 1050;
// int rotateLeft = 525;
// int rotateRight = 525;
#define ActiveRun 53
#define LedPin 52
bool pass =false;


void setup() {
  
  Serial.begin(9600);

  pinMode(STBY, OUTPUT);
  pinMode(LedPin, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(ActiveRun, INPUT_PULLUP);
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  mode = STOPPED;
  status=0;
  delay(2000);
  // adjust();
  // runExtraInch(115);
  // rotate(70,1);
  // runExtraInch(155);

  // rotate(150,0);
}

void loop() {
 Serial.println("Start First Pass");
  readLFSsensors();
  mazeSolve();  
  Serial.println("End First Pass"); 

  while(!pass){
  digitalWrite(LedPin, HIGH); 
  if(!digitalRead(ActiveRun)){
  digitalWrite(LedPin, LOW); 
    pass=true;
  }
  Serial.println("hit");
 
  }
  
  Serial.println("Starting 2nd Pass"); 
  pathIndex = 0;
  status = 0;
  Serial.println("End 2nd Pass"); 

  mode = STOPPED;
  status = 0;  // 1st pass
  pathIndex = 0;
  pathLength = 0;


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
    mode = FOLLOWING_LINE;  // Default case, following the line
  }

  Serial.print("Current mode: ");
  Serial.println(mode);
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

void mazeSolve() {
  while (!status) {
    readLFSsensors();  
    switch (mode) {   
      case NO_LINE:  
        motorStop();
        rotate(rotate_motorSpeed,1);//180
        recIntersection('B');
        break;
      
      case CONT_LINE: 
        runExtraInch(extra_motorSpeed);
        readLFSsensors();
        if (mode != CONT_LINE) {
          rotate(rotate_motorSpeed,0);
          recIntersection('L');  // Assuming "T" or "Cross" goes left
        } else {
          mazeEnd();
        }
        break;
        
      case RIGHT_TURN: 
        runExtraInch(extra_motorSpeed);
        readLFSsensors();
        if (mode == NO_LINE) {
          rotate(rotate_motorSpeed,1);
          recIntersection('R');
        } else {
          recIntersection('S');
        }
        break;   
        
      case LEFT_TURN:
        runExtraInch(extra_motorSpeed); 
        rotate(rotate_motorSpeed,0); 
        recIntersection('L');
        break;   
     
      case FOLLOWING_LINE: 
        followingLine(motor_Speed);
        break;      
    }
  }
}

void recIntersection(char direction) {
  path[pathLength] = direction;  // Store the intersection in the path variable.
  pathLength++;
  simplifyPath();  // Simplify the learned path.
}

void mazeEnd() {
  motorStop();
  for (int i = 0; i < pathLength; i++) {
    Serial.print(path[i]);
  }
  Serial.println("");
  Serial.print("  pathLength ==> ");
  Serial.println(pathLength);
  status = 1;
  mode = STOPPED;
}

void followingLine(int motor_Speed) {
  // Motor A forward
 move(1, 150, 0); //motor 1, full speed, left
  move(2, 150, 1); //motor 2, full speed, left
}

void simplifyPath() {
  if (pathLength < 3 || path[pathLength - 2] != 'B') {
    return;
  }

  int totalAngle = 0;
  for (int i = 1; i <= 3; i++) {
    switch (path[pathLength - i]) {
      case 'R':
        totalAngle += 90;
        break;
      case 'L':
        totalAngle += 270;
        break;
      case 'B':
        totalAngle += 180;
        break;
    }
  }

  totalAngle = totalAngle % 360;

  switch (totalAngle) {
    case 0:
      path[pathLength - 3] = 'S';
      break;
    case 90:
      path[pathLength - 3] = 'R';
      break;
    case 180:
      path[pathLength - 3] = 'B';
      break;
    case 270:
      path[pathLength - 3] = 'L';
      break;
  }

  pathLength -= 2;
}


void rotate180(int motorSpeed) {
  move(1, motorSpeed, 1); 
  move(2, motorSpeed, 1); 
  Serial.println("rotate180");
  delay(Uturn); 
  motorStop();
}

void runExtraInch(int extra_motorSpeed) {
 move(1, extra_motorSpeed, 0); //motor 1,150 speed, left
  move(2, extra_motorSpeed, 1); //motor 2, 150 speed, left
  Serial.println("runextrainch");

  delay(extraInch);
  motorStop();
}

void rotate(int motorSpeed,int direction) {
  int st = 1;
  // direction  =0 to turn left and 1 for right
  while(true){
      move(1, motorSpeed, direction); //turn
      move(2, motorSpeed, direction); //turn
      readLFSsensors();  

      if((sensorValues[0]==1)&&(sensorValues[1]==1)&&(sensorValues[2]==1)&&(sensorValues[3]==1)&&(sensorValues[4]==1)&&(sensorValues[5]==1)&&(sensorValues[6]==1)&&(sensorValues[7]==1)) {
          st=0;
      }
    if (sensorValues[3] == 0 && sensorValues[4] == 0 && st==0) {
      break; 
   }
}
  
  // delay(rotateLeft); 
  motorStop();
}

// void rotateRight90(int motorSpeed) {
//   move(1, 150, 1); 
//   move(2, 150, 1); 
//   Serial.println("rotateright90");

//   delay(rotateRight); 
//   motorStop();
// }
void motorStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);


}

void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
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
