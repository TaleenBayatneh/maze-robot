#include <Wire.h>            
#include <Adafruit_Sensor.h>   //Defining our library that we need to manage our components
#include <VL53L0X.h>
#include "I2Cdev.h"
#include "MPU6050.h"
//////////////////////////////////////////////////////////////////////////////////////
#define MAZE_ROWS 8                 //maze configuration assume each cell is 20*20 so for the encoder need to pass one cell
#define MAZE_COLS 8                 //we use this to make our code more flixiple withen the maze cells
const float CELL_SIZE_CM=  20.0;    //assumed need 5 pulses so 5*20=100 pulses to move thourgh one cell
const int   TICKS_PER_CM=     5;    //we use this to make our code more flixiple withen the maze cells
const int   TICKS_PER_CELL= int(CELL_SIZE_CM * TICKS_PER_CM);
//////////////////////////////////////////////////////////////////////////////////////
const int  WALL_THRESH_MM=    80;   // for the VL sinsor , if the VL sinsor detect an somthing in near 8 cm meters or 80mm it will consider it as a wall 
const int THINK_DELAY_MS=  2000;    // some thinking  delay to make the robot deciding its next move we will use it in our code later on 
const int BASE_SPEED=       150;    // the normal speed forward for our robot 
const int LEFT_OFFSET=      -10;    // since we have an problem with our left and right motors , so we dicide to make an offset to the left motors to make the power of it less 
const int TURN_SPEED_SLOW=   80;    // since the left motor is faster so that we reduce it power to make an ballance in our tow motors
const int TURN_MS_SLOW =    500;    // for more flixple, our code is generally route in 90 and 180 degree so the route speed is 80 and it must when route , the another motoer speed be zero, and it route for 0.5 seacnd for 90 degree and 1 secand for 180 degree                                      
//////////////////////////////////////////////////////////////////////////////////////
#define ENA 14                      //right motor PWM it will control the speed of the right motor by using the analog write 
#define IN1 32                      //right motor direction 1
#define IN2 33                      //right motor direction 2
#define ENB 12                      //left motor PWM
#define IN3 26                      //left Motor direction 1
#define IN4 18                      //left motor direction 2
#define ENCA_L 25                   //left Encoder  
#define ENCA_R 16                   //right Encoder
#define IR_LEFT_PIN  5              //side left IR sinsor
#define IR_RIGHT_PIN 4              //side right IR sinsor

VL53L0X vl53;                       //  Creates an object vl53 from the VL53L0X class globaly and for the mpu also 
MPU6050 mpu;
volatile long encL, encR;           // and for encoder tick counter we diffein them as volatile since thay are inside an interrupt service routines isr
int curR=0, curC=0, curDir=0;       // for more flixple move we assume that the robot is moving in many direction as 0 in the center and keep track the possition on it and direction 
bool goalReached=false;             //flag for reach the goul or not 
bool visited[MAZE_ROWS][MAZE_COLS] = { false };  // initialize all cell as unvisited and when we start it will begin 
const int goalR =MAZE_ROWS-1, goalC =MAZE_COLS-1;  // since we assume our maze is having an cell and so we need to make an center for the goul so we assume that the goul is if 10*10 cells   so the goul in 9*9 for assimpution
const int dr[4]={-1,0,1,0}, dc[4]={0,1,0,-1};      // so the robot need an direction to move so this array will help as to make the robot make his think and chose the next step 
//////////////////////////////////////////////////////////////////////////////////////
void IRAM_ATTR onEncL()                // for our encoder tick count we need to diffend an isr interappt 
{ encL++;                              //each time a motor shaft moves a littl, an encoder sends a pulse tick, and this function counts it
}
void IRAM_ATTR onEncR()                //this is an esp32 specific keyword it mean to put the function on the fast internal RAM . so we need it to use the interrupt on the esp
{ encR++;                              // encresed the encoder by 1 every time a pulse is detected, so these hellper methoud will till us how far the robot moved 
}
//////////////////////////////////////////////////////////////////////////////////////
void stopMotors() {                   // funcion to stop the motor in ena and enb pins that control the speed of the motores , what it do is seting the PWM speed to zero 
  analogWrite(ENA, 0);                // stop right motor
  analogWrite(ENB, 0);                // stop left motor
  delay(50);                          // Small delay by 50ms before we continue
}

void setMotorR(int speed) {          // ok know we will configer the right motor, the right one when his speed is bigger than zero is this case (moving forward)
  if (speed >= 0) {
  
    digitalWrite(IN1, LOW);          // we will give the right ir the left high and low or in oppiset one , since the tow motors generlly are inverses so 
    digitalWrite(IN2, HIGH);
  } else {                           // else mean if the speed is less that 0 then he will go backward 
                                     
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    speed = -speed;                  // PMW can get an possitve number only, so only if we go backward we need to use            
  }

  analogWrite(ENA, speed);           // give the motor PWM the value of the speed
}

void setMotorL(int speed) {         // the same consipt but for the left motor 
  if (speed >= 0) {
    
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed = -speed;  
  }                                        // The main defferant here that we detect an fast speed from the left motor as we said early so that we add an offset by -10 so the
  analogWrite(ENB, speed + LEFT_OFFSET);   // so that the left motor PWM will write on it an slower speed vale to make the balance if the tow motores
                                      
}                                   
//////////////////////////////////////////////////////////////////////////////////////
void moveOneCell(){                       // function for moving the robot forward exactly one cell 
  encL=encR=0;
  setMotorR(BASE_SPEED);                 
  setMotorL(BASE_SPEED);
  while((encL+encR)/2 < TICKS_PER_CELL){} // this here is used to wait until the robot has moved one full cell distance by checking the average encoder count
  stopMotors();                           // a stop function used after 
}
void slowTurnRight90(){                   // when every he route to 90 degree by using the constant in the begining of the code, onw mooter need to stop and one need to route 
  setMotorR( TURN_SPEED_SLOW);            // then a small delay to make te robot continue thinking 
  setMotorL(-TURN_SPEED_SLOW);
  delay(TURN_MS_SLOW);
  stopMotors();
}
void slowTurnLeft90(){                   // the same for the Left left turn 
  setMotorR(-TURN_SPEED_SLOW);
  setMotorL( TURN_SPEED_SLOW);
  delay(TURN_MS_SLOW);
  stopMotors();
}
void slowTurn180(){                      // turn 180 function  by compine both turn 90 * 2   
  slowTurnRight90();
  slowTurnRight90();
}
//////////////////////////////////////////////////////////////////////////////////////
void turnToSlow(int targetDirection)     //to rotate the robot from its current direction curDir to a target direction targetDirection using the minimum number of turns left or right or 180 turn
 {                                       // 0 north 1 east 2 south 3 west, so to go frm the cirrint location to the target location we need who many route 90 clockwise needed  so if i am in 
  int difference = (targetDirection - curDir + 4) % 4; 
                                                      
  if (difference == 1) {                              // posstioon of 0 north , then i need to go to the east then (1-0+4)%4=1 then turn right 90 degree
  
    slowTurnRight90();          
  } else if (difference == 2)
   {                                                  // and so one, these help us for updating our currant posstion 
   
    slowTurn180();
  } else if (difference == 3) {
   
    slowTurnLeft90();
  }
  
  curDir = targetDirection;
}

void thinkPause(){                                 // after any changes we need the robot to stoop and think and stop his every sinsor that he have to let the vl sensor and the ir sensor while he route , take an delay and not sensing , to not getting in the loop
  stopMotors(); delay(THINK_DELAY_MS);
}
//////////////////////////////////////////////////////////////////////////////////////
bool wallFront() {                               //check if there is a wall in front using the VL53L0X distance sensor

                                                  // read distance in millimeter, and the finction return true if there is an wall front of the robot and false if the path is clear
  uint16_t distance = vl53.readRangeSingleMillimeters();
  if (vl53.timeoutOccurred()) {                  // if the reading failed or timed out assume there is a wall
    return true;
  }
  if (distance > 0 && distance < WALL_THRESH_MM) { // if distance is less than wall threshold,so that there is a wall ahead
    return true;
  } else {
    return false;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////
bool wallLeft() {                               //function for check if theres a wall on the left using IR sensor
                                                // so that the IR sensor returns LOW when a wall is close 
  return digitalRead(IR_LEFT_PIN) == LOW;
}         
bool wallRight() {                             //check if theres a wall on the right using IR sensor and return low since the low mean there is somthing here 
  return digitalRead(IR_RIGHT_PIN) == LOW;
}
// Left Hand Step algo
void leftHandStep(){
  
  thinkPause(); // stop the movement of the robot and give time for sensors to be stable before start sensing again 
// use bool type to get reads of IR sesors and vl 
  bool wallInFront = wallFront(); // bool result of detedting wall in front useing vl 
  bool wallOnLeft  = wallLeft(); // bool result of detedting wall in left useing IR 
  bool wallOnRight = wallRight(); // bool result of detedting wall in right useing IR 
 // decide which direction to move next using left then straight, right and  back priority 
  int directionOrder[4] = {
    (curDir + 3) % 4,  // left of current dir
     curDir,           // continue moving straight
    (curDir + 1) % 4,  // Right of current dir
    (curDir + 2) % 4   // back which turn 180ْ  
  };

  int nextDirection = -1;  // its mean that there is no selected dir yet

  for (int i = 0; i < 4; i++) // loop through 4 dir priority to find the first valid way 
  {
    int direction = directionOrder[i]; // get directions 

    // for the next cell coordinates in the dir we will calculate its row and column based on dir 
    int newRow = curR + dr[direction]; // changing in row based on dir from dr array 
    int newCol = curC + dc[direction];// changing in column based on dir from dc array
    visited[curR][curC] = true;       // mark every visted cell as visted 

    // then check  if theres a wall in that direction 
    bool isBlocked = false; // initial value that there is no wall 
    if (direction == curDir) // current dir is  forward
     {
      isBlocked = wallInFront; // give the vl detect value to variable 
    } else if (direction == (curDir + 3) % 4) // current dir left 
     {
      isBlocked = wallOnLeft; // give the ir left detect value to variable 
    } else if (direction == (curDir + 1) % 4) // current dir is right 
     {
      isBlocked = wallOnRight; // give the ir right detect value to variable
    }

  //  if the dir is valid which no blocks walls and in maze bounds select this dir to be next dir and aslo not as visted  
    if (!isBlocked && newRow >= 0 && newRow < MAZE_ROWS && newCol >= 0 && newCol < MAZE_COLS && !visited[newRow][newCol]) {
      nextDirection = direction;  // so the dir is a valid dir row and column
      break;                      // no need to check the others dir
    }
  }

  if(nextDirection<0) return;   // no move found

//slow turn the robot to face the chosen dir and move forward one cell
  turnToSlow(nextDirection);//rotate robot to new dir 90°, 180°, or no turn 
  moveOneCell(); // always move forward one cell using encoders
  curR+=dr[nextDirection]; //always keep the dir row on update  
  curC+=dc[nextDirection]; // always keep the dir column on update 

// check if we reached the goal 
  if(curR==goalR && curC==goalC){
    stopMotors(); // stop it 
    //Serial.println("GOAL REACHED by left-hand rule!"); // print mes
    goalReached=true;// goal flag true to stop further moves
  }
}
void setup() {
  Serial.begin(115200); //start the serial monitor for debugging

// set motor control pins as output
  pinMode(ENA, OUTPUT); // Right motor speed
  pinMode(IN2, OUTPUT); // Right motor direction
  pinMode(ENB, OUTPUT); // Left motor speed
  pinMode(IN3, OUTPUT); // Left motor direction
  pinMode(IN4, OUTPUT); // Left motor direction

  // set encoder pins as input with pull-up resistors
  pinMode(ENCA_L, INPUT_PULLUP); // channel one for left
  pinMode(ENCA_R, INPUT_PULLUP);// channel one for right 

  //attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA_L), onEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), onEncR, RISING);

  //set IR sensor pins as input with pull-up resistors
  pinMode(IR_LEFT_PIN, INPUT_PULLUP);
  pinMode(IR_RIGHT_PIN, INPUT_PULLUP);

  Wire.begin();  //start I2C communication for sensor vl and mpu

  vl53.init();   // initialize distance sensor (VL53L0X)              
  vl53.setTimeout(200); // Set timeout to avoid hanging

  mpu.initialize();  // initialize MPU6050
}

void loop() {
  if (!goalReached) // check the flag if its not reached "true" keep looping
  {
    leftHandStep(); //keep solving the maze using left hand rule
  } else {
    delay(1000); // nothing after goal is reached will happen
  }
}
