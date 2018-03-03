/*
 * Make coins_center, coins_spoke, spoke_square_pos,   became a local value
 * changed the cap for the single-sensor-phase: hor 4, dia 5
 * 
 * @Version 01/24/2017
 */
#include <Adafruit_TCS34725.h>
#include <QTRSensors.h>
#include <math.h> 
#define INWARDS 1
#define OUTWARDS 0
#define NUM_SQUARE 4
#define NUM_SPOKE 6
#define MicrosecondPerDegree  12.5


/*
 * First wheel is front left
 * Second wheel is back right
 * Third wheel is back left
 * Fourth wheel is front right
 */
byte motion = B01010101;
byte r45 = B01010000;
byte frontmotors = B01000001;
byte rearmotors = B00010100;
byte rightmotors = B00010001; //check
byte leftmotors = B01000100;  //check

byte l45 = B00000101;
byte rotleft = B00000000;
byte rotright = B10101010;
byte fwd = B10001000;
byte rev = B00100010;
byte strafeleft = B00101000;
byte straferight = B10000010;

byte vibrate = B10100000; 

float steps_per_inch = 215.602; //recalibrate steps_per_inch and changed dis back?
float steps_per_deg = 22.9;

//float start_dis = .25;
//float straight = 1.006;//decrease to turn right, increase to turn left 
float straight = 1.006;
//double length_robot_center = 6; //inch
float baseTurnRatio = .17; //lower .05
//float baseTurnRatio = .;
//int bounce_left;   //center
//int bounce_right;    //center
float pos;
float oldpos;
float start_dis = .25;

float mov_hor[] = {  36.5, 9.1, 2.7, 8, 4.1, 3.25,  2.8};  //[0]: distance from center to the spoke, 
                                                 //[1]: dis within center, 
                                                 //[2]: backing up dis coming from spoke , 
                                                 //[3]: dis between each junction,
                                                 //[4]: backing up dis coming from spoke
												 //[5]: distance from the edge to deliver_gray_coin distance
												//[6]: backing up dis coming from center 
                                                                                             
float mov_dia[] = { 52.5, 10.5, 3.2, 6.5, 4.5, 5.74, 1.5};  //[0]: distance from center to the spoke, 
                                                 //[1]: dis within center, 
                                                 //[2]: backing up dis coming from spoke , 
                                                 //[3]: dis between each junction,
                                                 //[4]: backing up dis coming from spoke
                         //[5]: distance from the edge to deliver_gray_coin distance
                        //[6]: backing up dis coming from center 
float forwards_hor_dia_dis[] = {4.5, 5.5, 1, 1.5}; 
 //[0] forwards for horizontal coming from center 
 //[1] forwards for diagonal move coming from center 
 //[2] forwards for horizontal mov coming from spoke
 //[3] forwards for diagonal mov coming from spoke
boolean verbose = false;    //set verbose to true to see reports                
int mov_hor_steps[] = {0,0,0,0};
int mov_dia_steps[] = {0,0,0,0};
int src_pointer[] = {0};//source color spoke
int angle_pointer[] = {0};//the current angle from 0  


/*
int coin_color_table[NUM_SPOKE][NUM_SQUARE] ={              //coins positions from center outwards  simulation for round1
  {2, -1, 3, -1},                 //[0] red_spoke    :  blue, null, yellow, null
  {5, -1, 2, -1},                 //[1] green_spoke  :  cyon, null, blue, null
  {3, -1, 4, -1},                 //[2] blue_spoke   :  yellow, null, purple, null
  {0, -1, 5, -1},                 //[3] yellow_spoke :  red, null, cyon, null
  {5, -1, 0, -1},                 //[4] purple_spoke :  cyon, null, red, null
  {4, -1, 1, -1}                  //[3] cyon_spoke   :  purple, null, green, null
};                    //simulator for picking coint table
*/
/*
int coin_color_table[NUM_SPOKE][NUM_SQUARE] ={              //coins positions from center outwards  simulation for round2
  {2, 1, 3, -1},                 //[0] red_spoke    :  blue, green, yellow, null
  {5, 3, 2, -1},                 //[1] green_spoke  :  cyon, yellow, blue, null
  {3, 0, 4, -1},                 //[2] blue_spoke   :  yellow, red, purple, null
  {0, 4, 5, -1},                 //[3] yellow_spoke :  red, purple, cyon, null
  {5, 5, 0, -1},                 //[4] purple_spoke :  cyon, cyon, red, null
  {4, 2, 1, -1}                  //[3] cyon_spoke   :  purple, blue, green, null
};                    //simulator for picking coint table
*/
int coin_color_table[NUM_SPOKE][NUM_SQUARE] = {              //coins positions from center outwards  simulation for round3
  { 2, 1, 8, 3 },                 //[0] red_spoke    :  blue, green, gray, yellow
  { 8, 3, 2, 5 },                 //[1] green_spoke  :  gray, yellow, blue, cyon
  { 3, 8, 4, 0 },                 //[2] blue_spoke   :  yellow, gray, purple, red
  { 0, 8, 5, 4 },                 //[3] yellow_spoke :  red, gray, cyon, purple
  { 5, 5, 0, 8 },                 //[4] purple_spoke :  cyon, cyon, red, gray
  { 4, 2, 8, 1 }                  //[3] cyon_spoke   :  purple, blue, gray, green
};


//Red   Green   Blue   Yellow   Purple   Cyon    White(Upper) White(Lower)
//  0    1       2       3        4         5         6       7
float des_angle[] = {45, 90, 135, 225, 270, 315, 0, 180 };
long backingDelay = 600;     //Delay to back the robot
long average_delay = 300; //decrease 300
long turningDelay = 300;
long startingDelay = 800;   //use as a starting velocity so that the robot not jerk back
long slowDelay =  800;  //use to slow the robot down went it reach its destination
long stopingToPickDelay = 200;  //use when the robot found the coin to pick up
long fast_delay = 200;
long slowStepLimit = 300; //steps limit to make the robot move slow
long fastStepLimit = 300; //steps limit to make the robot move fast
long delay_bt_instr = 100;
float fixed_dist = 1;     //because the robot is big, add some dis-->adding fixed dist so when turing not lose junction
int offbit_max = 20;			//upper limit of recognizing the robot to be on the line
int onbit_max = 10;			//upper limit of recognizing the robot to be on the milestone	
int offbit_min = 5;			//lower limit of recognizing the robot to be on the line
int onbit_min =3;			//lower limit of recognizing the robot to be on the milstone

//*************Servo motion
int servo = 12;  
int relay = 26;
int drop_angle = 175;
int upright_angle = 25;
int pickup_angle = 185;
//**************Servo delay
int coin_conect_delay = 4000;
//100000
int servo_hold_delay_upright_angle = 2500;  //delay for when servo is in upright position
int servo_hold_delay_dropoff_angle = 1500;  //delay for when servo is in upright position
//****************End servo 

//Used for CCT interupt
volatile unsigned int countCompareValue;
volatile boolean flag1;
volatile int cnt;
volatile int steps = 0;
volatile int steps_counter = 0;  //count how many steps taken if more than 60 steps taken then slow the robot down
volatile byte masterWheels;
volatile byte slaveWheels;
volatile long masterCount; //counter of dis traveled
volatile double steerRatio;

//Line Sensor Values
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) { 3, 4, 5, 6, 7, 8, 9, 10 },
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
int sensorreadtime = 25;
 

//Set up a variable 
//In the interupt rountine, which used measure dis

void setup() {
  // put your setup code here, to run once: 
  DDRL = B11111111;//turn on the driving mode
  dis_to_step_setup();//save steps info to the array mov_hor_steps & mov_dia_steps
  delay(500);
  Serial.begin(9600);
  
    //**********Servo set up
   pinMode(servo, OUTPUT); //servo
   pinMode(relay, OUTPUT); //relay
   digitalWrite(relay, HIGH);
//  **********End Servo set up
//    while(true){
//    noInterrupts();
//    pickup();
//    dropoff();  
//    interrupts();
//    
//  }
 //****************Turning Testing
 
//  for(int i=0; i < 5; i++){
//  turn(rotright, 180, turningDelay);
//  delay(300);
//  }
//  exit(0);


  //***********
  //ctc1_setup();//setup the timer counter
//  autocal();      //move up and stop and up and stop and turn right to calibate
 
  selfCal();
//            
//    strafe_to_center();     //Middle line correction
//  exit(0);
//****************Round 1 testing
//  round1();
//****************Round 2 testing
//    round2();
//****************Round 3 testing
    round3();


//****************Line Testing 
// line_testing();
//  float mov_test[] = { 30, 14.15};
//    mov_interrupt(fwd, mov_test, average_delay, OUTWARDS );

 
//********************Moving  Testing
// vars(fwd,72,800, 1.006, rightmotors, leftmotors);
//turn(rotright,1080,800);

//line following testing
//mov_interrupt(fwd, mov_hor, average_delay, INWARDS);
//  turn_modify_angle(rotright, 90, 650);
}

void loop() {
//  unsigned int position = qtrrc.readLine(sensorValues);
//  for (unsigned char i = 0; i < NUM_SENSORS; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t');
//  }
//  Serial.println(position); // comment this line out if you are using raw values
//  delay(250);
}

 
 
/********************TESTING**********************/
void strafe_to_center(){
  byte tempPortL  = PORTL;
  unsigned int position = qtrrc.readLine(sensorValues);
    float pos;
    float oldpos;
    int sumIndexOfOn = 0;
    int SumOfOn = 0;
  while(position != 3500){  
    sumIndexOfOn = 0;
    SumOfOn = 0;
    position = qtrrc.readLine(sensorValues);
    for(int i=0; i<8; i++){
      if(sensorValues[i] == 1000){
        sumIndexOfOn += i;
        SumOfOn++;
      } 
     }
//*****************Checking************************
    if (SumOfOn != 0) {
      pos = (float)sumIndexOfOn / SumOfOn;
    }
    else {
     pos = oldpos;
    }
    oldpos = pos;

    if(pos<3.5){
      strafe(strafeleft,0.1,800);
    } else if(pos > 3.5){
      strafe(straferight,0.1,800);
    } else{
      break;
    }
  }
  PORTL = tempPortL;
}

void strafe( byte dir, float dist, long del){
  PORTL = dir;
  float stepf=dist*steps_per_inch;
  long steps=stepf;
  
  for(long i=0; i< steps;i++){
    delayMicroseconds(del);
    PORTL ^= motion; //toggle the step signal to continue moving 
  }
}
void turningTesting(int des){
  long mydelay = 400;
  mov(fwd, 42, mydelay);
  des = 6;      //red
  turn_src_to_des( des, 650);
  turn_modify_angle(rotleft, 180, 650);
  delay(500);

  des = 5;      //red to blue
  turn_src_to_des(des, 650);
  turn_modify_angle(rotleft, 180, 650);
  delay(500);

  des = 7;      //blue to green
  turn_src_to_des(des, 650);
  turn_modify_angle(rotleft, 180, 650);
  delay(500);

  des = 3;      //green to cyon
  turn_src_to_des( des, 650);
  turn_modify_angle( rotleft, 180, 650);
  delay(500);

  des = 1;      //cyon to red
  turn_src_to_des(des, 650);
  turn_modify_angle(rotleft, 180, 650);
  delay(500);

  des = 1;      //red to red
  turn_src_to_des( des, 650);
  turn_modify_angle(rotleft, 180, 650);
  delay(500);
 
  mov(fwd, 42, mydelay);
  return;
}

void line_testing(){
  long mydelay = 600;
   mov(fwd, 42, mydelay);

  //Red
  //0: out of center
  //1: towards center
  turn(rotleft, 45, mydelay);
  //mov_interrupt(fwd, mov_dia, mydelay,0 );
  movDiagonalOut();
  turn(rotleft, 180, mydelay);
  movDiagonalIn();
  //mov_interrupt(fwd, mov_dia, mydelay,1 );
  
  //Cyan
  turn(rotleft, 90, mydelay);
  //mov_interrupt(fwd, mov_dia, mydelay, 0 );
  movDiagonalOut();
  turn(rotleft, 180, mydelay);
  movDiagonalIn();
  //mov_interrupt(fwd, mov_dia, mydelay, 1 );
  
  //Purple
  turn(rotleft, 135, mydelay);
  movHorizontalOut();
  //mov_interrupt(fwd, mov_hor, mydelay, 0 );
  turn(rotleft, 180, mydelay);
  movHorizontalIn();
  //mov_interrupt(fwd, mov_hor, mydelay,1 );
  
  //Yellow
  turn(rotleft, 135, mydelay);
  //mov_interrupt(fwd, mov_dia, mydelay, 0 );
  movDiagonalOut();
  turn(rotleft, 180, mydelay);
  //mov_interrupt(fwd, mov_dia, mydelay, 1 );
  movDiagonalIn();
   
  //Blue
  turn(rotleft, 90, mydelay);
  //mov_interrupt(fwd, mov_dia, mydelay, 0 );
  movDiagonalOut();

  turn(rotleft, 180, 700);
  //mov_interrupt(fwd, mov_dia, mydelay, 1 );
  movDiagonalIn();
   
  //Green
  turn(rotleft, 135, mydelay);
  //mov_interrupt(fwd, mov_hor, mydelay, 0 );
  movHorizontalOut();
  turn(rotleft, 180, mydelay);
  //mov_interrupt(fwd, mov_hor, mydelay, 1 );
  movHorizontalIn();
  //White
  turn(rotright, 90, mydelay);
    delay(200);
  mov(fwd, 42, mydelay);

}
void round1(){
  boolean coins_center[NUM_SQUARE][NUM_SPOKE] = {
    { true, true, true, true, true, true },
    { false, false, false, false, false, false },
    { true, true, true, true, true, true },
    { false, false, false, false, false, false },
  };                    //coins searching FROM CENTER true means there is a coin, false mean there is not a coin
  boolean coins_spoke[NUM_SPOKE][NUM_SQUARE] = {
    { true, false, true, false },
    { true, false, true, false },
    { true, false, true, false },
    { true, false, true, false },
    { true, false, true, false },
    { true, false, true, false },
  };                    //coins searching FROM CENTER true means there is a coin, false mean there is not a coin
  run_round1_and_round2(coins_center, coins_spoke);
}

void round2(){
  boolean coins_center[NUM_SQUARE][NUM_SPOKE] = {
    { true, true, true, true, true, true },
    { true, true, true, true, true, true },
    { true, true, true, true, true, true },
    { false, false, false, false, false, false },
  };                    //coins searching FROM CENTER true means there is a coin, false mean there is not a coin
  boolean coins_spoke[NUM_SPOKE][NUM_SQUARE] = {
    { true, true, true, false },
    { true, true, true, false },
    { true, true, true, false },
    { true, true, true, false },
    { true, true, true, false },
    { true, true, true, false },
  };                    //coins searching FROM CENTER true means there is a coin, false mean there is not a coin
  run_round1_and_round2(coins_center, coins_spoke);
}

void run_round1_and_round2(boolean coins_center[][NUM_SPOKE], boolean coins_spoke[][NUM_SQUARE]) {
  int dest_color[] = { 0 };       //the index into the color angle
  int spoke_square_pos[] = { 0,0 };    //1: the index of spoke given by color, 2: the index of square start from center outwards
  int junction_counter[] = { 0 };      //junction counter is used to control the count of junction, where to stop to pick up the coin, very important to stop at the right place
  int hor_or_dia[] = { 0 };
  float distance[] = { 0.0 };

  accelerate_mov(fwd, 39, slowDelay);
  delay(delay_bt_instr);
  while (searching_for_coins_from_center(coins_center, coins_spoke, spoke_square_pos, dest_color)) {     //while there are still coins to pick
    if (verbose) {
      Serial.println("***************************Start Calculating Distance from center to coin");
    }
    calculate_dist_from_center_to_coin(distance, junction_counter, hor_or_dia, spoke_square_pos);   //calculate the junction
   find_hor_or_dia(hor_or_dia, spoke_square_pos);
   
    turn_src_to_des(dest_color[0], turningDelay);    //turning with the src_pointer and angle_pointer
    delay(delay_bt_instr);

    //move and pick up
    if (verbose) {
      Serial.println("***************************Start Picking Coin");
    }
    picking_interrupt_outwards(fwd, distance[0], startingDelay, junction_counter[0], hor_or_dia, spoke_square_pos, dest_color);
    delay(delay_bt_instr);

   
   
    turn_modify_angle(rotleft, 180, turningDelay);            //turn back 180 
    delay(delay_bt_instr);

    if (verbose) {
      Serial.println("***************************Coming back to the center");
    }
    returning_after_picking_interrupt_inwards(fwd, distance[0] + fixed_dist, startingDelay, junction_counter[0], hor_or_dia);   //go back to the center 
    delay(delay_bt_instr);
    if (dest_color[0] == -1) {    //failed to pick the coin
      continue;                //continue to search for the coin again
    }
   
   
    find_hor_or_dia(hor_or_dia, spoke_square_pos);                                        //change the horizontal and diag arr
    
    turn_src_to_des(dest_color[0], turningDelay);    //turn to the coin just picked
    delay(delay_bt_instr);
    if (verbose) {
      Serial.println("***************************Deliver the coin");
    }
    mov_interrupt_deliver_or_comingback(fwd, startingDelay, OUTWARDS, hor_or_dia);      //deliver the coin
    
     noInterrupts();
    dropoff();      //drop the coin
    interrupts();

    if (verbose) {
      Serial.println("***************************Searching for the coin from the spoke");
    }
    while (searching_for_coins_from_spoke(coins_center, coins_spoke, spoke_square_pos, dest_color)) {//searching for nearest coint and calculate the dis to get to it
      //delay(delay_bt_instr);
      delay(delay_bt_instr);
      turn_modify_angle(rotleft, 180, turningDelay);            //turn back 180, without change dest_color
      delay(delay_bt_instr);
      if (verbose) {
        Serial.println("***************************Calculate distance to picking up the coin");
      }
      calculate_dist_from_spoke_to_coin(junction_counter, spoke_square_pos);  //set junction counter to the spoke_sqaure_pos 
      
    if (verbose) {
        Serial.println("***************************Picking up the coin and move to center");
      }
      picking_interrupt_inwards_and_return_to_center(fwd, startingDelay, junction_counter[0], hor_or_dia, spoke_square_pos, dest_color, 1);    //picking up the coin and go to center
      delay(delay_bt_instr);
      if (dest_color[0] == -1) {    //failed to pick the coin
        break;                //continue to search for the coin again from the center
      }
      
      find_hor_or_dia(hor_or_dia, spoke_square_pos);                                        //change the horizontal and diag arr
      
      turn_src_to_des(dest_color[0], turningDelay);    //turn to the coin just picked
      delay(delay_bt_instr);
      if (verbose) {
        Serial.println("***************************Deliver the coin");
      }
      mov_interrupt_deliver_or_comingback(fwd, startingDelay, OUTWARDS, hor_or_dia);      //deliver the coin
      noInterrupts();
        dropoff();      //drop the coin
      interrupts();
      delay(delay_bt_instr);
      
      if (verbose) {
        Serial.println("***************************Searching for the coin from spoke");
      }
    }


    turn_modify_angle_change_dest_color(rotleft, 180, turningDelay, dest_color);            //turn back 180, and change dest_color accordingly
    delay(delay_bt_instr);
    mov_interrupt_deliver_or_comingback(fwd, startingDelay, INWARDS, hor_or_dia);      //coming back to the center if there is no coin in the spoke
    if (verbose) {
      Serial.println("***************************Searching for the coin from center");
    }
  }
  dest_color[0] = 7;     //turn to lower white
  delay(delay_bt_instr);
  turn_src_to_des(dest_color[0], turningDelay);    //turning with the src_pointer and angle_pointer
  delay(delay_bt_instr);
  accelerate_mov(fwd, 42, slowDelay);
  //mov(fwd, 42, average_delay);      

}



/*
void round3() {
  boolean coins_center[NUM_SQUARE][NUM_SPOKE] = {
    { true, true, true, true, true, true },
    { true, true, true, true, true, true },
    { true, true, true, true, true, true },
    { true, true, true, true, true, true },
  };                    //coins searching FROM CENTER true means there is a coin, false mean there is not a coin
  boolean coins_spoke[NUM_SPOKE][NUM_SQUARE] = {
    { true, true, true, true },
    { true, true, true, true },
    { true, true, true, true },
    { true, true, true, true },
    { true, true, true, true },
    { true, true, true, true },
  };                    //coins searching FROM CENTER true means there is a coin, false mean there is not a coin
  run_round3(coins_center, coins_spoke);
}*/

void round3(){
  int dest_color[] = {0};       //the index into the color angle
  int spoke_square_pos[] = { 0,0 };    //1: the index of spoke given by color, 2: the index of square start from center outwards
  int junction_counter[] = { 0 };      //junction counter is used to control the count of junction, where to stop to pick up the coin, very important to stop at the right place
  int hor_or_dia[] = {0};
  float distance[] = {0.0};

    boolean coins_center[NUM_SQUARE][NUM_SPOKE] = {
    { true, true, true, true, true, true },
    { true, true, true, true, true, true },
    { true, true, true, true, true, true },
    { true, true, true, true, true, true },
  };                    //coins searching FROM CENTER true means there is a coin, false mean there is not a coin
  boolean coins_spoke[NUM_SPOKE][NUM_SQUARE] = {
    { true, true, true, true },
    { true, true, true, true },
    { true, true, true, true },
    { true, true, true, true },
    { true, true, true, true },
    { true, true, true, true },
  };                    //coins searching FROM CENTER true means there is a coin, false mean there is not a coin

  accelerate_mov(fwd, 39, slowDelay);
  delay(delay_bt_instr);
  while(searching_for_coins_from_center(coins_center, coins_spoke, spoke_square_pos, dest_color)){     //while there are still coins to pick
    if (verbose) {
      Serial.println("***************************Start Calculating Distance from center to coin");
    }
    calculate_dist_from_center_to_coin(distance, junction_counter, hor_or_dia, spoke_square_pos);   //calculate the junction
    find_hor_or_dia(hor_or_dia, spoke_square_pos);
    
   
    turn_src_to_des(dest_color[0], turningDelay);    //turning with the src_pointer and angle_pointer
    delay(delay_bt_instr);

    //move and pick up
  if (verbose) {
    Serial.println("***************************Start Picking Coin");
  }
    picking_interrupt_outwards(fwd, distance[0], startingDelay, junction_counter[0], hor_or_dia, spoke_square_pos, dest_color);
    delay(delay_bt_instr);

    turn_modify_angle(rotleft, 180, turningDelay);            //turn back 180 
    delay(delay_bt_instr);
	//********************************* Readjust to balance


	//********************************** Readjust to balance
  if (spoke_square_pos[0] == 8) {
    deliver_gray_coin(fwd, distance[0] + fixed_dist, startingDelay, junction_counter[0], hor_or_dia); //go back to the center and deliver gray coin
    delay(delay_bt_instr);
    continue;
  }

  if (verbose) {
    Serial.println("***************************Coming back to the center");
  }
    returning_after_picking_interrupt_inwards(fwd, distance[0]+fixed_dist, startingDelay, junction_counter[0], hor_or_dia);   //go back to the center 
    delay(delay_bt_instr);
    find_hor_or_dia(hor_or_dia, spoke_square_pos);                                        //change the horizontal and diag arr
    turn_src_to_des(dest_color[0], turningDelay);    //turn to the coin just picked
    delay(delay_bt_instr);
  if (verbose) {
    Serial.println("***************************Deliver the coin");
  }
    mov_interrupt_deliver_or_comingback(fwd, startingDelay, OUTWARDS, hor_or_dia);      //deliver the coin
    noInterrupts();
    dropoff();      //drop the coin
   interrupts();

  if (verbose) {
    Serial.println("***************************Searching for the coin from the spoke");
  }
    while(searching_for_coins_from_spoke(coins_center, coins_spoke, spoke_square_pos, dest_color)){
        //delay(delay_bt_instr);
        delay(delay_bt_instr);
        turn_modify_angle(rotleft, 180, turningDelay);            //turn back 180, without change dest_color
        delay(delay_bt_instr);
       strafe_to_center();      //Middle line correction
       delay(delay_bt_instr);
      if (verbose) {
        Serial.println("***************************Calculate distance to picking up the coin");
      }
        calculate_dist_from_spoke_to_coin(junction_counter, spoke_square_pos);   
        
      if (verbose) {
        Serial.println("***************************Picking up the coin and move to center");
      }
        picking_interrupt_inwards_and_return_to_center(fwd, startingDelay, junction_counter[0], hor_or_dia, spoke_square_pos, dest_color, 3);    //picking up the coin and go to center
        delay(delay_bt_instr);
        
        if(spoke_square_pos[0] == 8){    //picked the gray coin
          break;           //not deliver, search for it from center
        }
     
      find_hor_or_dia(hor_or_dia, spoke_square_pos);
                                           //change the horizontal and diag arr
        
        turn_src_to_des(dest_color[0], turningDelay);    //turn to the coin just picked
        delay(delay_bt_instr);
      if (verbose) {
        Serial.println("***************************Deliver the coin");
      }
        mov_interrupt_deliver_or_comingback(fwd, startingDelay, OUTWARDS, hor_or_dia );      //deliver the coin
        delay(delay_bt_instr);
       noInterrupts();
        dropoff();      //drop the coin
      interrupts();
      if (verbose) {
        Serial.println("***************************Searching for the coin from spoke");
      }
    }

  if (spoke_square_pos[0] == 8) {
    continue; //re-search for next available coin
  }
  
    turn_modify_angle_change_dest_color(rotleft, 180, turningDelay, dest_color);            //turn back 180, and change dest_color accordingly
    delay(delay_bt_instr);
    mov_interrupt_deliver_or_comingback(fwd, startingDelay, INWARDS, hor_or_dia );      //coming back to the center if there is no coin in the spoke
  if (verbose) {
    Serial.println("***************************Searching for the coin from center");
  }
  }
  dest_color[0] = 7;     //turn to lower white
  delay(delay_bt_instr);
  turn_src_to_des(dest_color[0], turningDelay);    //turning with the src_pointer and angle_pointer 
  delay(delay_bt_instr);
  accelerate_mov(fwd, 42, slowDelay);
  //mov(fwd, 42, average_delay);      

}
/********************TESTING**********************/


/********************SETUP**********************/
//Count to match compare -- CTC setup
//delay 500 secounds
void ctc1_setup() {
  noInterrupts();
  TCCR1A = 0;  // clear counter control register
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 8000; // compare match register – 500 usecond delay
          // countCompareValue = del * 16
          // countCompa
//  alue = 16000000 / prescaler / desired frequency
  TCCR1B |= (1 << WGM12); // count to compare mode
  TCCR1B |= (1 << CS10); // 1 prescaler
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  interrupts();
}

void ctc4_setup() {
  noInterrupts();
  TCCR4A = 0;  // clear counter control register
  TCCR4B = 0;
  TCNT4 = 0;

  OCR4A = 8000; // compare match register – 500 usecond delay
          // countCompareValue = del * 16
          // countCompareValue = 16000000 / prescaler / desired frequency
  TCCR4B |= (1 << WGM42); // count to compare mode
  TCCR4B |= (1 << CS40); // 1 prescaler
  TIMSK4 |= (1 << OCIE4A); // enable timer compare interrupt
  interrupts();
}
void ctc3_setup() {
  noInterrupts();
  TCCR3A = 0;  // clear counter control register
  TCCR3B = 0;
  TCNT3 = 0;

  OCR3A = 8000; // compare match register – 500 usecond delay
          // countCompareValue = del * 16
          // countCompareValue = 16000000 / prescaler / desired frequency
  TCCR3B |= (1 << WGM32); // count to compare mode
  TCCR3B |= (1 << CS30); // 1 prescaler
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  interrupts();
}

/*
* To restart TCNT
*
*
* noInterrupts();
*  TCNT1  = 0;
*   OCR1A = 8000; // compare match register – 500 usecond delay
*interrupts();
*/
ISR(TIMER1_COMPA_vect) { // timer compare ISR
  if (steps > 0) {
    PORTL ^= motion;
    steps--;
  }
}

//3: Slave
ISR(TIMER3_COMPA_vect) { // timer compare ISR
  if (steps > 0) {
    PORTL ^= slaveWheels;
    if(steps_counter < fastStepLimit){
       steps_counter++;
    }
    
  }
}

//4: Master
ISR(TIMER4_COMPA_vect) { // timer compare ISR
  if (steps > 0) {
    PORTL ^= masterWheels;
    steps--;
  }
}

 
void autocal() {//auto calibration using interruption
 
  int loop_counter = 1;
  while (loop_counter-->0) {
    mov_i(fwd, 42, 1000);
    while (steps > 0) {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)    
    }
   
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  }
}

void selfCal() {
  int calmin[] = { 668, 668, 668, 668, 668, 668, 668, 668 };
  int calmax[] = { 900, 900, 900, 900, 900, 900, 900, 900 };
  for (int i = 0; i < 10; i++) {
    qtrrc.calibrate();
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    qtrrc.calibratedMinimumOn[i] = calmin[i];
    qtrrc.calibratedMaximumOn[i] = calmax[i];
  }
 
}

/*
 * mov the robot without tracking or interrupt
 */
void mov( byte dir, float dist, long del){
  PORTL = dir;
  float stepf=dist*steps_per_inch;
  long steps=stepf;
  
  for(long i=0; i< steps;i++){
  delayMicroseconds(del);
  PORTL ^= motion; //toggle the step signal to continue moving
  
  }
}

/*
 * Turning dist array from inch to step
 */
void dis_to_step_setup(){
  int numElement =  sizeof(mov_hor) / sizeof(mov_hor[0]);
  for(int i=0; i< numElement; i++){
    mov_hor_steps[i] = mov_hor[i] * steps_per_inch;
    mov_dia_steps[i] = mov_dia[i] * steps_per_inch;
  }
}
      
    


/********************SETUP**********************/

/********************FUNCTION**********************/
  void movDiagonalOut() {
 
        delay(200);
        mov_interrupt(fwd, mov_dia, average_delay, OUTWARDS );
        delay(200);
      }
      void movDiagonalIn() {
 
        delay(200);
        mov_interrupt(fwd, mov_dia, average_delay, INWARDS );
        delay(200);
      }
      void movHorizontalOut() {
 
        delay(200);
        mov_interrupt(fwd, mov_hor, average_delay, OUTWARDS );
        delay(200);
      }
      void movHorizontalIn() {
 
        delay(200);
        mov_interrupt(fwd, mov_hor, average_delay, INWARDS );
        delay(200);
      }
/*
 * Used For Setup
 * Simple move and set the float
 */
void mov_i(byte dir, float dist, long del) {//move with i
  PORTL = dir;
  float stepf = dist * steps_per_inch;
  steps = stepf;
  //  T time interval between consecutive interrupt requests, the value pf OCRx register
   //************************Set Variables********************************
    noInterrupts();
    TCNT1 = 0;
    OCR1A = (del * 16);
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    interrupts();
    //************************Set Variables********************************
}

/*
 * Used for moving and slowing down the robot after a certain distance
 */
void mov_i_slowdown(byte dir, float dist, long del) {//move with i
  boolean isNotSlowDown = true;
   mov_i(dir, dist, del);
   while (steps > 0) {
      if(steps < slowStepLimit && isNotSlowDown){
           noInterrupts();
          OCR1A = (slowDelay * 16); //reset the speed
          TCNT1 = 0;
           interrupts();
        isNotSlowDown = false;
      }
   }
}




/*
 * Balance the robot master and slave wheels
 */
void balance(byte dir, unsigned int del, int steps_setter){
    PORTL =  dir;
    noInterrupts();
    TCNT4 = 0;
    TCNT3 = 0;
    OCR4A = del * 16;
    OCR3A = del * 16;
    if(steps_setter >0){
       steps = steps_setter;
    }
   
    interrupts();
}

 



/*
 * @Deprecated don't use it
* Turning the robot
*/
void turn(byte dir, float deg, long del) {
  PORTL = dir;
  float stepf = deg*steps_per_deg;
  long steps = stepf;

  for (long i = 0; i< steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motion; //toggle the step signal to continue moving
  }
}

/*
 * Use this instead of turn
* Optimized turn left or right based on the angle
*/
void turn_opt(byte dir, float deg, long del) {
  if (!(dir^rotleft)) {   //direction is left
    if (deg < 180) {
      turn(rotleft, deg, del);
    }
    else {
      deg = 360-deg;
      turn(rotright,deg, del);
    }
  }
  else {
    if (deg >= 180) {
      deg = 360-deg;
      turn(rotleft, deg, del);
    }
    else {
      turn(rotright, deg, del);
    }
  }
}

void turn_src_to_des(int des, long del) {
  //src_des_arr[1] = des;
  //mov from center to red;
  int curr_angle = angle_pointer[0];
  float turn_angle = curr_angle - des_angle[des]; //substract src angle from des angle
  if (turn_angle < 0) {
   float temp = turn_angle * -1;
    turn_opt(rotleft, temp, del);
  }
  else {
    turn_opt(rotright, turn_angle, del);
  }
  angle_pointer[0] = curr_angle - turn_angle;
  src_pointer[0] = des;
 delay(500);
}

/*
 * Turning while updating variable 
 * dest_color to its correct position
 * angle pointer to its correct angle
 */
void turn_modify_angle(byte dir, float deg, long del){
  turn_opt(dir, deg, del);
  angle_pointer[0] += deg;
  if(angle_pointer[0]>=360)
    angle_pointer[0]-= 360;
}

/*
 * The same as turn_modify_angle but change dest_color
 */
void turn_modify_angle_change_dest_color(byte dir, float deg, long del, int  dest_color[]){
    turn_modify_angle(dir, deg, del);
    switch(dest_color[0]){
    case 0:           //Red is dest
      dest_color[0] = 3;       //turn 180 means it's yelllow
      break;
    case 1:           //Green is dest
      dest_color[0] = 4;       //turn 180 means it's purple
      break;
    case 2:           //Blue is dest
      dest_color[0] = 5;       //turn 180 means it's Cyon
      break;
    case 3:           //Yellow is dest
      dest_color[0] = 0;       //turn 180 means it's Red
      break;
    case 4:           //Purple is dest
      dest_color[0] = 1;       //turn 180 means it's Green
      break;
    case 5:           //Cyon is dest
      dest_color[0] = 2;       //turn 180 means it's Blue
      break;
    case 6:           //Lower white is dest
      dest_color[0] = 7;       //turn 180 means it's Upper white
      break;
    case 7:           //Upper white is dest
      dest_color[0] = 6;       //turn 180 means it's Lower white
      break;
    default:
      exit(-1);
  }
}

//****************************************************************************************************
 

/*
* An optimized function readSensor based on old readSensor
*@param arr_sum_of_on_status  the array of status that contains sumOfOn and sumIndexOfON
*@param senVal  the Value of the sensors
@param arr_event_status the array of status that contain num of on-to-off, and num of off-to-on, num of new-on, number of old-on
*/

unsigned int readSensor(int* arr_position_sensor, unsigned int* senVal, int* arr_event_status, int *hor_or_dia) {

/*
	int junction_recognition_limit_oldBit;   //limit to control old bit
	int junction_recognition_limit_newBit;   //limit to control new bit
	//Case 1, if arr_event is null
	if (hor_or_dia[0]) { //if this move diagonally
		junction_recognition_limit_oldBit = 30;
		junction_recognition_limit_newBit = 30;

	}
	else { //  horizontally
		junction_recognition_limit_oldBit = 20;
		junction_recognition_limit_newBit = 30;

	}
*/
	unsigned int SumOfOn = 0;      //sum of sensors that are on
	unsigned int sumIndexOfOn = 0;      //Sum index of on sensors
	unsigned int eventCounter = 0;

	//May need to change the cap for horizontal to 4
   //May need to change the cap for dia to 5
	int temp_sumOfOff = 0;
	for (int i = 0; i < 8; i++) {
		if (senVal[i] == 1000) { break; }  //check if it all off, if there is 1 one, jump out of loop
		temp_sumOfOff += 1;
	}
	if (temp_sumOfOff == 8) {
		return 0;
	}

	for (int i = 0; i < 8; i++) {
		if (senVal[i] == 1000) {
			if (arr_position_sensor[i] == 0) {    //there is an event if the old one is not like new one
				arr_position_sensor[i] = 1;
				eventCounter++;
			}
			sumIndexOfOn += i;
			SumOfOn++;
		}
		else {
			if (arr_position_sensor[i] == 1) {//there is an event if the old one is not like new one
				arr_position_sensor[i] = 0;
				eventCounter++;
			}
		}
	}
	
	if (SumOfOn != 0) {
		pos = (float)sumIndexOfOn / SumOfOn;
	}
	else {
		pos = oldpos;
	}
	oldpos = pos;

	//*****************Checking************************
	if (verbose) {
		Serial.print("Pos: ");
		Serial.print(pos);
		Serial.print("\tSum of Count: ");
		Serial.print(SumOfOn);
		Serial.print("\tsaved_onbit: ");
		Serial.print(arr_event_status[0]);
		Serial.print("\tsaved_offbit: ");
		Serial.print(arr_event_status[1]);
    Serial.print("\tjuncStatus: ");
    Serial.print(arr_event_status[2]);
    Serial.print("\ton_count: ");
    Serial.print(arr_event_status[4]);
     Serial.print("\toff_count: ");
    Serial.println(arr_event_status[5]);
  
	}

	//***********************************Start new Code
	 
	/*
	arr_event_status[0]: saved_onbit; should be 0
	arr_event_status[1] saved_offbit; should be 1 when start
	arr_event_status[2] valid_junction = 0;
	arr_event_status[3] junction;
	arr_event_status[4] on_count;
	arr_event_status[5] off_count;
	*/


	if (arr_event_status[0] > arr_event_status[1] && arr_event_status[2]) {
		arr_event_status[2] = 0;
		arr_event_status[3]++;
	}
	if (SumOfOn < 4) {	//less than 4 sensors are on
		if (arr_event_status[5] >= offbit_max) {	//max: to be recognized
			arr_event_status[1] = 1;
			arr_event_status[5] = 0;
			arr_event_status[2] = 1;

			arr_event_status[0] = 0;
			arr_event_status[4] = 0;
		}
		else {
			if (arr_event_status[4] <= onbit_min) { //min: to be ignored
				arr_event_status[0] = 0;
				arr_event_status[4] = 0;
			}  
        arr_event_status[5]++;
			 
		}
	}
	else {
		if (arr_event_status[4] >= onbit_max) {	//max: to be recognized
			arr_event_status[0] = 1;
			arr_event_status[4] = 0;
 
			arr_event_status[1] = 0;
			arr_event_status[5] = 0;
		}
		else {
			if (arr_event_status[5] <= onbit_min) {
				arr_event_status[1] = 0;
				arr_event_status[5] = 0;
			}  
        arr_event_status[4]++;
     
		}
	}

	if (arr_event_status[5] > offbit_max) {
		arr_event_status[5] = offbit_max;
	}

	if (arr_event_status[4] > onbit_max) {
		arr_event_status[4] = onbit_max;
	}

	//***********************************End new Code


   
  return eventCounter;        //return number of events
}




/*
* Variable speed
*/
void new_vars(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  ctc3_setup();
  ctc4_setup();
  PORTL = dir;
  float stepf = dist*steps_per_inch;
  masterWheels = master;
  slaveWheels = slave;

  noInterrupts();
  //OCR4A = 16 * del - 1;//setup speed for master
  OCR4A = 16 * del;
  TCNT4 = 0;//reset
  float temp = del * ratio;
  long slaveDelay = temp;
  //OCR3A = slaveDelay * 16 - 1;//setup speed for slave 
  OCR3A = slaveDelay * 16;
  TCNT3 = 0;//reset
  steps = stepf;
  steps_counter = 0;  //reset step counter
  interrupts();

}

/*
 * Its going to change the robot's speed, not only can slow down but can speed up adjust delay after call this function to see affect
 */
void slow_down(int del){
    noInterrupts();
  //OCR4A = 16 * del - 1;//setup speed for master
  OCR4A = 16 * del;
  TCNT4 = 0;//reset
  OCR3A = del * 16;
  TCNT3 = 0;//reset
   interrupts();
}
/*
Possible to reduce baseTurnRatio once converged
*/
void track(double mypos, unsigned int del) {
  //Wait for event
 
   double stepRatio = ((double)(mypos - 3.5)) * baseTurnRatio;
 
    if(mypos < 3.5){ //turning right  
      double turnRatio = straight + stepRatio;
      unsigned int steps_to_turn = turnRatio * del * 16;
      //************************Set Variables********************************
      noInterrupts();
      TCNT4 = 0;
      TCNT3 = 0;
      OCR4A =   steps_to_turn ;//don't want to messup with steps--> change speed for slave only
      OCR3A = 16 * del; 
      interrupts();
      //************************Set Variables********************************
    }
    else if (mypos > 3.5) {
      double turnRatio = straight - stepRatio;
      unsigned int steps_to_turn = turnRatio * del * 16;
      //************************Set Variables********************************
      noInterrupts();
      TCNT4 = 0;
      TCNT3 = 0;
      OCR4A =   del * 16 ;  
      OCR3A = steps_to_turn;   
      interrupts();
      //************************Set Variables********************************
    }
   
    else if (mypos == 3.5) {
      int temp = 16 *del;
      if(OCR4A != temp || OCR3A != temp){
       
        noInterrupts();
        TCNT4 = 0;
        TCNT3 = 0;
        OCR4A = temp; //same speed
        OCR3A = temp; //same speed
        interrupts();
        
      }
    }
}
//if step count == 20 or 50
//then call a function that reset the counting parameter and slowing the robot down
//modify ISA from inside
//call it 1 time 

//array of position 0,1,2,3,4,5,6,7
//spoke var carry which diagonal you are on --> can derive the dis 
//angle you on and angle you want to go and substract
//array of distances
//if I im at coin 4 at spoke 3---distance to the center
/*
* Special interrupt move
* function to move the robot while readining the line, auto correct its position
* int_or_out 0 for outwards center, 1 for  inwards
*/
void mov_interrupt(byte dir, float dist[], long del, int in_or_out) {
  boolean isNotSlowDown = true;
  int hor_or_dia = 1;//1 mean horizontal
  int arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  /*
   * 0: saved_onbit
   * 1: saved_offbit
   * 2: valid_junction
   * 3: junction counter
   * 4: on_count
   * 5: off_count
   *
   */
  int  arr_event_status[] = { 0,1,0,0,0, 0};
  int junction_limit = 5;
  new_vars(dir, dist[0], del, straight, rightmotors, leftmotors);
  int just_stop_count =1;
  while (steps > 0) {
    if(steps < slowStepLimit && isNotSlowDown){
      slow_down(slowDelay);
      del = slowDelay;
      isNotSlowDown = false;
    }
    unsigned int position = qtrrc.readLine(sensorValues);
    int oldVal = arr_event_status[3];
    unsigned int sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, &hor_or_dia); //read the sensor  
    if (verbose) {
      Serial.print("\\\\\\\\\\\\\\\\Junction: ");
      Serial.println(arr_event_status[3]);
   }
    if (in_or_out == OUTWARDS) {   //going out
      int coint_picking_juction = 4;
      if(oldVal != arr_event_status[3]&& arr_event_status[3] <= coint_picking_juction ){  //found a junction 
        //void mov( byte dir, float dist, long del){
        if(just_stop_count % 2){
          //junction flag on or off
          unsigned int temp_step = steps;
          steps =0;
          delay(200);
          mov(rev, dist[2], backingDelay);    //moving back
          delay(200);
          
           
          balance(fwd, average_delay, (temp_step + dist[2]*steps_per_inch));
          if(verbose){
            Serial.print("Steps to take: ");
            Serial.println(steps);
          }
          delay(200);
        
          arr_event_status[0] =0;
          arr_event_status[1] =1;
          arr_event_status[3]--;
         
//          sumEvents = 0;    //reset event
        }
        just_stop_count++;
      }
      if (sumEvents > 0 && arr_event_status[3] < junction_limit) {
        track(pos, del);
      } 
    }
    else {            //coming in
      if (arr_event_status[3] < junction_limit) {     //if it not reach final sqaure
        if (sumEvents > 0) {
          //Call a track function if an event occurs
          track(pos, del);
        }
      }
      else {
         
          arr_event_status[3] =0;     //reset
       
          balance(fwd,del, (dist[1] * steps_per_inch));
        
      }
    }
  }
}


 

/*
 * Search for next coin
 */
int searching_for_coins_from_center(boolean coins_center[][NUM_SPOKE], boolean coins_spoke[][NUM_SQUARE], int *spoke_square_pos, int dest_color[]){
  for(int i =0; i< NUM_SQUARE; i++){
      //use dest color to find the min of dis
      int min_spoke_from_dest = 1000;     //1000 is some arbitrary num to check in case couldn't find the spoke
      int spoke_from_dest = dest_color[0]; //the index of spoke from the dest_color
    for(int j=0; j< NUM_SPOKE; j++){
      if(coins_center[i][j]){   //if the value is true, found the next coin, set position to that coin
        int curr_diff =  abs(j-dest_color[0]);
        if(min_spoke_from_dest > curr_diff){
          min_spoke_from_dest = curr_diff;      //set the min to the new difference
          spoke_from_dest = j;                  //set the turning spoke
         if(verbose){
            identify_color_spoke(j);
         }
        }
      }
    }
    if(min_spoke_from_dest != 1000){        //found somewhere to turn
      spoke_square_pos[0] = spoke_from_dest;   //set spoke to spoke_from_dest
      spoke_square_pos[1] = i;   //set square to i 
      dest_color[0] = spoke_from_dest;         //set direction
      coins_center[i][spoke_from_dest] = false; //there is no coin there, after picking
      coins_spoke[spoke_from_dest][i] = false; //there is no coin there, after picking
      return 1;   //there are still coins to pick
    }
  }
  return 0;   //there no coin to pick
}
/*
 * set the junction_counter_ptr to the val based on the square 
 * set hor_or_dia to 0 if move horizontal, 1 if move diagonal
 * return the distance from center to coin
 */
void calculate_dist_from_center_to_coin(float *dist, int *junction_counter, int *hor_or_dia, int *spoke_square_pos){
  
  switch(spoke_square_pos[1]){
    case 0:
      junction_counter[0] = 1;
      dist[0] = 16.7;   //estimated
      break;
    case 1:
    junction_counter[0] = 2;
      dist[0] = 23.2;    //estimated
      break;
    case 2:
    junction_counter[0] = 3;
      dist[0] = 31.5;    //estimated
      break;
    case 3:
    junction_counter[0] = 4;
      dist[0] = 39.94;    //estimated
      break;
    default:
      exit(-1);
  }
  
}
void find_hor_or_dia(int *hor_or_dia, int *spoke_square_pos){
  int spoke_loc = spoke_square_pos[0];
  
  if(spoke_loc == 1 || spoke_loc ==4){
    hor_or_dia[0] = 0;
  } else{
    hor_or_dia[0] = 1;
  }
}

/*
* Special interrupt move outwards (from the center to the spoke) to pick up the coin
* function to move the robot while readining the line, auto correct its position
* whichIndex  index of dist for the distance to move
* backup_dist distance to backing up to pick the coin
*/
void picking_interrupt_outwards(byte dir, float dist, long del, int junction_limit,int *hor_or_dia, int *spoke_square_pos, int dest_color[]) {
   boolean isNotSlowDown = true;  //slowdown at the end
   boolean isNotSpeedUp = true;  //speed up at the start
   float backup_dist;
   float forward_dist;
   if(hor_or_dia[0]){ //if this move diagonally
      backup_dist = mov_dia[6]; //backup diagonal dis
      forward_dist = forwards_hor_dia_dis[1];
   } else{
     backup_dist = mov_hor[6];  //back up horizontal dis
     forward_dist = forwards_hor_dia_dis[0];
   }
  int arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  /*
  * 0: saved_onbit
  * 1: saved_offbit
  * 2: valid_junction
  * 3: junction counter
  * 4: on_count
  * 5: off_count
  *
  */
  int  arr_event_status[] = { 0,1,0,0,0,0};
  arr_event_status[3] = 0;
  int turn_off_picking = 1;             //1 means not to turn off picking coin
  new_vars(dir, dist, del, straight, rightmotors, leftmotors);
  
  while (steps > 0) {
    if(steps_counter == fastStepLimit && isNotSpeedUp){
      slow_down(average_delay);
      del = average_delay;
      isNotSpeedUp = false;
    }
    if(steps < slowStepLimit && isNotSlowDown){
      slow_down(slowDelay);
      del = slowDelay;
      isNotSlowDown = false;
    }

    unsigned int position = qtrrc.readLine(sensorValues);
    unsigned int sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, hor_or_dia); //read the sensor  

    if(verbose){
      Serial.print("\\\\\\\\\\\\\\\\Junction: ");
    Serial.println(arr_event_status[3]);
    }

      if(arr_event_status[3] == junction_limit && turn_off_picking){  //found a picking up coin   
      
          //junction flag on or off
          unsigned int temp_step = steps;
        slow_down(slowDelay);   //make it slow when picking up the coin
        del = slowDelay;
          steps = fixed_dist * steps_per_inch;   //steps to move upwards
          while(steps > 0){
        if (steps_counter == fastStepLimit && isNotSpeedUp) {
          slow_down(backingDelay);
          del = backingDelay;
          isNotSpeedUp = false;
        }
        if (steps < slowStepLimit && isNotSlowDown) {
          slow_down(slowDelay);
          del = slowDelay;
          isNotSlowDown = false;
        }
        position = qtrrc.readLine(sensorValues);
        sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, hor_or_dia); //read the sensor 
        track(pos, del); 
          }
          delay(stopingToPickDelay);

          //******************** Backing up
          PORTL = rev;  //set direction to reverse
          slow_down(slowDelay);
			del = slowDelay;

          steps = (backup_dist+fixed_dist) * steps_per_inch;   //moving back while reading
          steps_counter = 0;    //reset steps_counter
      isNotSpeedUp = true;
      isNotSlowDown = true;
      while(steps > 0){
      if (steps_counter == fastStepLimit && isNotSpeedUp) {
        slow_down(slowDelay);
        del = slowDelay;
        isNotSpeedUp = false;
      }
      if (steps < slowStepLimit && isNotSlowDown) {
        slow_down(slowDelay);
        del = slowDelay;
        isNotSlowDown = false;
      }
       }
          PORTL = fwd;  //set direction to fowards
          //******************** End Backing up
   
    delay(stopingToPickDelay);
    picking_up_function_simulator(spoke_square_pos, dest_color);    //psuedo code of picking up function
      
    arr_event_status[0] =0;
    arr_event_status[1] =1;
    turn_off_picking = 0;                         //turning off checking to pick the coin
          
    delay(stopingToPickDelay);

	  slow_down(slowDelay);   //make it slow then it will accelerate
	  del = slowDelay;
    //float dis_forwards = temp_step + backup_dist*steps_per_inch*2 + fixed_dist * steps_per_inch; //adding fixed dist so when turing not lose junction
    float dis_forwards = (forward_dist+backup_dist) * steps_per_inch; 
    steps_counter = 0;    //reset steps_counter 
    isNotSpeedUp = true;      //it will speed up 
    isNotSlowDown = true;     //it will slow down at the end
    balance(fwd, slowDelay, dis_forwards);
          
      }
        track(pos, del);
  }
  
}

/*
 * make the robot mov fast and slow according to a certain step limit
 * no tracking needed
 */
void accelerate_mov(byte dir, float dist, long del) {
  boolean isNotSpeedUp = true;
  boolean isNotSlowDown = true;
  new_vars(dir, dist, del, straight, rightmotors, leftmotors);
 
  while (steps > 0) {
   
    if (steps_counter == fastStepLimit && isNotSpeedUp) {
      slow_down(average_delay);
      del = average_delay;
      isNotSpeedUp = false;
    }
    if (steps < slowStepLimit && isNotSlowDown) {
      slow_down(slowDelay);
      del = slowDelay;
      isNotSlowDown = false;
    }
  }
}

/*
* make the robot mov fast and slow according to a certain step limit
* tracking is performed
*/
void accelerate_mov_tracking(byte dir, float dist, long del) {
  int arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  /*
  * 0: saved_onbit
  * 1: saved_offbit
  * 2: valid_junction
  * 3: junction counter
  * 4: on_count
  * 5: off_count
  *
  */
  int  arr_event_status[] = { 0,1,0,0,0,0 };
  boolean isNotSpeedUp = true;
  boolean isNotSlowDown = true;
  new_vars(dir, dist, del, straight, rightmotors, leftmotors);
  unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, 1); //read the sensor  

  while (steps > 0) {
    if (steps_counter == fastStepLimit && isNotSpeedUp) {
      slow_down(average_delay);
      del = average_delay;
      isNotSpeedUp = false;
    }
    if (steps < slowStepLimit && isNotSlowDown) {
      slow_down(slowDelay);
      del = slowDelay;
      isNotSlowDown = false;
    }
  
    position = qtrrc.readLine(sensorValues);
    sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, 1); //read the sensor 
    track(pos, del);
  }
}

/*
 * Function to pick up the coin, identify color and set picking_interrupt_outwards[0] accordingly
 */
void picking_up_function_simulator( int *spoke_square_pos, int dest_color[]){
  noInterrupts();
  pickup();
  interrupts();
  //simlation pickup
  int pick_spoke = spoke_square_pos[0];
  int pick_square = spoke_square_pos[1];
  spoke_square_pos[0] = coin_color_table[pick_spoke][pick_square];    //change the spoke position
  dest_color[0] = spoke_square_pos[0];     //set direction
  
 /* if(dest_color[0] == -1){
    //Reset the table
    coins_spoke[pick_spoke][pick_square] = true;    //check back because square can be different
    coins_center[pick_square][pick_spoke] = true;   //check back because square can be different
  }*/
}

 
/*
 *  whichIndex  index of dist for the distance to move
 */

void returning_after_picking_interrupt_inwards(byte dir, float dist, long del, int junction_limit,int *hor_or_dia) {
   boolean isNotSlowDown = true;
   boolean isNotSpeedUp = true;
   float align_cent_dis;      //align to center distance
   if(hor_or_dia[0]){ //if this move diagonally
      align_cent_dis = mov_dia[1]; //backup diagonal dis
   } else{
     align_cent_dis = mov_hor[1];  //back up horizontal dis
   }
  int arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  /*
  * 0: saved_onbit
  * 1: saved_offbit
  * 2: valid_junction
  * 3: junction counter
  * 4: on_count
  * 5: off_count
  *
  */
  int  arr_event_status[] = { 0,1,0,0,0,0 };
 
  new_vars(dir, dist, del, straight, rightmotors, leftmotors);
  
  while (steps > 0) {
    if(verbose){
      Serial.print("*******************Steps_counter:  ");
      Serial.println(steps_counter);
    }
 
    if(steps_counter == fastStepLimit && isNotSpeedUp){
 
       slow_down(average_delay);
       del = average_delay;
       isNotSpeedUp = false;
 
     }
    if(steps < slowStepLimit && isNotSlowDown){
      slow_down(slowDelay);
      del = slowDelay;
      isNotSlowDown = false;
    }
    unsigned int position = qtrrc.readLine(sensorValues);
    unsigned int sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, hor_or_dia); //read the sensor  
  if(verbose){
     Serial.print("\\\\\\\\\\\\\\\\Junction: ");
    Serial.println(arr_event_status[3]);
  }


      if (arr_event_status[3] < junction_limit) {     //if it not reach final sqaure
        if (sumEvents > 0) {
          //Call a track function if an event occurs
          track(pos, del);
        }
      }
      else {
        //move fixed distance.
        arr_event_status[3] =0;     //reset

        steps = align_cent_dis * steps_per_inch;    //set the steps
        //slow_down(average_delay);
        //del = average_delay;
        if(verbose){
          Serial.print("Im going to move according from edge to center with these step ");
          Serial.println(steps);
        }
        balance(fwd, del, -1);
      }
 
  }
  arr_event_status[3] = 0;  //reset event status
}

void deliver_gray_coin(byte dir, float dist, long del, int junction_limit, int *hor_or_dia) {
  boolean isNotSlowDown = true;
  boolean isNotSpeedUp = true;
  float align_cent_dis;      //align to center distance
  float deliver_cent_dis;   //distance to deliver the gray coin
  float remaining_dis;    //distance from the gray coin to the center
  if (hor_or_dia[0]) { //if this move diagonally
    align_cent_dis = mov_dia[1]; //backup diagonal dis
    deliver_cent_dis = mov_dia[5];
  }
  else {
    align_cent_dis = mov_hor[1];  //back up horizontal dis
    deliver_cent_dis = mov_hor[5];
  }
    deliver_cent_dis = mov_dia[5];
    remaining_dis = align_cent_dis - deliver_cent_dis;
  int arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  /*
  * 0: saved_onbit
  * 1: saved_offbit
  * 2: valid_junction
  * 3: junction counter
  * 4: on_count
  * 5: off_count
  *
  */
  int  arr_event_status[] = { 0,1,0,0,0,0 };
  new_vars(dir, dist, del, straight, rightmotors, leftmotors);

  while (steps > 0) {
    if (verbose) {
      Serial.print("*******************Steps_counter:  ");
      Serial.println(steps_counter);
    }

    if (steps_counter == fastStepLimit && isNotSpeedUp) {

      slow_down(average_delay);
      del = average_delay;
      isNotSpeedUp = false;

    }
    if (steps < slowStepLimit && isNotSlowDown) {
      slow_down(slowDelay);
      del = slowDelay;
      isNotSlowDown = false;
    }
    unsigned int position = qtrrc.readLine(sensorValues);
    unsigned int sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, hor_or_dia); //read the sensor  
    if (verbose) {
      Serial.print("\\\\\\\\\\\\\\\\Junction: ");
      Serial.println(arr_event_status[3]);
    }


    if (arr_event_status[3] < junction_limit) {     //if it not reach final sqaure
      if (sumEvents > 0) {
        //Call a track function if an event occurs
        track(pos, del);
      }
    }
    else {
      //move fixed distance.
      arr_event_status[3] = 0;     //reset

      //****************move to deliver the gray coin
      steps = deliver_cent_dis * steps_per_inch;    //set the steps
      steps_counter = 0;
      //slow_down(slowDelay);   //make it slow when picking up the coin
      //del = slowDelay;
      //isNotSpeedUp = true;
      //isNotSlowDown = true;
      while (steps > 0) {
        if (steps < slowStepLimit && isNotSlowDown) {
          slow_down(slowDelay);
          del = slowDelay;
          isNotSlowDown = false;
        }
        position = qtrrc.readLine(sensorValues);
        sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, hor_or_dia); //read the sensor 
        track(pos, del);
      }
      delay(stopingToPickDelay);
      //****************move to deliver the gray coin
     
      //****************deliver coin algorithm here
      noInterrupts();
  	  dropoff();
      interrupts();

      //****************deliver coin algorithm here

      //move to the rest distance
      steps = remaining_dis * steps_per_inch;    //set the steps
      steps_counter = 0;
      slow_down(slowDelay);   //make it slow when picking up the coin
      del = slowDelay;
      isNotSpeedUp = true;
      isNotSlowDown = true;
      balance(fwd, del, -1);
    }

  }
}


/*
 * Search for the squared given by dest_color
 */
int searching_for_coins_from_spoke(boolean coins_center[][NUM_SPOKE], boolean coins_spoke[][NUM_SQUARE], int *spoke_square_pos, int dest_color[]){
    for(int j=NUM_SQUARE-1; j>=0 ; j--){    //from 3 to 0, outer to inner
      if(coins_spoke[dest_color[0]][j]){   //if the value is true, found the next coin, set position to that coin
        spoke_square_pos[1] = j;   //set square to j
        if(verbose){
          Serial.print("**********Picking up square: ");
          Serial.println((3-j)+1);
        }
        coins_spoke[dest_color[0]][j] = false; //there is no coin there, after picking
        coins_center[j][dest_color[0]] = false; //there is no coin there, after picking
        return 1;   //there are still coins to pick
      }
    }
  return 0;   //there's no coin to pick
}

/*
 * set the junction_counter_ptr to the val based on the square 
 * set hor_or_dia to 0 if move horizontal, 1 if move diagonal
 * return the distance from center to coin
 */
void calculate_dist_from_spoke_to_coin(int *junction_counter, int *spoke_square_pos){
  //Square position from center to spoke so the dist is reverse
  switch(spoke_square_pos[1]){
    case 3:                       //nearest to the spoke, because arr coin_spoke is from center to spoke so it has weird index
    junction_counter[0] = 1;
//      dist[0] = 4.5;   //estimated
      break;
    case 2:
    junction_counter[0] = 2;
//      dist[0] = 13.2;    //estimated
      break;
    case 1:
    junction_counter[0] = 3;
//      dist[0] = 21.7;    //estimated
      break;
    case 0:
    junction_counter[0] = 4;
//      dist[0] = 30.2;    //estimated
      break;
    default:
      exit(-1);
  } 
}

/*
 * Given the value, identify its color
 */
void identify_color_spoke(int val){
  if(val == 0){
    Serial.println("***********************RED********************");
  } else if(val ==1){
     Serial.println("***********************GREEN********************");
  } else if(val ==2){
     Serial.println("***********************BLUE********************");
  } else if(val ==3){
     Serial.println("***********************YELLOW********************");
   } else if(val ==4){
     Serial.println("***********************PURPLE********************");
    } else if(val ==5){
     Serial.println("***********************CYON********************");
    } else if(val ==6){
     Serial.println("***********************UPPER WHITE********************");
      } else if(val ==7){
     Serial.println("***********************LOWER WHITE********************");
   } else if(val == -1){
     Serial.println("***********************FAILED TO PICK UP********************");
  }
}


/*
* Special interrupt move inwards (from the spoke to the center) to pick up the coin
* function to move the robot while readining the line, auto correct its position
* picking_coint_junct the junction where to pick the coin
*/
void picking_interrupt_inwards_and_return_to_center(byte dir, long del, int picking_coin_junct,int *hor_or_dia, int *spoke_square_pos, int dest_color[], int round_specify) {
  boolean isNotSlowDown = true;
  boolean isNotSpeedUp = true;
  float *dist;        //pointer to an array of mov_dia or mov_hor
  float backup_dist;
  float forwards_dist;
  float remaining_dis;      //distance from gray to the rest of the edge
  float deliver_gray_cent_dis;  //distance from edge to gray
  float dis_cent;   //distance to center
  if(hor_or_dia[0]){ //if this move diagonally
   dist = mov_dia; //backup diagonal dis
   forwards_dist = forwards_hor_dia_dis[3];
  } else{
   dist = mov_hor;  //back up horizontal dis
   forwards_dist = forwards_hor_dia_dis[2];
  }
  dis_cent = dist[1];
  backup_dist = dist[2]; //backup dis from spoke based on dist pointer
  deliver_gray_cent_dis = dist[5];  
  remaining_dis = dis_cent - deliver_gray_cent_dis;

  int junction_limit = 5;      //junction from the center
  int arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  /*
  * 0: saved_onbit
  * 1: saved_offbit
  * 2: valid_junction
  * 3: junction counter
  * 4: on_count
  * 5: off_count
  *
  */
  int  arr_event_status[] = { 0,1,0,0,0,0 };

  int not_turn_off_picking = 1;             //1 means not to turn off picking coin
  new_vars(dir,  dist[0], del, straight, rightmotors, leftmotors);
  
  while (steps > 0) {
 
     if(steps_counter == fastStepLimit && isNotSpeedUp){
       slow_down(average_delay);
       del = average_delay;
       isNotSpeedUp = false;
     }
     if(steps < slowStepLimit && isNotSlowDown){
      slow_down(slowDelay);
      del = slowDelay;
      isNotSlowDown = false;
    }
    unsigned int position = qtrrc.readLine(sensorValues);
    unsigned int sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, hor_or_dia); //read the sensor  
  if (verbose) {
    Serial.print("\\\\\\\\\\\\\\\\Junction: ");
    Serial.println(arr_event_status[3]);
  }
 
    
      if(arr_event_status[3] == picking_coin_junct && not_turn_off_picking){  //found a picking-up-coin position
        //void mov( byte dir, float dist, long del){
          //junction flag on or off
          unsigned int temp_step = steps;
          int saved_count = arr_event_status[3];
//          steps =0;
          noInterrupts();
          steps = (forwards_dist) * steps_per_inch;   //steps to move upwards
          interrupts();
          slow_down(slowDelay);
          del = slowDelay;
          while(steps > 0){
            position = qtrrc.readLine(sensorValues);
            sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, hor_or_dia); //read the sensor 
            track(pos, del); 
          }
      delay(stopingToPickDelay);
    //******************** Backing up
      PORTL = rev;  //set direction to reverse
      slow_down(slowDelay);
      del = slowDelay;
      steps = (backup_dist +  forwards_dist) * steps_per_inch;   //moving back while reading
      steps_counter = 0;    //reset steps_counter
      isNotSpeedUp = true;
      isNotSlowDown = true;
      while (steps > 0) {
        if (steps_counter == fastStepLimit && isNotSpeedUp) {
          slow_down(backingDelay);
          del = backingDelay;
          isNotSpeedUp = false;
        }
        if (steps < slowStepLimit && isNotSlowDown) {
          slow_down(slowDelay);
          del = slowDelay;
          isNotSlowDown = false;
        }
      }

      PORTL = fwd;  //set direction to fowards
    //******************** Backing up
          delay(stopingToPickDelay);
          strafe_to_center();     //Middle line correction
          delay(stopingToPickDelay);
          picking_up_function_simulator(spoke_square_pos, dest_color);    //psuedo code of picking up function
          arr_event_status[0] =0;
          arr_event_status[1] =1;
          arr_event_status[3] = saved_count;
          arr_event_status[3]--;                        //decrease count
          not_turn_off_picking = 0;                         //turning off checking to pick the coin
          delay(stopingToPickDelay);
          
           
          balance(fwd, slowDelay, (temp_step + backup_dist*steps_per_inch));
      steps_counter = 0;  //reset step counter
      isNotSpeedUp = true;      //it will speed up 
      isNotSlowDown = true;     //it will slow down at the end

          
       
      }
      
      if (arr_event_status[3] < junction_limit) {     //if it not reach final sqaure
        if (sumEvents > 0) {
          //Call a track function if an event occurs
          track(pos, del);
        }
      }
      else {
          arr_event_status[3] =0;     //reset
          
      if (round_specify == 3 && dest_color[0] == 8) {   //if the coin we picked is gray and this is round 3
      //****************move to deliver the gray coin
        steps = deliver_gray_cent_dis * steps_per_inch;    //set the steps
        steps_counter = 0;
        //slow_down(slowDelay);   //make it slow when picking up the coin
        //del = slowDelay;
        //isNotSpeedUp = true;
        //isNotSlowDown = true;
        while (steps > 0) {
          if (steps < slowStepLimit && isNotSlowDown) {
            slow_down(slowDelay);
            del = slowDelay;
            isNotSlowDown = false;
          }
          position = qtrrc.readLine(sensorValues);
          sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, hor_or_dia); //read the sensor 
          track(pos, del);
        }
        delay(stopingToPickDelay);
        //****************move to deliver the gray coin
        //****************deliver coin algorithm here
          noInterrupts();
          dropoff();      
          interrupts(); 


        //****************deliver coin algorithm here

        //move to the rest distance
        steps = remaining_dis * steps_per_inch;    //set the steps
        steps_counter = 0;
        slow_down(slowDelay);   //make it slow when picking up the coin
        del = slowDelay;
        isNotSpeedUp = true;
        isNotSlowDown = true;
        balance(fwd, del, -1);
      }
      else {
        balance(fwd, del, (dis_cent * steps_per_inch)); //set the steps
      }

      }
  }
  
}


/*
* Special interrupt move to deliver to the spoke after picking up
* function to move the robot while readining the line, auto correct its position
* int_or_out 0 for outwards center, 1 for  inwards
*/
void mov_interrupt_deliver_or_comingback(byte dir, long del, int in_or_out, int *hor_or_dia) {
  boolean isNotSlowDown = true;
  boolean isNotSpeedUp = true;
   
  float *dist;        //pointer to an array
  if(hor_or_dia[0]){ //if this move diagonally
   dist = mov_dia; //backup diagonal dis
  } else{
   dist = mov_hor;  //back up horizontal dis
  }
  int arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  /*
  * 0: saved_onbit
  * 1: saved_offbit
  * 2: valid_junction
  * 3: junction counter
  * 4: on_count
  * 5: off_count
  *
  */
  int  arr_event_status[] = { 0,1,0,0,0,0 };
  arr_event_status[3] = 0;
  int junction_limit = 5;
  new_vars(dir, dist[0], del, straight, rightmotors, leftmotors);
//  int just_stop_count =1;
  while (steps > 0) {
     if(steps_counter == fastStepLimit && isNotSpeedUp){
       slow_down(average_delay);
       del = average_delay;
       isNotSpeedUp = false;
     }
    if(steps < slowStepLimit && isNotSlowDown){
      slow_down(slowDelay);
      del = slowDelay;
      isNotSlowDown = false;
    }
    unsigned int position = qtrrc.readLine(sensorValues);
//    int oldVal = arr_event_status[3];
    unsigned int sumEvents = readSensor(arr_position_sensor, sensorValues, arr_event_status, hor_or_dia ); //read the sensor  
  if (verbose) {
    Serial.print("\\\\\\\\\\\\\\\\Junction: ");
    Serial.println(arr_event_status[3]);
  }

    if (in_or_out == OUTWARDS) {   //going out
      if (sumEvents > 0 && arr_event_status[3] < junction_limit) {
        track(pos,  del);
	  }
	 	 
    }
    else {            //coming in
      if (arr_event_status[3] < junction_limit) {     //if it not reach final sqaure
        if (sumEvents > 0) {
          //Call a track function if an event occurs
          track( pos, del);
        }
      }
      else {//if it reached final square
           arr_event_status[3] =0;     //reset
           balance(fwd,del, (dist[1] * steps_per_inch));   //set the steps
          
          if(verbose){
            Serial.print("IM moving because you told me too: ");
            Serial.println(steps);
          }
 
      }
    }
  }
  
}


void vib(float dis, int del){
  PORTL = vibrate;
  steps = dis * steps_per_inch;
  for(int i =0; i< steps; i++){
    PORTL ^= motion;
    delayMicroseconds(del);
  }
}

void pickup(){
  
  servoTurn(upright_angle, servo_hold_delay_upright_angle);         //moves servo to upright position (0) and holds it for 500ms
  servoTurn(pickup_angle , servo_hold_delay_upright_angle);     //moves servo to ground position (140) and holds it for 1000ms

  digitalWrite(relay, LOW); //relay LOW turns magnet ON
  delay(coin_conect_delay);              //delay to give time for coin to connect

  servoTurn(upright_angle,servo_hold_delay_upright_angle);         //ALWAYS end a pickup or dropoff by moving the servo AWAY from the ground position
}

void dropoff(){

  //servoTurn(upright_angle,servo_hold_delay_upright_angle);         //moves servo to upright position (0) and holds it for 500ms
  servoTurn(drop_angle, servo_hold_delay_dropoff_angle);     //moves servo to ground position (140) and holds it for 1000ms
  digitalWrite(relay, HIGH); //relay HIGH turns magnet OFFn 
  
  servoTurn(upright_angle,servo_hold_delay_upright_angle);          //ALWAYS end a pickup or dropoff by moving the servo AWAY from the ground position
}

void servoTurn(int degreeServo, int milliHold)
{
  
  int Cycles = MicroSec2Cycles(milliHold);
  int highDelay = degree2Delay(degreeServo);
  int lowDelay = 20000 - highDelay;
 
  for(int i = 0; i < Cycles; i++)
    {
       if(verbose){
      Serial.println("High Delay: ");
      Serial.print(highDelay);
       }
      digitalWrite(servo, HIGH);
      delayMicroseconds(highDelay);
      digitalWrite(servo, LOW);
      delayMicroseconds(lowDelay);
    }
  
}

int MicroSec2Cycles(int milliHold)
{
    if(verbose){
    Serial.println(milliHold);
    }
    int Cycles = milliHold/20;
    return Cycles;
}

int degree2Delay(float degreeServo)
{
    int highDelay = degreeServo*MicrosecondPerDegree + 750;
    return highDelay;
}
