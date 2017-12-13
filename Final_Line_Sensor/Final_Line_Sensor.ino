/*
 * Advanced and almost finialized version of line sensor
 * 
 * @Version 12/04/2017
 */
#include <Adafruit_TCS34725.h>
#include <QTRSensors.h>
#include <math.h> 
byte motion = B01010101;
byte r45 = B01000001;
byte frontmotors = B01010000;
byte rearmotors = B00000101;
byte rightmotors = B00010001;
byte leftmotors = B01000100;
byte l45 = B00010100;

byte rotleft = B10000000;
byte rotright = B00101010;
byte fwd = B00001000;
byte rev = B10100010;
byte strafeleft = B10001010;
byte straferight = B00100000;
float steps_per_inch = 215.602;
float steps_per_deg = 26.3;
float start_dis = .25;
float straight = .993;//decrease to turn right, increase to turn left 
double length_robot_center = 6; //inch
float baseTurnRatio = .2;
int bounce_left;   //center
int bounce_right;    //center
float pos;      //new position
float oldpos;   //old position
        //Used for CCT interupt
volatile unsigned int countCompareValue;
volatile boolean flag1;
volatile int cnt;
volatile int steps = 0;
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
//int sensormin;
//int sensormax;

//Set up a variable 
//In the interupt rountine, which used measure dis

void setup() {
  // put your setup code here, to run once: 
  DDRL = B11111111;//turn on the driving mode
  delay(500);
  Serial.begin(9600);
  delay(1000);

  //ctc1_setup();//setup the timer counter
  //autocal();      //move up and stop and up and stop and turn right to calibate
  selfCal();
  delay(200);

  long mydelay = 650;
    
  //checking variable speed
  //new_vars(fwd, 72, 800, straight, rightmotors, leftmotors);
  //Path 
  mov(fwd, 42, mydelay);
//  turn(rotleft, 135, 800);

//  for (int i = 0; i< 4; i++) {
//    mov_interrupt(fwd, 50.9, 800);
//    turn(rotleft, 180, 800);
//    mov_interrupt(fwd, 50.9, 800);
//    turn(rotleft, 90, 800);
//  }
    
    //Red
    turn(rotleft, 45, mydelay);
    mov_interrupt_down(fwd, 49, mydelay);
    turn(rotleft, 180, mydelay);
    mov_interrupt_up(fwd, 49, mydelay);
    //Cyan
    turn(rotleft, 90, mydelay);
    mov_interrupt_down(fwd, 49, mydelay);
    turn(rotleft, 180, mydelay);
    mov_interrupt_up(fwd, 49, mydelay);
    //Purple
    turn(rotleft, 135, mydelay);
    mov_interrupt_down(fwd, 30, mydelay);
    turn(rotleft, 180, mydelay);
    mov_interrupt_up(fwd, 30, mydelay);
    //Yellow
    turn(rotleft, 135, mydelay);
    mov_interrupt_down(fwd, 49, mydelay);
    turn(rotleft, 180, mydelay);
    mov_interrupt_up(fwd, 49, mydelay);
    //Blue
    turn(rotleft, 90, mydelay);
    mov_interrupt_down(fwd, 49, mydelay);
    turn(rotleft, 180, 700);
    mov_interrupt_up(fwd, 49, mydelay);
    //Green
    turn(rotleft, 135, mydelay);
    mov_interrupt_down(fwd, 30, mydelay);
    turn(rotleft, 180, mydelay);
    mov_interrupt_up(fwd, 30, mydelay);
    //White
    turn(rotright, 90, mydelay);
    mov(fwd, 42, mydelay);
    
//    turn(rotleft, 180, 800);
//    mov_interrupt(fwd, 50.9, 800);
  /*
  //Replace mov with acceleration
  mov(fwd, 42, 800);
  turn(rotleft, 135, 800);

  for (int i = 0; i< 4; i++) {
    acceler_interrupt(fwd, 50.9, 800, 2);
    turn(rotleft, 180, 800);
    acceler_interrupt(fwd, 50.9, 800, 2);
    turn(rotleft, 90, 800);

  }
  */
//  turn(rotleft, 45, 800);
//  acceler_interrupt(fwd, 42, 800, 4);
  







}


void loop() {

}

/********************FUNCTION**********************/
//Count to match compare -- CTC setup
//delay 500 secounds
void ctc1_setup() {
  noInterrupts();
  TCCR1A = 0;  // clear counter control register
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 8000; // compare match register – 500 usecond delay
          // countCompareValue = del * 16
          // countCompareValue = 16000000 / prescaler / desired frequency
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
  }
}

//4: Master
ISR(TIMER4_COMPA_vect) { // timer compare ISR
  if (steps > 0) {
    PORTL ^= masterWheels;
    masterCount++;
    steps--;
  }
}

/*void mov_i(byte dir, float dist, long del) {//move with i
  PORTL = dir;
  float stepf = dist * steps_per_inch;
  steps = stepf;
  //  T time interval between consecutive interrupt requests, the value pf OCRx register
  //must be set by the user:
  OCR1A = (16 * del) - 1;//get the second delay by using formula OCRx + 1 = T / ( P / (16 * 10^6) ) 
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
}
*/
void autocal() {//auto calibration using interruption
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  int loop_counter = 1;
  while (loop_counter-->0) {
    mov_i(fwd, 6, 1000);
    while (steps > 0) {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)    
    }
    delay(200);
    mov_i(fwd, 3, 1000);
    while (steps > 0) {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }
    delay(200);
    digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

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

  //turn(rotright, 90,1000);

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
    OCR1A = (del * 16) -1;
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    interrupts();
    //************************Set Variables********************************
}

 
/*
* function to move the robot while readining the line, auto correct its position
*/
void mov_interrupt_down(byte dir, float dist, long del) {
  //double turnRatio = straight;//the turning ratio
  //int off_which_direction = 0; //determine which to turn
  //int arr_on_event[] = { 0,0,0,0,0,0,0,0 };
  //int arr_off_event[] = { 0,0,0,0,0,0,0,0 };
  //int steps_while_turning = -1;   //used to block the signal while driving
  //  new_vars(fwd, 12, del, straight, rightmotors, leftmotors);
  int arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  int old_arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  int  arr_sum_of_on_status[] = { 0,0 };
  int  arr_event_status[] = { 0,0 };
  //int del = 800;
  new_vars(dir, dist, del, straight, rightmotors, leftmotors);
  int j_count_1_to_0 = 0;		//junction counter val changed from 1 to 0
  int j_count_0_to_1 = 0;    //junction counter val changed from 0 to 1
   boolean is_5_junc_1_to_0 = false;  //if it true then turn off track
   boolean is_5_junc_0_to_1 = false;
  //   unsigned int position;
  while (steps > 0) {
	 
    unsigned int position = qtrrc.readLine(sensorValues);
    int arr_on_event[] = { 0,0,0,0,0,0,0,0 };
    int arr_off_event[] = { 0,0,0,0,0,0,0,0 };
    unsigned int sumEvents = readSensor(arr_position_sensor, old_arr_position_sensor, arr_sum_of_on_status, sensorValues, arr_on_event, arr_off_event, arr_event_status); //read the sensor
	if (arr_event_status[3] > arr_event_status[4]) {
		j_count_1_to_0++;
	}
  if (arr_event_status[3] < arr_event_status[4]) {      
   j_count_0_to_1++;
  }
	arr_event_status[4] = arr_event_status[3];
	/*Serial.print("OLd NUmber of on OLd: \t");
	Serial.print(arr_event_status[3]);
	Serial.println();
	Serial.print("New NUmber of on New: \t");
	Serial.print(arr_event_status[4]);
	Serial.println();
 Serial.print("\\\\\\\\\\\\\\\\\\\\\\\\COunting: \t");
 Serial.print(j_count_1_to_0);
 Serial.println();*/
	if (j_count_0_to_1 > 3) {
		is_5_junc_0_to_1 = true;
		j_count_0_to_1 = 0;
	}
    if (sumEvents > 0 && is_5_junc_0_to_1) {
      //Call a track function
      //if an event occurs
      //  Change speed ratio of the motors
      //  Call function that changes variable speed
      //track(int* arr_event_status, double position, unsigned int del)
      track(arr_event_status, pos, del);
    }


    //char mycase = '0';

    //switch (mycase) {
    //case '0':           //read sensor 
    //  unsigned int sumEvents = readSensor(arr_position_sensor, arr_sum_of_on_status, sensorValues, arr_on_event, arr_off_event, arr_event_status);
    //  break;
    //case '1':
    //  //Call findPosition
    //  break;
    //case '2':
    //  //Call findDirection
    //  break;
    //case '3':
    //  //Call turnToCenter
    //  break;
    //case '4':
    //  //Call convergeToCenter
    //  break;
    //case '5':
    //  //Call track
    //  break;
    //default:
    //  
    //}
/*  Serial.print("INput:\t");
  int senscount=0;
  for (unsigned char i = 0; i < NUM_SENSORS; i++) {
    Serial.print(arr_position_sensor[i]);
    if(arr_position_sensor[i]) senscount++;
    
    Serial.print('\t');
  }
  Serial.print(senscount);
  Serial.println();
  */
//  Serial.print("On Events:\t");
//  for (unsigned char i = 0; i < NUM_SENSORS; i++) {
//    Serial.print(arr_on_event[i]);
//    Serial.print('\t');
//  }
//  Serial.println();
//   
  }
}

/*
* function to move the robot while readining the line, auto correct its position
*/
void mov_interrupt_up(byte dir, float dist, long del) {
  //double turnRatio = straight;//the turning ratio
  //int off_which_direction = 0; //determine which to turn
  //int arr_on_event[] = { 0,0,0,0,0,0,0,0 };
  //int arr_off_event[] = { 0,0,0,0,0,0,0,0 };
  //int steps_while_turning = -1;   //used to block the signal while driving
  //  new_vars(fwd, 12, del, straight, rightmotors, leftmotors);
  int arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  int old_arr_position_sensor[] = { 0,0,0,0,0,0,0,0 };
  int  arr_sum_of_on_status[] = { 0,0 };
  int  arr_event_status[] = { 0,0 };
  //int del = 800;
  new_vars(dir, dist, del, straight, rightmotors, leftmotors);
  int j_count_1_to_0 = 0;    //junction counter val changed from 1 to 0
  int j_count_0_to_1 = 0;    //junction counter val changed from 0 to 1
   boolean is_5_junc_1_to_0 = false;  //if it true then turn off track
   boolean is_5_junc_0_to_1 = false;
  //   unsigned int position;
  while (steps > 0) {
   
    unsigned int position = qtrrc.readLine(sensorValues);
    int arr_on_event[] = { 0,0,0,0,0,0,0,0 };
    int arr_off_event[] = { 0,0,0,0,0,0,0,0 };
    unsigned int sumEvents = readSensor(arr_position_sensor, old_arr_position_sensor, arr_sum_of_on_status, sensorValues, arr_on_event, arr_off_event, arr_event_status); //read the sensor
  if (arr_event_status[3] > arr_event_status[4]) {
    j_count_1_to_0++;
  }
  if (arr_event_status[3] < arr_event_status[4]) {      
   j_count_0_to_1++;
  }
  arr_event_status[4] = arr_event_status[3];
  if (j_count_0_to_1 > 4) {
    is_5_junc_0_to_1 = true;
    j_count_0_to_1 = 0;
  }
    if (sumEvents > 0) {
      //Call a track function
      //if an event occurs
      //  Change speed ratio of the motors
      //  Call function that changes variable speed
      //track(int* arr_event_status, double position, unsigned int del)
      track(arr_event_status, pos, del);
    }
  }
}
/*
* Accelerate while following the line
* @param N number of step?
*/
void acceler_interrupt(byte dir, float dist, long del, int N) {
  float total_dis = N*(N + 1) / 2 * 3 * start_dis;
  if (total_dis > dist) {
    float m = sqrt((dist / start_dis) * 2 / 3);
    N = m;
  }

  float mid_dis = dist - start_dis * 3 * N*(N + 1) / 2;
  for (int i = 1; i <= N; i++) {
    //    mov(dir, start_dis * i, del/i);
    mov_interrupt_up(dir, start_dis * i, del / i);

  }

  //   mov(dir, mid_dis, del/N);
  mov_interrupt_up(dir, mid_dis, del / N);
  for (int i = N; i>0; i--) {
    //    mov(dir, start_dis * 2*i, del/i);
    mov_interrupt_up(dir, start_dis * 2 * i, del / i);
  }
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

  long masterCount = 0; //set up master count
  noInterrupts();
  // long slaveCount =0;
  // long stepCount =0;
  OCR4A = 16 * del - 1;//setup speed for master
  TCNT4 = 0;//reset
  float temp = del * ratio;
  long slaveDelay = temp;
  OCR3A = slaveDelay * 16 - 1;//setup speed for slave 
  TCNT3 = 0;//reset
  steps = stepf;
  interrupts();

}

 

/*
*  a method determine which sensor are on
*Loop through all 8 sensors testing each sensor “if (sensorValues[i] == 1000)”
*Count #on
*Mark on sensors
*Sum #on
*Sum index of on sensors

*@param arr_on_sensor the array of sensor that will be marked on 1 and off 0
*@param arr_sum_of_on_status  the array of status that contains sumOfOn and sumIndexOfON
*@param senVal  the Value of the sensors
*@param arr_off_event the array of on events on-to-off
*@param arr_on_event  the array of on events off-to-on
@param arr_event_status the array of status that contain num of on-to-off, and num of off-to-on, num of new-on, number of old-on
*/

unsigned int readSensor(int* arr_position_sensor, int* old_arr_position_sensor, int* arr_sum_of_on_status, unsigned int* senVal, int* arr_on_event, int* arr_off_event, int* arr_event_status) {
  //Case 1, if arr_event is null

  unsigned int SumOfOn = 0;      //sum of sensors that are on
  unsigned int sumIndexOfOn = 0;      //Sum index of on sensors

  unsigned int eventOn = 0;
  unsigned int eventOff = 0;
  for (int i = 0; i < 8; i++) {
    if (senVal[i] == 1000) {
      arr_position_sensor[i] = 1;
      sumIndexOfOn += i;
      SumOfOn++;
    }
    else {
      arr_position_sensor[i] = 0;
    }
    if (SumOfOn != 0) {
      pos = sumIndexOfOn / SumOfOn;
    }
    else {
      pos = oldpos;
    }
  }
  
  if (SumOfOn >= 4) {			//if there are more than 3 sensor are on
	  arr_event_status[3] = 1;
  } else{
	  arr_event_status[3] = 0;
  }
  

  for (int i = 0; i< 8; i++) {
    //      Serial.println("readInputLine");
    if (arr_position_sensor[i] == 1) {
      if (old_arr_position_sensor[i] != 1) {//there is an event if the old one is not like new one
        eventOn++;
        arr_on_event[i] = 1;
      }
      else {
        arr_on_event[i] = 0;
      }
    }
    else {
      if (old_arr_position_sensor[i] != 0) {//there is an event if the old one is not like new one
        eventOff++;       //increase event num OFF
        arr_off_event[i] = 1;        //there is some event

      }
      else {
        arr_off_event[i] = 0;
      }
    }
  }
  arr_sum_of_on_status[0] = SumOfOn;
  arr_sum_of_on_status[1] = sumIndexOfOn;
  arr_event_status[0] = eventOn;
  arr_event_status[1] = eventOff;
  for (int i = 0; i < 8; i++) {
    old_arr_position_sensor[i] = arr_position_sensor[i];
  }
  oldpos = pos;
  return eventOn + eventOff;        //return number of events
}

/*
Wait for event
Read pos
Adjust speeds
stepRatio = (pos – 3.5) * baseTurnRatio
turnRatio = straight + stepRatio
OCR3A = del *16
OCR4A = turnRatio*del*16
Reset CNTx registers

Possible to reduce baseTurnRatio once converged

*/
void track(int* arr_event_status, double position, unsigned int del) {
  //Wait for event
  if (arr_event_status[0] + arr_event_status[1] > 0) {  //if there is some event
    double stepRatio = ((double)(position - 3.5)) * baseTurnRatio;
    double turnRatio = straight - stepRatio;
    //************************Set Variables********************************
    noInterrupts();
    TCNT4 = 0;
    TCNT3 = 0;
    OCR4A = del * 16;
    OCR3A = turnRatio * 16 * del;//don't want to messup with steps--> change speed for slave only
    interrupts();
    //************************Set Variables********************************

  }

}
 
void turn(byte dir, float deg, long del) {
  PORTL = dir;
  float stepf = deg*steps_per_deg;
  long steps = stepf;

  for (long i = 0; i< steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motion; //toggle the step signal to continue moving
  }
}
 

void selfCal() {
  int calmin[] = { 124, 64, 124, 124, 124, 128, 124, 192 };
  for (int i = 0; i < 10; i++) {
    qtrrc.calibrate();
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    qtrrc.calibratedMinimumOn[i] = calmin[i];
    qtrrc.calibratedMaximumOn[i] = 2500;
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
void mov( byte dir, float dist, long del){
  PORTL = dir;
  float stepf=dist*steps_per_inch;
  long steps=stepf;
  
for(long i=0; i< steps;i++){
delayMicroseconds(del);
PORTL ^= motion; //toggle the step signal to continue moving

}
 
}
 
