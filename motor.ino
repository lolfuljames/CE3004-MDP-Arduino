#include <ArduinoSort.h>
#include <EnableInterrupt.h>
#include <ArduinoSort.h>
#include <string.h>
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

#define DEBUG false
#define CALIBRATE true
boolean fastest = false;
#define SPEED 90.71
//optimal rpm 60.03, 90.71, 110.72

#define KP1 1.5  //multiplier 1.5
#define KI1 0.7 //0.6
#define KD1 KI1/4  //
#define KP2 1.3 //multiplier 1.6
#define KI2 0.6 //0.2
#define KD2 KI2/4 //0.05
#define INTERVAL 40   //constant interval, minimum 16
#define SIZE 40

#define irF1 A3 //front left
#define irL1 A1 //left front //a1
#define irF2 A2 //front middle
#define irF3 A0 //front right //a0
#define irL2 A4 //left back //a4
#define irR A5 //right sensor (longrange) //a5

char command[50];
int pin1 = 5, pin2 = 11, count; //originally 5! pin broken
int sensors[6]; //left2, left1, front1, front2, front3, right
double rpm1, rpm2, pwm1, pwm2, previous1, previous2;
double prop, deri, inte, distanceCMR1, distanceCMR2;
double diff = 0, prev_diff = 0;
unsigned long time1, time2;
volatile int distance_block;
volatile unsigned long tick1 = 0, tick2 = 0, oldtick1 =0 , oldtick2 =0, current1, current2, distance1, distance2;
volatile double ir_valFnt[SIZE], distanceCMFnt;
int step_count = 0;
boolean sleep = false;

/*
 * RPM = Revolution per minute
 * Speed = PWM used any value between 400 and -400, optimal 250
 * Previous = Previous calcated RPM
 * Duration = Pulse Width (used to count RPM)
 * Time = Used for constant time loop
 */

void setup()
{
  Serial.begin(115200);
//  Serial.println("Dual VNH5019 Motor Shield");
  Serial.println("8rduino connected");
  setup_motor();
  pwm1 = 0;
  pwm2 = 0;
  previous1 = rpm1;
  previous2 = rpm2;
  calibrate();
  rotate_left_degree(90);
  calibrate();
  rotate_right_degree(90);
  calibrate();
//  String test = "Rs6Ls7Rs5Ls9s1Rs1";
//  test.toCharArray(command,50);
  time1 = millis();
  time2 = millis();
  current1 = tick1;
  current2 = tick2;
  reset_pid();
}

void loop()
  {
    if(CALIBRATE){
//        md.setSpeeds(400,400);
//        calibrate();
        go_straight();
//        rotate_left_degree(90);
//        rotate_right_degree(90);
//        delay(1000);
    }
    else{
      if(!sleep) get_command();
      count=0;
      while(command[count] != '\0'){
        switch(command[count]){
          case 'S':
          case 's': 
                  switch(command[count+1]){
                      case '0':
                      case '1':
                      case '2':
                      case '3':
                      case '4':
                      case '5':
                      case '6':
                      case '7':
                      case '8':
                      case '9': go_straight(int(command[count+1]) - 48); break; //convert numbers to decimal
                      default: if(DEBUG) {
                                   Serial.println("S's ERROR INVALID COMMAND: "); 
                                   Serial.println(command[count]); 
                               }
                               break;
                  };
                  count++;
                  break;
          case 'R':
          case 'r':
                  if(DEBUG) Serial.println("rotating right");
                  rotate_right_degree(90);
                  break;
          case 'L':
          case 'l':
                  if(DEBUG) Serial.println("rotating left");
                  rotate_left_degree(90);
                  break;
          case 'C':
          case 'c':
                  if(DEBUG) Serial.println("calibrating");
                  calibrate();
                  break;
          case 'V':
          case 'v':
                  read_all_sensors();
                  break;
          case 'Z':
          case 'z':
                  //stop command from android
                  sleep = true;
                  break;
          case 'F':
          case 'f':
                  fastest = true;
                  break;
          case 'U':
          case 'u':
                  calibrate();
                  rotate_right_degree(90);
                  calibrate();
                  rotate_left_degree(90);
                  calibrate();
                  break;
          default: if(DEBUG) {
                       Serial.print("ERROR INVALID COMMAND: "); 
                       Serial.println(command[count]); 
                   }
                   break;
            }
          count++;
       }
       command[0] = '\0';
    }
  }


void setup_motor(){
  md.init();
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  enableInterrupt(pin1, motor1, CHANGE);
  enableInterrupt(pin2, motor2, CHANGE);
}

/*
 * Method to align the robot to the wall on the LEFT.
 */
void calibrate(){
  //boo
  int i = 0;
//  if(!has_obstacle_left1() || !has_obstacle_left2()) return;
  volatile double distanceCMR1;
  volatile double distanceCMR2;
  while (i<50) {
//      int ir_valL1 = analogRead(irL1);
//      int ir_valL2 = analogRead(irL2);
      int ir_valL1 = analogRead(irL1);
      int ir_valL2 = analogRead(irL2);
      distanceCMR1 = to_short(ir_valL1);
      distanceCMR2 = (to_short(ir_valL2));
      if(distanceCMR1>33) distanceCMR1 = 33;
      if(distanceCMR2>33) distanceCMR2 = 33;
      if(distanceCMR1<0) distanceCMR1 = 0;
      if(distanceCMR2<0) distanceCMR2 = 0;
      i++;
      delay(10);  
      if(distanceCMR2>distanceCMR1){
//        Serial.println("Turn clock");
        if((distanceCMR2-distanceCMR1)>0.05){
//          Serial.print("LEFT");
          setRPM(80,-80);
          delay((distanceCMR2-distanceCMR1)*7);
        }
      }
      else if(distanceCMR1>distanceCMR2){
//        Serial.println("Turn anti-clock");
        if((distanceCMR1-distanceCMR2)>0.05){
          setRPM(-80,80);
          delay((distanceCMR1-distanceCMR2)*7);
        }
    }
    if(DEBUG){
      Serial.print(distanceCMR1);
      Serial.print("              |             ");
      Serial.println(distanceCMR2);
//      Serial.print(ir_valL1);
//      Serial.print("              |             ");
//      Serial.println(ir_valL2);
  }
      brake();
  }

  if(!CALIBRATE){
      if(distanceCMR1 > 12.5 && distanceCMR1 != 33){
        rotate_left_degree(90);
//        Serial.println((distanceCMR1-10)/10);
        go_straight((distanceCMR1-10.5)/10);
        delay(100);
        rotate_right_degree(90);
        brake();
        delay(20);
        calibrate();
      }
      if(distanceCMR1 < 9 && distanceCMR1 != 33){
        rotate_right_degree(90);
        go_straight((10.5-distanceCMR1)/10);
        delay(100);
        rotate_left_degree(90);
        brake();
        delay(20);
        calibrate();
      }
  }
  return;
}

/*
 * Method to check whether there is obstacle in FRONT sensors
 */
int distance_obstacle_front1(){ //front left
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irF1);
  }
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irF1);
  }
  sortArray(ir_valFnt,SIZE);
  distanceCMFnt = to_short(ir_valFnt[SIZE/2]);
  distance_block = int((distanceCMFnt-4)/10.5); //11.5
  if(distance_block > 2 || distance_block < 0) distance_block = 9;
  sensors[2] = (distance_block);
  if(DEBUG){
    Serial.print(" FRONT1: ");
    Serial.print(distanceCMFnt);
  }
  return distance_block;
}

int distance_obstacle_front2(){ //front middle
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irF2);
  }
    for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irF2);
  }
  sortArray(ir_valFnt,SIZE);
  distanceCMFnt = to_short(ir_valFnt[SIZE/2]);
  distance_block = int((distanceCMFnt-1)/10.6); //10.5
  if(distance_block > 2 || distance_block < 0) distance_block = 9;
  sensors[3] = (distance_block);
  if(DEBUG){
    Serial.print(" FRONT2: ");
    Serial.print(distanceCMFnt);
  }
  return distance_block;
}

int distance_obstacle_front3(){ //front right
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irF3);
  }
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irF3);
  }
  sortArray(ir_valFnt,SIZE);
  distanceCMFnt = to_short(ir_valFnt[SIZE/2]);
  distance_block = int((distanceCMFnt-4)/10.4); //11.5
  if(distance_block > 2 || distance_block < 0) distance_block = 9;
  sensors[4] = (distance_block);
  if(DEBUG){
    Serial.print(" FRONT3: ");
    Serial.print(distanceCMFnt);
  }
  return distance_block;
}

/*
 * A method to detect whether there is obstacle on the LEFT sensors.
 */
 
int distance_obstacle_left1(){ //left front
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irL1);
  }
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irL1);
  }
  sortArray(ir_valFnt,SIZE);
  distanceCMFnt = to_short(ir_valFnt[SIZE/2]);
  distance_block = int((distanceCMFnt-6)/10.1);
  if(distance_block > 2 || distance_block < 0) distance_block = 9;
  sensors[1] = (distance_block);
  if(DEBUG){
    Serial.print(" LEFT1: ");
    Serial.print(distanceCMFnt);
  }
  return distance_block;
}

int distance_obstacle_left2(){ //front middle
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irL2);
  }
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irL2);
  }
  sortArray(ir_valFnt,SIZE);
  distanceCMFnt = to_short(ir_valFnt[SIZE/2]);
  distance_block = int((distanceCMFnt-4)/10.1);
  if(distance_block > 2 || distance_block < 0) distance_block = 9;
  sensors[0] = (distance_block);
  if(DEBUG) {
    Serial.print(" LEFT2: ");
    Serial.print(distanceCMFnt);
  }
  return distance_block;
}

/*
 * A method to detect whether there is obstacle on the RIGHT sensors.
 */
 
int distance_obstacle_right(){ //right middle
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irR);
  }
  for(int i=SIZE; i>0; i--){
    ir_valFnt[i] = analogRead(irR);
  }
  sortArray(ir_valFnt,SIZE);
  distanceCMFnt = to_long(ir_valFnt[SIZE/2]);
  distance_block = int((distanceCMFnt-14)/10);
  if(distance_block > 2 || distance_block < 0) distance_block = 9;
  sensors[5] = distance_block;
  if(DEBUG) {
    Serial.print(" RIGHT: ");
    Serial.println(distanceCMFnt);
//    Serial.println(ir_valFnt);
  }
  return distance_block;
}

boolean has_obstacle_left1(){
  if(distance_obstacle_left1() == 0){
    return true;
  }
  return false;
}

boolean has_obstacle_left2(){
  if(distance_obstacle_left2() == 0){
    return true;
  }
  return false;
}

boolean has_obstacle_front1(){
  if(distance_obstacle_front1() == 0){
    return true;
  }
  return false;
}

boolean has_obstacle_front2(){
  if(distance_obstacle_front2() == 0){
    return true;
  }
  return false;
}

boolean has_obstacle_front3(){
  if(distance_obstacle_front3() == 0){
    return true;
  }
  return false;
}

boolean has_obstacle_right(){
  if(distance_obstacle_right() == 0){
    return true;
  }
  return false;
}


/*
 * Method to move the robot continiously using PID.
 */
void go_straight(){
//  if(rpm1 < 10){
//    setRPM(SPEED,SPEED);
//    reset_pid();
//  }
  time2 = time1;
  while((millis()-time2)<INTERVAL){};
  time1 = millis();
  if(DEBUG){
      if((time1-time2)==INTERVAL) Serial.println("YES");
      else Serial.println("NOOOOOOOOOOOOOOOOOOO");
  }
  oldtick1 = current1;
  oldtick2 = current2;
  current1 = tick1;
  current2 = tick2;
  
  previous1 = rpm1;
  previous2 = rpm2;


  rpm1 = ticks_to_rpm(current1-oldtick1);
  rpm2 = ticks_to_rpm(current2-oldtick2);
  if(DEBUG) print_rpm();

  
  rpm1 = getPID1(rpm1, previous1);
  rpm2 = getPID2(rpm2, previous2);

  setRPM(rpm1,rpm2);
 }

void go_straight(double unit){
  int target_tick = distance_to_tick(unit);
  if(DEBUG) {
    Serial.print("Moving forward for ");
    Serial.print(target_tick);
    Serial.println("units");
  }
  if(target_tick<0) return;
  reset_pid();
  step_count += unit;
  distance1 = tick1;
  for(double speed = 0; speed < SPEED; speed+= 20){
    setRPM(speed,speed + 5);
    delay(5);
  }
  while((tick1-distance1)< target_tick){
    go_straight();
  }
  md.setM1Brake(400);
  delay(3);
  brake();
  if(!fastest) delay(100);
//  print_ticks();
 }
//
//void get_reading(){
//  int j = 0;
//  int i = 0;
//  double duration = 0;
//  for(j = 150; j < 400; j+= 2){
//    md.setM1Speed(j);
//    go_straight();
////    delay(100);  
////    Serial.print("ms      - rpm:");
//    Serial.println(rpm1);
//  }
//  Serial.println("---------------------------------------");
//  for(j = 150; j < 400; j+= 2){
//    md.setM2Speed(j);
//    go_straight(); 
////    delay(100);  
////    Serial.print("ms      - rpm:");
//    Serial.println(rpm2);
//  }
//  brake();
//  delay(10000);
//  
//}


/*
 * Method to rotate the robot given parameter in DEGREES
 */
void rotate_left_degree(int degree){
  brake();
  delay(110);
  oldtick1 = tick1;
  setRPM(-88,70); // 6.33V , 6.27, 6.15
//  setRPM(-88,70); // 6.31
  while((tick1 - oldtick1)<(8.851*degree-41.4));
  brake();
  if(!fastest) delay(300);
  else delay(50);
  return;
}

void rotate_right_degree(int degree){
  brake();
  delay(110);
  oldtick1 = tick1;
  setRPM(71,-90); //6.27
  while((tick1 - oldtick1)<(8.851*degree-41.4));
  brake();
  if(!fastest) delay(300);
  else delay(50);
  return;
}
  
/*
 * Method to stop the robot
 *
 */
void brake(){
  md.setBrakes(400, 400);
//  delay(100);
}

/*
 * Method to convert analog reading to actual distance in cm using the SHORT range sensor
 */
double to_short(double ir_value){
  return (27.728 * pow(map(ir_value, 0, 1023, 0, 5000)/1000.0, -1.2045));
}

/*
 * Method to convert analog reading to actual distance in cm using the LONG range sensor
 */
double to_long(double ir_value){
  return (60.374 * pow(map(ir_value, 0, 1023, 0, 5000)/1000.0, -1.16)); 
}


/*
 * Method to set RPM required for straight line motion.
 */
void setRPM(double rpm1, double rpm2){
// EQUATION PUT HERE
    pwm1 = 2.031 * rpm1 + 21.571; //power value 6.16V
    pwm2 = 2.195 * rpm2 + 21.563; //power value 6.17
    md.setM1Speed(pwm1);
    md.setM2Speed(pwm2);
//  print_pwm();
}

/*
 * Method to convert compensated RPM required for straight line motion for left motor.
 */
double getPID1(double rpm, double previous){
  prop = (SPEED-rpm)*KP1; //e(k-1) current error
  inte = (SPEED-previous)*KI1; //e(k-2) previous error
  deri = (SPEED -(rpm + rpm-previous))*KD1; //e(k)
  return (rpm + prop + deri + inte);
}

/*
 * Method to convert compensated RPM required for straight line motion for right motor.
 */
double getPID2(double rpm, double previous){
  prop = (SPEED-rpm)*KP2; //e(k-1)
  inte = (SPEED-previous)*KI2; //e(k-2)
  deri = (SPEED-(rpm + rpm-previous))*KD2; //e(k)
  return (rpm + prop + deri + inte);
}

void print_rpm(){
    Serial.print("RPM1: ");
    Serial.print(rpm1);
    Serial.print(" RPM2: ");
    Serial.print(rpm2);
//    diff = diff - rpm2 + previous2;
    diff = diff - rpm1 + rpm2;
     Serial.print(" DIFFERENCE: ");
     Serial.println(diff);
}

void print_pwm(){
    Serial.print("PWM1: ");
    Serial.print(pwm1);
    Serial.print(" PWM2: ");
    Serial.println(pwm2);
}

void print_ticks(){
    Serial.print("LEFT_TICKS: ");
    Serial.print(current1 - oldtick1);
    Serial.print(" RIGHT_TICKS: ");
    Serial.println(current2 - oldtick2);
}
void motor1(){
  tick1++;
}

void motor2(){
  tick2++;
}

double ticks_to_rpm(unsigned long tick){
  return ((double)tick*2/2249/INTERVAL*1000*60);
}

void reset_pid(){
  time1 = millis();
  current1 = tick1;
  current2 = tick2;
}

void read_all_sensors(){
  delay(100);
  has_obstacle_left2();
  has_obstacle_left1();
  has_obstacle_front1();
  has_obstacle_front2();
  has_obstacle_front3();
  has_obstacle_right();
  Serial.flush();
  Serial.print("aS");
  Serial.print(sensors[0]);
  Serial.print(sensors[1]);
  Serial.print(sensors[2]);
  Serial.print(sensors[3]);
  Serial.print(sensors[4]);
  Serial.print(sensors[5]);
  Serial.println("E");
//  Serial.println(debugcount++);
}

int strlen(String text){
  int i = 0;
  while(text[i] != '\0') i++;
  return i;
}

void get_command(){
    int i = 0;
    while(Serial.available()>0){
       command[i] = Serial.read();
       i++;
       delay(1); //essential delay cause of serial being too slow
    };
    command[i] = '\0';
    if(DEBUG && command[0]!='\0'){
        Serial.print("COMMAND :");
        Serial.println(command);
    }
}

int distance_to_tick(double distance){
  if(distance< 1) return (450*distance);
  if(SPEED == 60.03) return (590*distance - 60);
  else if (SPEED == 90.71) return (600*distance - 92); //520
  else return (580*distance - 90);
}
