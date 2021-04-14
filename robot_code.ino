#include <Servo.h>
#include <Wire.h>
#include <LSM303.h>

bool initialized = false;

typedef struct
{
  double x;
  double y;
}Point;

double global_target_radius = 1.5;

typedef struct
{
  Point center;
  double target_elapsed;
}Waypoint;

Point generate_waypoint_origin();

bool turn_to_star = false;
int pump_tick = 0;

double wait_time = 15500;
double halt_time = wait_time + 60000;

// UPHILL /////////////////////////////////
//
//double target_angle_1 = 199*PI/180;
//double target_angle_2 = 90*PI/180;
//Waypoint waypoint_array[] = {{{1000*cos(170*PI/180),1000*sin(170*PI/180)}, 800+wait_time},
//                           {{1000*cos(target_angle_1),1000*sin(target_angle_1)}, 20000},
//                          {{1000*cos(target_angle_2),1000*sin(target_angle_2)}, 100000}};

// DOWNHILL ///////////////////////////////
//Position 1
// 32000 WT 300
//Position 2
// 32250 WT +400
//Position 3
// 32500 WT +400
//Position 4
// 32750 WT 300
//Position 5
// 35000 WT +400
//OTHER
// WT +2300
/*
 * This is where the variables related to the waypoints is stored. During actual use this array
 * will be the only thing being manipulated by the user. NOTHING ELSE WILL BE TOUCHED DURING USE.
 */
double target_angle_1 = 135.8*PI/180;
double target_angle_2 = 60.8*PI/180;
Waypoint waypoint_array[] = {{{1000*cos(100*PI/180),1000*sin(100*PI/180)}, 1600+wait_time},
                          {{1000*cos(target_angle_1),1000*sin(target_angle_1)}, 35000},
                          {{1000*cos(target_angle_2),1000*sin(target_angle_2)}, 100000}};

bool turning = false;
bool positioning = false;

Point robot_position;

Point calculate_position();

int calculate_direction();

double calculate_steer_angle();

void fire_piston();

bool check_hit = false;

double prev_angle;
double prev_real_angle;

double prev_reed_time = 0;

/*
 * Variables related to time.
 */
double current_time = 0;
double time_of_hit = 0;
double prev_time = 0;
double prev_target_angle = 0;

/*
 * Variables used for angle and position calculation.
 */
double origin_angle;
double reading_angle;
double current_angle;
double filtered_angle;
double real_angle;
double wheel_diameter;
double wheel_arc;
int switchState;
int prev_switchState;


/*
 * SETUP variables related to the solenoid and reedswitch.
 */
int switchPin = 4;
const int D2 = 4;   // Disables the shield if low
const int M2DIR = 8;  // Determines the direction of the motor port #2
const int M2PWM = 2;  // Input voltage for motor port #2

/*
 * Variables related to the on and off times for the solenoid and, as a result, speed.
 */
long onTime = 150;
long offTime = 200;
int solenoidPolarity = HIGH;
int solenoidState = LOW;
int prev_solenoidState = LOW;

unsigned long currentMillis;
unsigned long previousMillis = 0;

/*
 * Variables related to steering.
 */
Servo myServo;
int servoPin = 3;
int servoPos = 90;
int current_waypoint_id = 0;

/*
 * Variables related to the angle calculations.
 */
double proportional_gain = 0.25;
double derivative_gain = 50.0;
double delta_angle = 0;
double delta_dangle = 0;

double previous_target_heading = atan(waypoint_array[0].center.y/waypoint_array[0].center.x);

LSM303 compass;

double low_pass_filter(double measured_reading, double filter_strength, double prev);

double filter_strength_1 = 0.75;

void setup(){
  Serial.begin(9600);

  pinMode(switchPin, INPUT_PULLUP);

  pinMode(D2, OUTPUT);
  digitalWrite(D2, HIGH); // Enable shield

  pinMode(M2DIR, OUTPUT);
  digitalWrite(M2DIR, solenoidPolarity);  // Choose solenoid polarity

  pinMode(M2PWM, OUTPUT); // Set the solenoid pin as output

  currentMillis = millis(); // Initialize the start time

  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.writeReg(0x24, 0x74);
  compass.m_min = (LSM303::vector<int16_t>){ -1933,  -1032,  +3688};
  compass.m_max = (LSM303::vector<int16_t>){ +1617,  +2177,  +4052};

  myServo.attach(servoPin);

  compass.read();
  double head = compass.heading();
  origin_angle = head;
  myServo.write(servoPos);

  wheel_diameter = 0.3125;
  double wheel_radius = wheel_diameter/2;
  wheel_arc = PI*wheel_radius;
  robot_position = {0,0};

  delay(wait_time);
  prev_time = millis();
  prev_target_angle = 170;
}

void loop(){
  /*
   * Makes it so everytime the robot starts up, the forward direction will always be 90 degrees from
   * the robot's current position. This makes positioning easier and allows errors within the control
   * code to be adjusted physically by simply turning the robot a CW or CCW if the straight angle dips
   * to far to the left or to the right.
   */
  compass.read();
  double head = compass.heading();
  reading_angle = (head-origin_angle+90)*PI/180;
  current_time = millis();

  /*
   * This could have been better placed in the setup section but it initializes the readings only once.
   * The reason why they weren't placed in the setup function is because the reading calculations were
   * within the loop function. To prevent redundancy I left the statement within the loop function and
   * used a boolean so it could only run once.
   */
  if(!initialized){
  filtered_angle = reading_angle;
  prev_angle = reading_angle;
  prev_real_angle = reading_angle;
  initialized = true;
  }

  /*
   * The firing and angle readings will be based on the check_hit boolean. If the robot has not hit the
   * last waypoint then check_hit is false then the robot will function normally. As soon as the last
   * waypoint has been hit, the robot will stop firing and will no longer move.
   */
  if(!check_hit){
  /*
  * Magnetometer calibrations will be incorrect and will result in angles coming out negative because
  * values are greater than 2*PI, 360 degrees, therefore the statement exists so these readings will
  * be cut down to size.
  *
  * This error also exists because of the reading calculations displacing the angles to be based on the
  * origin angle.
  */
  if(reading_angle > 2*PI){
    reading_angle = reading_angle - 2 * PI;
  }
  if(reading_angle < 0){
    reading_angle = reading_angle + 2 * PI;
  }

  /*
  * Angles are put through a low pass filter so noise coming from the servo and solenoid could be reduced.
  * The low pass filter doesn't necessarily prevent the noise from affecting the readings but it slows down
  * the effect of change on the readings allowing the robot to get consistent readings through the median of
  * the fluctuations.
  */
  filtered_angle = low_pass_filter(reading_angle, filter_strength_1, prev_real_angle);
  current_angle = filtered_angle;

  /*
  * reed switch from the robot is read for position calculations.
  */
  switchState = digitalRead(switchPin);

  /*
  * Steering angle is calculated then put into the servo code to determine angle of steer.
  */
  servoPos = calculate_steer_angle();
 
  myServo.write(servoPos);

  /*
  * The reed switch will sometimes calculate too many times with one magnet pass, therefore this interrupt
  * function exists so it can only be read once instead of 3-4 times with one hit. When the switch state is
  * hit once, there will be a cooldown time before the next hit can be read.
  */
  if(switchState == LOW){
    double current_reed_time = millis();
    if(current_reed_time - prev_reed_time >= 200){
      robot_position = calculate_position();
      prev_angle = current_angle;
      prev_reed_time = current_reed_time;
    }
  }

  /*
  * The pump tick forces the robot to only fire a certain amount of times before it stops. Its main purpose
  * is for the robot to build up enough momentum so it can roll into the final waypoint, which is going to be
  * the star square.
  */
  if(pump_tick <= 8)
  {
    fire_piston();
  }

  /*
  * All the previous state readings are put into the prev variables for use in the next iteration.
  */
  prev_switchState = switchState;
  prev_real_angle = filtered_angle;
  prev_solenoidState = solenoidState;

  /*
  * This statement turns off the robot after 1 min of activation.
  */
  if(current_time >= halt_time)
  {
    check_hit = true;
  }
  }
}

/*
 * Translates the target waypoint to be based on the robot's current position to ease use.
 */
Point generate_waypoint_origin(){
  Point current_waypoint_origin = waypoint_array[current_waypoint_id].center;
  Point modified_origin = robot_position;
  Point modified_waypoint_origin = {current_waypoint_origin.x - robot_position.x, current_waypoint_origin.y - robot_position.y};

  return modified_waypoint_origin;
}

/*
 * Calculates the robot's current position based on the reed switch ticks and the magnetometer reading.
 */
Point calculate_position(){
  Point delta_position;
  Point delta_actual_position;
  Point new_position;

  double new_distance;

  double angle_difference = current_angle - prev_angle;
  double other_angle_difference;
  double actual_angle;

  if(angle_difference == 0){
  /*
  * If the robot's angle difference 0 AKA if the magnetometer reads no change in angle, the robot will use simple trigonometry to
  * calculate its new position.
  */
  delta_actual_position = {wheel_arc*cos(current_angle), wheel_arc*sin(current_angle)};
  }else{
  double pivot_point_distance = abs(wheel_arc/angle_difference);
  delta_position = {pivot_point_distance - pivot_point_distance * cos(angle_difference), pivot_point_distance * sin(angle_difference)};
 
  other_angle_difference = atan(delta_position.x/delta_position.y);
  new_distance = sqrt(sq(delta_position.x) + sq(delta_position.y));
  actual_angle = other_angle_difference + current_angle;
  delta_actual_position = {new_distance * cos(actual_angle), new_distance * sin(actual_angle)};
  }

  new_position = {robot_position.x + delta_actual_position.x, robot_position.y + delta_actual_position.y};
 
  return new_position;
}

double low_pass_filter(double measured_reading, double filter_strength, double prev){
  double new_filtered_reading = (1 - filter_strength) * measured_reading + filter_strength * prev;
  return new_filtered_reading;
}

/*
 * Function calculates how much the servo should turn based on the target's angle from the robot's
 * current position.
 */
double calculate_steer_angle()
{
  Point target_point = generate_waypoint_origin();
  double target_heading;
  double steering;

  /*
   * If statements make sure that the angle readings can go from 0 to 360 degrees based on the
   * y and x values of the target.
   */
  if(target_point.y >= 0 && target_point.x < 0)
  {
  target_heading = atan(target_point.y/target_point.x)+PI;
  }else if(target_point.y < 0 && target_point.x >= 0)
  {
  target_heading = atan(target_point.y/target_point.x)+2*PI;
  }else if(target_point.y < 0 && target_point.x < 0)
  {
  target_heading = atan(target_point.y/target_point.x)+PI;
  }else
  {
  target_heading = atan(target_point.y/target_point.x);
  }

  /*
   * Sets the target angle after the robot has turned towards the initial turning angle.
   */
  if(turning && current_waypoint_id == 1)
  {
  target_heading = target_angle_1;
  }else if(turning && current_waypoint_id == 2)
  {
  target_heading = target_angle_2;
  }

  /*
   * This does the same thing as the above statements except it switches the robot's state from turning
   * to simply adjusting its trajectory to hit the target waypoint.
   */
  if(turning && (current_angle > target_heading) && (prev_real_angle <= target_heading))
  {
  turning = false;
  positioning = true;
  }else if(turning && (current_angle < target_heading) && (prev_real_angle >= target_heading))
  {
  turning = false;
  positioning = true;
  }

  /*
   * If the angle enters a point where one angle is in the 4th quadrant while another is in the 1st, it will force the robot
   * to take the shortest distance between the two. Without it, the robot will take the longer route to the target heading.
   */
  if(current_angle > 270*PI/180 && target_heading < 120*PI/180){
  steering = PI/2 - proportional_gain * (current_angle - target_heading) + (derivative_gain/(current_time - prev_time)) * ((current_angle - prev_real_angle) - (target_heading - prev_target_angle));
  }else if(target_heading > 270*PI/180 && current_angle < 120*PI/180){
  steering = PI/2 - proportional_gain * (current_angle - target_heading) + (derivative_gain/(current_time - prev_time)) * ((current_angle - prev_real_angle) - (target_heading - prev_target_angle));
  }else{
  steering = PI/2 + proportional_gain * (current_angle - target_heading) - (derivative_gain/(current_time - prev_time)) * ((current_angle - prev_real_angle) - (target_heading - prev_target_angle));
  }
  steering = steering*180/PI;

  /*
   * Prevents the robot's servo from steering to far to the left or to the right. This is due to the
   * mechanical restraints of the steering mechanism.
   */
  if(steering < 60)
  {
  steering = 60;
  }else if(steering > 120)
  {
  steering = 120;
  }

  /*
   * Sets the current position based on the robots position. This was added as a slice of life feature as it allowed
   * control of the robot to become easier. After the robot has finished turning to its desired starting angle, it
   * will translate the target waypoint based on the robot's current position making it easier for the controller to
   * set the desired target waypoint.
   */
  if(current_waypoint_id == 1 && !turning && positioning)
  {
  waypoint_array[current_waypoint_id].center.x = waypoint_array[current_waypoint_id].center.x + robot_position.x;
  waypoint_array[current_waypoint_id].center.y = waypoint_array[current_waypoint_id].center.y + robot_position.y;
  positioning = false;
  }else if(current_waypoint_id == 2 && !turning && positioning)
  {
  waypoint_array[current_waypoint_id].center.x = waypoint_array[current_waypoint_id].center.x + robot_position.x;
  waypoint_array[current_waypoint_id].center.y = waypoint_array[current_waypoint_id].center.y + robot_position.y;
  positioning = false;
  }

  if(current_time >= waypoint_array[current_waypoint_id].target_elapsed)
  {
  current_waypoint_id += 1;
  turning = true;

  if(current_waypoint_id == 2)
  {
    turn_to_star = true;
  }
  }

  prev_time = current_time;
  prev_target_angle = target_heading;

  return steering;
}

/*
 * Fire piston functions fires the solenoid based on the ontime and offtimes set before the setup function.
 */
void fire_piston(){
  currentMillis = millis();
 
  if((currentMillis - previousMillis > onTime && solenoidState == HIGH) || (currentMillis - previousMillis > offTime && solenoidState == LOW)) {
  previousMillis = currentMillis;

  if(solenoidState == LOW){
    solenoidState = HIGH;

    /*
    * If the robot is heading towards the final waypoint, the star square, it will start to record the amount of times it fired.
    * When this variable hits a certain ceiling value, it will stop and roll towards the start square.
    */
    if(turn_to_star)
    {
      pump_tick++;
    }
  }else{
    solenoidState = LOW;
  }

  digitalWrite(M2PWM, solenoidState);
  }
}
