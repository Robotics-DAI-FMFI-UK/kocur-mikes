#include <Wire.h>
#include "MPU6050.h"
#include "HMC5883L.h"
#include "Servo.h"

#define LED_PIN 13

#define OUTPUT_READABLE_ACCELGYRO

#define DIST1 A6
#define DIST2 A7
#define DIST3 A2

#define CUBE A1

#define ENCB1 A3
#define ENCB2 3
#define ENCA1 4
#define ENCA2 8
#define MOTORA 10
#define MOTORB 9
#define COMPASS_INT_PIN 2  //todo

// motorA = left
// motorB = right

#define WAITING_FOR_START 0
#define EXPECTING_COMMAND 1
#define WAITING_DIGIT1 2
#define WAITING_DIGIT2 3
#define WAITING_SIGN1 4
#define WAITING_DIGIT3 5
#define WAITING_DIGIT4 6
#define WAITING_SIGN2 7
#define WAITING_AZIMUTH1 8
#define WAITING_AZIMUTH2 9
#define WAITING_AZIMUTH3 10
#define WAITING_DIGIT2B 11
#define WAITING_DIGIT4B 12

#define VELOCITY_MEASURING_PERIOD 8

#define DEGREES_PER_SECOND_MULTIPLIER  (VELOCITY_MEASURING_PERIOD * 1000000.0 * 360.0 / 144.0)
#define DEFAULT_LAZINESS 7

#define STARTING_FROM_ZERO_ADDITIVE_BIAS 19

Servo motorA;  
Servo motorB;

HMC5883L compass;
MPU6050 mpu;
uint8_t mpu_initialized;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t mx, my, mz;

// install Pin change interrupt for a pin, can be called multiple times
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

uint8_t current_left_speed;   // 0..180, 90 = stop, 180 = full fwd, 0 = full bwd
uint8_t current_right_speed;  // same

volatile int32_t counterA;
volatile int32_t counterB;
volatile int16_t velocityA;
volatile int16_t velocityB;

volatile int32_t lastVA;
volatile int32_t lastVB;
volatile uint32_t lastVelocityA;
volatile uint32_t lastVelocityB;

uint8_t nextCase[] = { 2, 0, 3, 1 };
volatile uint8_t last_caseA, last_caseB;
volatile uint8_t cases[200];
volatile uint8_t ncases = 0;

uint8_t inpstate;

uint8_t command;
uint8_t velocity_regulation;
uint8_t sending_status;

int16_t wished_speed_left;
int16_t wished_speed_right;
int8_t speed_up_left;
int8_t speed_up_right;
int8_t laziness;

int16_t azimuth_speed;    // -90..90, 0=stop
int16_t azimuth;
uint8_t azimuth_regulation;

inline void updateA()
{
  uint8_t caseA = (digitalRead(ENCA2) << 1) | digitalRead(ENCA1);
  cases[ncases++] = caseA;
  if (ncases == 200) ncases = 0;
  if (nextCase[last_caseA] == caseA) counterA--; else counterA++;
  last_caseA = caseA;
  if (abs(counterA - lastVA) >= VELOCITY_MEASURING_PERIOD)
  {
    uint32_t tm = micros();
    velocityA = (int16_t)(DEGREES_PER_SECOND_MULTIPLIER / (float)(tm - lastVelocityA));
    if (counterA < lastVA) velocityA *= -1;
    lastVelocityA = tm;
    lastVA = counterA;
  }
}

inline void updateB()
{
  uint8_t caseB = (digitalRead(ENCB2) << 1) | digitalRead(ENCB1);
  
  if (nextCase[last_caseB] == caseB) counterB--; else counterB++;
  last_caseB = caseB;
  if (abs(counterB - lastVB) >= VELOCITY_MEASURING_PERIOD)
  {
    uint32_t tm = micros();
    velocityB = (int16_t)(DEGREES_PER_SECOND_MULTIPLIER / (float)(tm - lastVelocityB));
    if (counterB < lastVB) velocityB *= -1;
    lastVelocityB = tm;
    lastVB = counterB;
  }
}

uint8_t occ = 0;  //only for debugging encoders, to be removed

//ENCB1
ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  updateB();
  occ |= 1;
}

//ENCB2
void tickENCB2()
{
  updateB();
  occ |= 2;
}

//ENCA1
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
  updateA();
  occ |= 4;
}  

// ENCA2
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{    
  updateA();
  occ |= 8;
}

int angle_difference(int alpha, int beta)
{
  int diff = beta - alpha;
  if (diff > 180) return diff - 360;
  else if (diff < -180) return diff + 360;
  return diff;
}

void setup_mpu(void)
{
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    mpu_initialized = 0;
    return;
  }

  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);

  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();
  
  // Set threshold sensivty. Default 3.
  mpu.setThreshold(3);

  // Initialize HMC5883L
  if (!compass.begin()) 
  { 
    mpu_initialized = 0;
    return;
  }
  
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(-271, 66);

  mpu_initialized = 1;
}

void setup() 
{
    counterA = 0; counterB = 0; lastVA = 0; lastVB = 0;
    current_left_speed = 90; current_right_speed = 90; 
    velocity_regulation = 0;
    azimuth = 0;
    azimuth_regulation = 0;
    azimuth_speed = 0;
    inpstate = WAITING_FOR_START;
    sending_status = 1;
    laziness = DEFAULT_LAZINESS;
    
    pinMode(ENCA1, INPUT);  // encA1
    pinMode(ENCA2, INPUT);  // encA2
    pinMode(ENCB1, INPUT);  // encB1
    pinMode(ENCB2, INPUT);  // encB2
    pinMode(MOTORA, OUTPUT); // motorA
    pinMode(MOTORB, OUTPUT); // motorB

    pciSetup(ENCA1);    
    pciSetup(ENCB1);
    pciSetup(ENCA2);
    attachInterrupt(digitalPinToInterrupt(ENCB2), tickENCB2, CHANGE);

    pinMode(COMPASS_INT_PIN, INPUT);  // interrupt pin for compass
    motorA.attach(MOTORA);
    motorB.attach(MOTORB);
    
    motorA.write(90);
    motorB.write(90);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    setup_mpu();

    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
}

int8_t requested_left_motor_sign;
int8_t requested_right_motor_sign;
int8_t requested_left_motor_speed;
int8_t requested_right_motor_speed;

void stop_now()
{
  float speedL = current_left_speed;
  float speedR = current_right_speed;
  
  float deltaL = (90.0 - speedL) / 20.0;
  float deltaR = (90.0 - speedR) / 20.0;
  
  for (int i = 0; i < 20; i++)
  {
    motorA.write((int)speedL);
    motorB.write((int)speedR);
    speedL += deltaL;
    speedR += deltaR;
    delay(25);
  }
  motorA.write(90);
  motorB.write(90);
  current_left_speed = 90;
  current_right_speed = 90;
  azimuth_speed = 0;
  azimuth_regulation = 0;
  velocity_regulation = 0;
}

void set_requested_speed(int8_t lt, int8_t rt)
{
  if (lt > 0)
    current_left_speed = 90 - (int8_t)((float)lt * 1.09);  //1.09 - current measured motor strength differences (different HB25 versions and mechanics)
  else
    current_left_speed = 90 - (int8_t)((float)lt * 1.096);    
  current_right_speed = 90 - rt;
  motorA.write(current_left_speed);
  motorB.write(current_right_speed);
}

void set_wished_speed()
{
  wished_speed_left = requested_left_motor_speed;
  wished_speed_right = requested_right_motor_speed;
  if (current_left_speed == 0) 
  {
      if (requested_left_motor_speed > 0) current_left_speed = 90 - STARTING_FROM_ZERO_ADDITIVE_BIAS;
      else if (requested_left_motor_speed < 0) current_left_speed = 90 + STARTING_FROM_ZERO_ADDITIVE_BIAS;
  }
  if (current_right_speed == 0) 
  {
      if (requested_right_motor_speed > 0) current_right_speed = 90 - STARTING_FROM_ZERO_ADDITIVE_BIAS;
      else if (requested_right_motor_speed < 0) current_right_speed = 90 + STARTING_FROM_ZERO_ADDITIVE_BIAS;
  }
}

/* raspberry <-> arduino serial protocol:
 *  
 *  raspberry -> arduino:
 *  
 *  @MsLLzRR    - set absolute motor power -90..90  sLL and zRR are left and right power, where s/z is either '-' or a space, values are in decimal
 *  @S          - stop now (within about 0.5 second, not abruptly)
 *  @R          - reset 32-bit rotation counters of both encoders
 *  @AXXX       - set new azimuth - should be used together with @M/@V command, robot will automatically start moving towards a specified azimuth XXX = 0..360 degrees
 *  @X          - stop following the azimuth (cancel @A mode)
 *  @VsLLLzRRR  - set wished motor velocities (speed-regulated mode) the robot will try to move so that the actual speed of the left (LLL) and right (RRR) wheel
 *                is as specified (in degrees/second - the same unit as reported in the output status)
 *  @-          - stop sending status data                
 *  @+          - resume sending status data (by default they are being sent)
 *  @L XX       - set speed/up slowdown laziness (1/acceleration) - default = 7
 *                
 *   notes:                
 *     @M cancels @V
 *     @V cancels @M and @A  (either regulate direction or velocity)
 *     @A cancels @V
 *     @S cancels @M, @V, and @A
 *     @M can used before @A
 *     @A can be cancelled by @X, but it does not stop the robot (will continue moving in random way as abandonded), 
 *        therefore use @S, or zero @M before or right after @X, or @V instead of @X to switch to different regulation mode
 *        change of speed @M does not cancel @A, and can be used multiple times for a single @A (average of left/right @M motor speeds will be used for @A)
 *     sentences do not need to be terminated by EOL
 *             
 *  arduino -> raspberry (status reporting):
 *  
 *  $countL countR veloL veloR dist1 dist2 dist3 cube heading ax ay az gx gy gz
 *  
 *    countL, countR - counters from the encoders (144 per wheel revolution)
 *    veloL, veloR   - current rotational velocity (in degrees/second) 
 *    dist1 - dist3  - last analog value read from SHARP distance sensors (2Y0A21)
 *    cube           - same type of sensor for detecting dice presence
 *    heading        - current compass heading in degrees
 *    ax, ay, az     - three-axis axelerometer current values (probably to be removed from here)
 *    gx, gy, gz     - three-axis gyroscope current values (also to be removed from here)
 *    
 *    currently the frequency of status reporting is somewhat less than 50Hz, but this may change (you cannot push much more data through 115kbps anyway
 */

void process_char(uint8_t ch)
{
  //if (ch == '!') for (int i = 0; i < 200; i++) Serial.println(cases[i]); else 
  if (ch == '@') inpstate = EXPECTING_COMMAND; 
  else switch (inpstate) {
    case EXPECTING_COMMAND: if (ch == 'S') { stop_now(); inpstate = WAITING_FOR_START; }
                            else if (ch == 'R') counterA = counterB = 0;
                            else if (ch == 'X') azimuth_regulation = 0;
                            else if (ch == 'A') inpstate = WAITING_AZIMUTH1; 
                            else if (ch == '+') sending_status = 1;
                            else if (ch == '-') sending_status = 0;
                            else if ((ch == 'V') || (ch == 'M') || (ch == 'L')) inpstate = WAITING_SIGN1; 
                            command = ch;
                            break;
    case WAITING_SIGN1: requested_left_motor_sign = (ch == '-')?(-1):1; 
                        inpstate = WAITING_DIGIT1;
                        break;
    case WAITING_DIGIT1: requested_left_motor_speed = (ch - '0') * 10;
                         inpstate = WAITING_DIGIT2;
                         break;
    case WAITING_DIGIT2: requested_left_motor_speed += ch - '0';
                         if (command == 'V') inpstate = WAITING_DIGIT2B; 
                         else if (command == 'L') 
                         {
                           laziness = requested_left_motor_speed;
                           inpstate = WAITING_FOR_START;
                         } else inpstate = WAITING_SIGN2;                         
                         break;
    case WAITING_DIGIT2B: requested_left_motor_speed *= 10;
                          requested_left_motor_speed += ch - '0';
                          inpstate = WAITING_SIGN2;
                          break;
    case WAITING_SIGN2: requested_left_motor_speed *= requested_left_motor_sign;
                        requested_right_motor_sign = (ch == '-')?(-1):1; 
                        inpstate = WAITING_DIGIT3;
                        break;                         
    case WAITING_DIGIT3: requested_right_motor_speed = (ch - '0') * 10;                         
                         inpstate = WAITING_DIGIT4;
                         break;
    case WAITING_DIGIT4: requested_right_motor_speed += ch - '0';
                         if (command == 'M') 
                         {
                            inpstate = WAITING_FOR_START;
                            requested_right_motor_speed *= requested_right_motor_sign;
                            azimuth_speed = (int8_t)(((int16_t)requested_left_motor_speed + (int16_t)requested_right_motor_speed) / (int16_t)2);
                            set_requested_speed(requested_left_motor_speed, requested_right_motor_speed);
                            velocity_regulation = 0;
                         }
                         else if (command == 'V')
                         {
                            inpstate = WAITING_DIGIT4B;
                            requested_right_motor_speed *= 10; 
                         }
                         break;    
    case WAITING_DIGIT4B: requested_right_motor_speed += ch - '0';
                          requested_right_motor_speed *= requested_right_motor_sign;
                          inpstate = WAITING_FOR_START;
                          set_wished_speed();
                          velocity_regulation = 1;
                          azimuth_regulation = 0;
                          break;
    case WAITING_AZIMUTH1: azimuth = (ch - '0') * 100; 
                           inpstate = WAITING_AZIMUTH2;
                           break;
    case WAITING_AZIMUTH2: azimuth += (ch - '0') * 10; 
                           inpstate = WAITING_AZIMUTH3;
                           break;
    case WAITING_AZIMUTH3: azimuth += ch - '0'; 
                           azimuth_regulation = 1;
                           velocity_regulation = 0;
                           inpstate = WAITING_FOR_START;
                           break;
  }
}

void regulate_speed()
{
  if (velocityA < wished_speed_left) speed_up_left++;
  else if (velocityA > wished_speed_left) speed_up_left--;
  if (speed_up_left > laziness) 
  {
    if (current_left_speed > 0) current_left_speed--;
    speed_up_left = 0;
  }
  else if (speed_up_left < -laziness)
  {
    if (current_left_speed < 180) current_left_speed++;
    speed_up_left = 0;
  }
  if (velocityB < wished_speed_right) speed_up_right++;
  else if (velocityB > wished_speed_right) speed_up_right--;
  if (speed_up_right > laziness) 
  {
    if (current_right_speed > 0) current_right_speed--;
    speed_up_right = 0;
  }
  else if (speed_up_right < -laziness)
  {
    if (current_right_speed < 180) current_right_speed++;
    speed_up_right = 0;
  }
  motorA.write(current_left_speed);  
  motorB.write(current_right_speed);  
}

void update_speed_according_to_dir(int current_heading)
{
  int delta = angle_difference(azimuth, current_heading);
  //Serial.print("delta="); Serial.print(delta); Serial.print("az="); Serial.print(azimuth); Serial.print("h="); Serial.print(current_heading);Serial.println();

  int16_t lt, rt;
  
  if (delta < -14)
  {
    rt = -azimuth_speed;
    lt = azimuth_speed;    
  }
  else if (delta < -4)
  {
    rt = 2 * azimuth_speed / 3;
    lt = azimuth_speed;
  }
  else if (delta < 0)
  {
    rt = 4 * azimuth_speed / 5;
    lt = azimuth_speed;    
  }
  else if (delta > 14)
  {
    rt = azimuth_speed;
    lt = -azimuth_speed;    
  }
  else if (delta > 4)
  {
    rt = azimuth_speed;
    lt = 2 * azimuth_speed / 3;    
  }
  else if (delta > 0)
  {
    rt = azimuth_speed;
    lt = 4 * azimuth_speed / 5;    
  }
  else
  {
     rt = azimuth_speed;
     lt = azimuth_speed;
  }
  set_requested_speed((int8_t)lt, (int8_t)rt);
}

void loop() 
{
    // read accel/gyro measurements from device
    Vector normAccel = mpu.readNormalizeAccel();
    ax = normAccel.XAxis;
    ay = normAccel.YAxis;
    az = normAccel.ZAxis;

    Vector normGyro = mpu.readNormalizeGyro();
    gx = normGyro.XAxis;
    gy = normGyro.YAxis;
    gz = normGyro.ZAxis;

    // read heading measurements from device
    Vector norm = compass.readNormalize();

    // Calculate heading
    float heading = atan2(norm.YAxis, norm.XAxis);

    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bratislava declination angle is 4'15E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    float declinationAngle = (4.0 + (15.0 / 60.0)) / (180 / M_PI);
    heading += declinationAngle;

    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;

    // Convert to degrees
    int heading_int = (int)(heading * 180/M_PI);

    if (azimuth_regulation) update_speed_according_to_dir(heading_int);
    if (velocity_regulation) regulate_speed();
    uint32_t tm = micros();
    if (tm - lastVelocityA > 250000) velocityA = 0;  // falls to 0 after 0.25s inactivity on encoders
    if (tm - lastVelocityB > 250000) velocityB = 0; 
    if (sending_status)
    {
      Serial.print("$");
      Serial.print(counterA); Serial.print(" ");
      Serial.print(counterB); Serial.print("\t");
      Serial.print(velocityA); Serial.print(" ");
      Serial.print(velocityB); Serial.print("\t");    
      Serial.print(analogRead(DIST1)); Serial.print(" ");
      Serial.print(analogRead(DIST2)); Serial.print(" ");
      Serial.print(analogRead(DIST3)); Serial.print(" ");
      Serial.print(analogRead(CUBE)); Serial.print("\t");
      Serial.print(heading_int); Serial.print("\t");
      Serial.print(ax); Serial.print(" ");
      Serial.print(ay); Serial.print(" ");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print(" ");
      Serial.print(gy); Serial.print(" ");
      Serial.print(gz); Serial.print(" ");
      Serial.print(occ); Serial.print(" ");
      //Serial.print(current_left_speed); Serial.print(" ");
      //Serial.print(current_right_speed); Serial.print(" ");
      Serial.print("\n");
    }

    while (Serial.available()) process_char(Serial.read());
    delay(20);
}

