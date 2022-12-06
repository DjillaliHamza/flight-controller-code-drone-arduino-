/**
 * The software is provided "as is", without any warranty of any kind.
 * Feel free to edit it if needed.
 *
 * @author lobodol <grobodol@gmail.com>
 */

// ---------------------------------------------------------------------------
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_BMP280.h>
////////////////////// PPM CONFIGURATION//////////////////////////
#define channel_number 6  //set the number of channels
#define sigPin 2  //set PPM signal output pin on the arduino
#define PPM_FrLen 27000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
//////////////////////////////////////////////////////////////////
// ------------------- Define some constants for convenience -----------------
/*#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3*/

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis
#define MPU_ADDRESS 0x68  // adresse du MPU-6050
#define FREQ        250   // frequence d'échantillonnage
#define SSF_GYRO    65.5  // facteur de sensibilité

/*#define STOPPED  0
#define STARTING 1
#define STARTED  2*/
/*/ ---------------- Receiver variables ---------------------------------------
// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};

// Used to calculate pulse duration on each channel
volatile unsigned long current_time;
volatile unsigned long timer[4]; // Timer of each channel

// Used to configure which control (yaw, pitch, roll, throttle) is on which channel
int mode_mapping[4];*/
int ppm[channel_number];

const uint64_t pipeIn =  0xE8E8F0F0E1LL;

RF24 radio(9, 10);
Adafruit_BMP280 bmp;
// The sizeof this struct should not exceed 32 bytes
struct MyData {
  byte thrust;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

MyData data;

void resetData() 
{
  // 'safe' values to use when no radio input is detected
  data.thrust = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2= 0;
  
  setPPMValuesFromData();
}

void setPPMValuesFromData()
{
  ppm[0] = map(data.thrust,   0, 255, 1000, 2000);
  ppm[1] = map(data.yaw,      0, 255, 1000, 2000);
  ppm[2] = map(data.pitch,    0, 255, 1000, 2000);
  ppm[3] = map(data.roll,     0, 255, 1000, 2000);
  ppm[4] = map(data.AUX1,     0, 1, 1000, 2000);
  ppm[5] = map(data.AUX2,     0, 1, 1000, 2000);  
  }

/**************************************************/

void setupPPM() {
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0);  //set the PPM signal pin to the default state (off)

  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}
// ----------------------- MPU et BMP variables -------------------------------------
// valeurs brutes du gyro: X, Y, Z
int gyro_raw[3] = {0,0,0};

// decalage gyro: X, Y, Z
long gyro_offset[3] = {0, 0, 0};

// angles calculés du gyro: X, Y, Z
float gyro_angle[3]  = {0,0,0};

// valeurs brutes du accel: X, Y, Z
int acc_raw[3] = {0 ,0 ,0};

// angles calculés du accel: X, Y, Z
float acc_angle[3] = {0,0,0};

// vecteur acceleration total
long acc_total_vector;

// Mouvement angulaire calculé sur chaque axe: Yaw, Pitch, Roll, Thrust
float angular_motions[3] = {0, 0, 0};

float measures[3] = {0, 0, 0};

// MPU's temperature
int temperature;

//BMP pression
float P, P0;

//BMP temperature
float T;

//BMP altitude
float z, z0;

// Init flag set to TRUE after first loop
boolean initialized;
// ----------------------- Variables for servo signal generation -------------
unsigned int  period; // Sampling period
unsigned long loop_timer;
unsigned long now, difference;

unsigned long pulse_length[4]={1000, 1000, 1000, 1000};

// ------------- variables du PID --------------------
float pid_set_points[4] = {0, 0, 0, 0}; // Yaw, Pitch, Roll, Thrust
// Errors
float err[4];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll, Thrust]
float delta_err[4]      = {0, 0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll, Thrust
float err_sum[4]        = {0, 0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll, Thrust]
float previous_err[4] = {0, 0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll, Thrust]
// PID coefficients
float Kp[4] = {4.0, 1.3, 1.3, 3};    // P coefficients in that order : Yaw, Pitch, Roll, Thrust
float Ki[4] = {0.02, 0.04, 0.04, 0.03}; // I coefficients in that order : Yaw, Pitch, Roll, Thrust
float Kd[4] = {0, 18, 18, 12};        // D coefficients in that order : Yaw, Pitch, Roll, Thrust
float pid[4]; 
// ---------------------------------------------------------------------------
/**
 * Status of the quadcopter:
 *   - 0 : stopped
 *   - 1 : starting
 *   - 2 : started
 *
 * @var int
 */
bool status = false;
// ---------------------------------------------------------------------------
int battery_voltage;
// ---------------------------------------------------------------------------
float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}
/**
 * Setup configuration
 */
 void setupMpu6050Registers(){
    // Configure power management
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
    Wire.write(0x00);                    // Apply the desired configuration to the register
    Wire.endTransmission();              // End the transmission

    // Configure the gyro's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1B);                    // Request the GYRO_CONFIG register
    Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
    Wire.endTransmission();              // End the transmission

    // Configure the acceleromter's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
    Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
    Wire.endTransmission();              // End the transmission

    // Configure low pass filter
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1A);                    // Request the CONFIG register
    Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
    Wire.endTransmission();              // End the transmission
}

unsigned long lastRecvTime = 0;
/**
 * Main program loop
 */

/**
 * Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
 * Direct port manipulation is used for performances.
 *
 * This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff.
 *
 * @see https:// www.arduino.cc/en/Reference/PortManipulation
 */
void applyMotorSpeed() {
    // Refresh rate is 250Hz: send ESC pulses every 4000µs
    while ((now = micros()) - loop_timer < period);

    // Update loop timer
    loop_timer = now;

    // Set pins #4 #5 #6 #7 HIGH
    PORTD |= B11110000;

    // Wait until all pins #4 #5 #6 #7 are LOW
    while (PORTD >= 16) {
        now        = micros();
        difference = now - loop_timer;
        for(int i = 0; i < 4; i++){
          if (difference >= ppm[i]) PORTD &= ~(B00010000 << i) ; // Set pin LOW
        }
    }
}

/**
 * Request raw values from MPU6050.
 */
void readSensor() {
    Wire.beginTransmission(MPU_ADDRESS); // Start communicating with the MPU-6050
    Wire.write(0x3B);                    // Send the requested starting register
    Wire.endTransmission();              // End the transmission
    Wire.requestFrom(MPU_ADDRESS,14);    // Request 14 bytes from the MPU-6050

    // Wait until all the bytes are received
    while(Wire.available() < 14);

    acc_raw[X]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[X] variable
    acc_raw[Y]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Y] variable
    acc_raw[Z]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Z] variable
    temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
    gyro_raw[X] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[X] variable
    gyro_raw[Y] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Y] variable
    gyro_raw[Z] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Z] variable
    T = bmp.readTemperature();
    z = bmp.readAltitude(P0);
}

/**
 * Calculate real angles from gyro and accelerometer's values
 */
void resetGyroAngles() {
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}

void calculateAngles() {
    calculateGyroAngles();
    calculateAccelerometerAngles();

    if (initialized) {
        // Correct the drift of the gyro with the accelerometer
        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
    } else {
        // At very first start, init gyro angles with accelerometer angles
        resetGyroAngles();

        initialized = true;
    }

    // To dampen the pitch and roll angles a complementary filter is used
    measures[ROLL]  = measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1;
    measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
    measures[YAW]   = -gyro_raw[Z] / SSF_GYRO; // Store the angular motion for this axis

    // Apply low-pass filter (10Hz cutoff frequency)
    angular_motions[ROLL]  = 0.7 * angular_motions[ROLL]  + 0.3 * gyro_raw[X] / SSF_GYRO;
    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y] / SSF_GYRO;
    angular_motions[YAW]   = 0.7 * angular_motions[YAW]   + 0.3 * gyro_raw[Z] / SSF_GYRO;
}

/**
 * Calculate pitch & roll angles using only the gyro.
 */
void calculateGyroAngles() {
    // Subtract offsets
    gyro_raw[X] -= gyro_offset[X];
    gyro_raw[Y] -= gyro_offset[Y];
    gyro_raw[Z] -= gyro_offset[Z];

    // Angle calculation using integration
    gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
    gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); // Change sign to match the accelerometer's one

    // Transfer roll to pitch if IMU has yawed
    gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
    gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
}

/**
 * Calculate pitch & roll angles using only the accelerometer.
 */
void calculateAccelerometerAngles() {
    // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
    acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));

    // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
    if (abs(acc_raw[X]) < acc_total_vector) {
        acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
    }

    if (abs(acc_raw[Y]) < acc_total_vector) {
        acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
    }
}

/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
 */
void pidController() {
    for(int i = 0; i < 4; i++){
      pid[i] = 0;
    }

    // Initialize motor commands with throttle
    for(int i = 0; i < 4; i++){
      pulse_length[i] = 1000;
    }
        // PID = e.Kp + ∫e.Ki + Δe.Kd
        for(int i = 0; i < 4; i++){
          pid[i] = err[i]*Kp[i]+err_sum[i]*Ki[i]+delta_err[i]*Kd[i];
        }

        // Keep values within acceptable range. TODO export hard-coded values in variables/const
        for(int i = 0; i < 4; i++){
          pid[i] = minMax(pid[i], -400, 400);
        }

        // Calculate pulse duration for each ESC
        pulse_length[0] += pid[3] - pid[2] - pid[1] + pid[0];
        pulse_length[1] += pid[3] + pid[2] - pid[1] - pid[0];
        pulse_length[2] += pid[3] - pid[2] + pid[1] - pid[0];
        pulse_length[3] += pid[3] + pid[2] + pid[1] + pid[0];
    for(int i = 0; i < 4; i++){
          pulse_length[i] = minMax(pulse_length[i], 1100, 2000);
    }
}

/**
 * Calculate errors used by PID controller
 */
void calculateErrors() {
    // Calculate current errors
    err[YAW]   = angular_motions[YAW]   - pid_set_points[YAW];
    err[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
    err[ROLL]  = angular_motions[ROLL]  - pid_set_points[ROLL];
    err[3] = z - pid_set_points[3];

    // Calculate sum of errors : Integral coefficients
    err_sum[YAW]   += err[YAW];
    err_sum[PITCH] += err[PITCH];
    err_sum[ROLL]  += err[ROLL];
    err_sum[3]  += err[3];

    // Keep values in acceptable range
    err_sum[YAW]   = minMax(err_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
    err_sum[PITCH] = minMax(err_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
    err_sum[ROLL]  = minMax(err_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);
    err_sum[3]  = minMax(err_sum[3],  -400/Ki[3],  400/Ki[3]);

    // Calculate error delta : Derivative coefficients
    delta_err[YAW]   = err[YAW]   - previous_err[YAW];
    delta_err[PITCH] = err[PITCH] - previous_err[PITCH];
    delta_err[ROLL]  = err[ROLL]  - previous_err[ROLL];
    delta_err[3]  = err[3]  - previous_err[3];

    // Save current error as previous_error for next time
    previous_err[YAW]   = err[YAW];
    previous_err[PITCH] = err[PITCH];
    previous_err[ROLL]  = err[ROLL];
    previous_err[3]  = err[3];
}

void calibrateMpu6050() {
    int max_samples = 2000;

    for (int i = 0; i < max_samples; i++) {
        readSensor();

        gyro_offset[X] += gyro_raw[X];
        gyro_offset[Y] += gyro_raw[Y];
        gyro_offset[Z] += gyro_raw[Z];
        // Generate low throttle pulse to init ESC and prevent them beeping
        PORTD |= B11110000;      // Set pins #4 #5 #6 #7 HIGH
        delayMicroseconds(1000); // Wait 1000µs
        PORTD &= B00001111;      // Then set LOW
        // Just wait a bit before next loop
        delay(3);
    }

    // Calculate average offsets
    gyro_offset[X] /= max_samples;
    gyro_offset[Y] /= max_samples;
    gyro_offset[Z] /= max_samples;
}
/**
 * Calculate PID set points on axis YAW, PITCH, ROLL
 */
void calculateSetPoints() {
    pid_set_points[YAW]   = calculateYawSetPoint(ppm[1], ppm[0]);
    pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], ppm[2]);
    pid_set_points[ROLL]  = calculateSetPoint(measures[ROLL], ppm[3]);
}

/**
 * Calculate the PID set point in °/s
 *
 * @param float angle         Measured angle (in °) on an axis
 * @param int   channel_pulse Pulse length of the corresponding receiver channel
 * @return float
 */
float calculateSetPoint(float angle, int channel_pulse) {
    float level_adjust = angle * 16; // Value 16 limits maximum angle value to ±31°
    float set_point    = 0;

    // Need a dead band of 8µs for better result
    if (channel_pulse > 1504) {
        set_point = channel_pulse - 1504;
    } else if (channel_pulse <  1496) {
        set_point = channel_pulse - 1496;
    }

    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}

/**
 * Calculate the PID set point of YAW axis in °/s
 *
 * @param int yaw_pulse      Receiver pulse length of yaw's channel
 * @param int throttle_pulse Receiver pulse length of throttle's channel
 * @return float
 */
float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
    float set_point = 0;

    // Do not yaw when turning off the motors
    if (throttle_pulse > 1050) {
        // There is no notion of angle on this axis as the quadcopter can turn on itself
        set_point = calculateSetPoint(0, yaw_pulse);
    }

    return set_point;
}

/**
 * Compensate battery drop applying a coefficient on output values
 */
void compensateBatteryDrop() {
    if (isBatteryConnected()) {
        for(int i = 0; i < 4; i++){
          pulse_length[i] += pulse_length[i] * ((1240 - battery_voltage) / (float) 3500);
        }
    }
}

bool isBatteryConnected() {
    // Reduce noise with a low-pass filter (10Hz cutoff frequency)
    battery_voltage = battery_voltage * 0.92 + (analogRead(A0) + 65) * 0.09853;

    return battery_voltage < 1240 && battery_voltage > 800;
}

void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis();
  }
}

float calculateThrustsetpoint(int v){
  v /= 100;
  z0 += v;
  if(z0 >= 20){
    z0 = 20;
  }
  pid_set_points[3] = z0;
  }
// #error Delete this line befor you cahnge the value (clockMultiplier) below
#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1_COMPA_vect){
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; // turn pin 2 off. Could also use: digitalWrite(sigPin,0)
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; // turn pin 2 on. Could also use: digitalWrite(sigPin,1)
    state = true;

    if(cur_chan_numb >= channel_number) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

void setup() {
    //Wire.begin();//debut I2C
    //TWBR = 12; // Horloge I2C 400kHz.

    // Turn LED on during setup
    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
    digitalWrite(13, HIGH);
    digitalWrite(12, LOW);
   /* if (!bmp.begin()) {
      digitalWrite(12, HIGH);
      while (1);
    }*/
    // Set pins #4 #5 #6 #7 as outputs
    for(int i = 4; i < 8; i++){
      pinMode(i, OUTPUT);
    }

   // setupMpu6050Registers();

   // calibrateMpu6050();

    period = (1000000/FREQ) ; // Sampling period in µs
  resetData();
  setupPPM();
  
  // Set up radio module
  radio.begin();
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  radio.setAutoAck(false);

  radio.openReadingPipe(1,pipeIn);
  radio.startListening();
  //P0 = bmp.readPressure();
  //z0 = bmp.readAltitude(P0);
    // Initialize loop_timer
    loop_timer = micros();

    // Turn LED off now setup is done
    digitalWrite(13, LOW);
}

void loop() {
    // 1. First, read raw values from MPU-6050
    //readSensor();

    recvData();

  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    resetData();
  }
  if(data.AUX1){
    status = true;
  }else if(data.AUX2){
    status = false;
  }
  if(status){
  setPPMValuesFromData();
  }else{
    for(int i = 0; i < 4; i++){
      ppm[i]=1500;
      }
      //z0 = z;
    }
    // 2. Calculate angles from gyro & accelerometer's values
    //calculateAngles();

    // 3. Calculate set points of PID controller
    //calculateSetPoints();

    //calculateThrustsetpoint(data.thrust - 128);

    // 4. Calculate errors comparing angular motions to set points
    //calculateErrors();
        // 5. Calculate motors speed with PID controller
        //pidController();

        //compensateBatteryDrop();

    // 6. Apply motors speed
    applyMotorSpeed();
}
