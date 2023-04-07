#include <Servo.h>
#include <math.h>
#include "HAVOC.h"

/*
 * ===============================================================================================================================
 * This code is for the 7th flight of HAVOC (April 8th, 2023)
 * This will be the first flight of HAVOC payload equipped with a camera gimbal system
 * Payload will stabilize at an altitude above 18km. Will stabilize rotation based on startup orientation.
 * 
 * Data recording is enabled
 * ===============================================================================================================================
 * - Jaiden Stark
 */


float seaLevel = 1013.25;

//================================================================
bool target = true; // <---------------- CHANGE BEFORE FLIGHT
//================================================================
bool burst = false;

double target_altitude = 13000; // Activation altitude, meters

double deactivate_altitude = 28000; //Approximate burst altitude

double targetOri = 0;   // Target Z-axis angle for azimuth stabilization
double azimuth_bounds = 30; // Target +- bound for azimuth stabilization


// Digital pins defined for Solenoid Valves
#define CW 7
#define CCW 9

// Analog pins defined for LEDs
#define led_red 23
#define led_green 22
#define led_T 13

// PWM Pins defined for gimbal axes
#define pitch_pin 3 //Yellow
#define roll_pin 4  //Blue
#define yaw_pin 5   //Green

// Servo objects for each gimbal axis
Servo g_pitch;
Servo g_roll;
Servo g_yaw ;

// Gimbal angle values
int pitch = 0;
int roll = 0;
int yaw = 0;


// PWM Pins defined for gimbal functions
#define pan_pin 11     // YELLOW 
#define standby_pin 12 // BLUE

Servo g_pan;
Servo g_standby;

int pan_state = 0;
bool standby = false;

#define THERMISTORPIN A0
#define RESISTOR 10000
#define B 3950
#define To 25


HAVOC payload;   // HAVOC Payload class. Object used for payload subfunctions.
                 // Class construction automatically runs some setup commands and IMU calibration

Adafruit_BMP3XX bm; // BMP barometer object. 

void setup() {
  Serial.println("Payload bootup complete"); // Successful class construction
  //Serial.begin(9600);

  // Barometer setup --------------------------------------------------
  if (!bm.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1){
      delay(50);
      analogWrite(led_red, 200);
      delay(50);
      analogWrite(led_red, 0);
    }
  }
  bm.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bm.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bm.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  // Cycle pressure readings to set sea level
  bm.readPressure();
  delay(10);
  bm.readPressure();
  delay(10);
  bm.readPressure();
  delay(10);
  bm.readPressure();
  seaLevel = bm.pressure/100.0;
 //--------------------------------------------------------------------

  // Attach gimbal axes pins to servo objects
  g_pitch.attach(pitch_pin);
  g_roll.attach(roll_pin);
  g_yaw.attach(yaw_pin);

  //pitch = roll = yaw = 45;
  setAxes(0,0,0);
  delay(3000);
  setAxes(45, 0, 0);
  delay(2000);
  setAxes(0,0,0);
  delay(2000);
  setAxes(0, 45, 0);
  delay(2000);
  setAxes(0,0,0);
  delay(2000);
  setAxes(0, 0, 45);
  delay(2000);
  setAxes(0,0,0);
  delay(2000);

  pitch = roll = yaw = 0;

  g_pan.attach(pan_pin);
  g_standby.attach(standby_pin);

  setFcn(g_pan, 1);
  
 //delay(10*1000);   // Optional delay before exiting setup
  
}

void loop() {
  
   // Get time
  unsigned long time = millis();
  
  // Get IMU Error
  float error = payload.getInput(targetOri);
  float velocity = payload.getVelocity();
  
  // Get Barometer Readings
  float alt = bm.readAltitude(seaLevel);
  float temp = bm.temperature; //C
  float pressure = bm.readPressure()/(float)100.0;

  // Get Gimbal Thermistor Reading
  float temp_g = (1.0/(log((RESISTOR/((1023.0/analogRead(THERMISTORPIN)) - 1))/RESISTOR)/B + 1.0/(To + 273.15))) - 273.15; 

  // Check for target altitude
  if(alt >= target_altitude && !target && !burst){
    target = true;                  // Set target boolean to TRUE
    setFcn(g_pan, 1);               // Set camera pan to HOLD,HOLD,HOLD
    pitch = roll = yaw = 0;         // Zero all axes
    setAxes(pitch, roll, yaw);      // Write to all gimbal axes
    payload.activate();             // Activate payload thrusters
  }
  
  // Change gimbal mode when approaching burst
  if(alt >= deactivate_altitude && target &!burst){
    target = false;                // Set target boolean to FALSE
    burst = true;                  // Set balloon burst boolean to TRUE
    setFcn(g_pan, 0);              // Set camera pan to PAN,PAN,PAN
    payload.shutdown();            // Shutdown payload thrusters
  }
  
  if(target){ // If target is true, run stabilization code
    
    if(abs(error) >= azimuth_bounds){ // If outside orientation bounds, correct orientation
      if(error < 0 && velocity < 30){
        digitalWrite(CW, HIGH);
        digitalWrite(CCW, LOW);  
      }
      else if(error > 0 && velocity >-30){
        digitalWrite(CCW, HIGH); 
        digitalWrite(CW, LOW);
      }
      else{ digitalWrite(CCW, LOW); digitalWrite(CW, LOW);}
    }
    else if(abs(velocity) >= 15){ // If within orientation bounds, control speed to 5 deg/sec
      if(velocity < 0){ 
        digitalWrite(CW, HIGH);
        digitalWrite(CCW, LOW);}
      else{ 
        digitalWrite(CCW, HIGH);
        digitalWrite(CW, LOW);}
    }
    else{ digitalWrite(CCW, LOW); digitalWrite(CW, LOW);}
  }

  // Run print command
  payload.printData(time, targetOri, target, alt, pressure, temp, pitch, roll, yaw, temp_g, burst);
  
  // Serial monitor debug print
  //Serial.println((String)(time/1000.0) + ", " + (String)error + ", " + (String)velocity + ", " + (String)pos + ", " + (String)neg + ", " + (String)alt);


}; // End of loop()
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Functions:

/* setAxes() takes input for the three gimbal axes: pitch, roll, yaw
 * This function properly maps and writes these angle setpoints to the gimbal
 */

void setAxes(int pit, int rol, int ya){

  pitch = constrain(pit, -180, 180);
  int servoPitch = map(pitch, -180, 180, 135, 45);

  roll = constrain(rol, -180, 180);
  int servoRoll = map(roll, -180, 180, 135, 45);

  yaw = constrain(ya, -180, 180);
  int servoYaw = map(yaw, -180, 180, 135, 45);

  g_pitch.write(servoPitch);
  g_roll.write(servoRoll);
  g_yaw.write(servoYaw);

};

/* setFcn() accepts a servo object and a desired fcn selection (0-4)
 *  
 *  Use this function to select different gimbal settings on their associated pins
 *  Consult the Input and Functions page: http://www.olliw.eu/storm32bgc-wiki/Inputs_and_Functions
 *  
 */

void setFcn(Servo servo, int fcn){
  int val = 0;

  // Switch-case for setting selection, Input: int 0 - 4  
  switch(fcn){
    case 1:       // Setting #1 -> +333...+500
      val = 500;
      break;
    case 2:       // Setting #2 -> -500...-333
      val = -500;
      break;
    case 3:       // Setting #3 -> +216...+277
      val = 250;
      break;
    case 4:       // Setting #4 -> -277...-216
      val = -250;
      break;
    default:   // Setting #0/default -> -166...+166
      val = 0;
      break;
  }
  val = map(val, -500, 500, 1000, 2000); // map value for 1ms - 2ms RC PWM Pulse
  servo.writeMicroseconds(val); // Write mapped PWM value as microseconds
  
}
