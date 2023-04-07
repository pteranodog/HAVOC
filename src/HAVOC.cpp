#include "HAVOC.h"


HAVOC::HAVOC()
{
  pinMode(CW, OUTPUT); pinMode(CCW, OUTPUT); // Set thruster pins to output mode
  pinMode(led_red, OUTPUT); pinMode(led_green, OUTPUT); pinMode(led_T, OUTPUT);
  setFrequency(frequency);

  for(int i=0;i<5;i++){
    digitalWrite(CW, HIGH);
    delay(100);
    digitalWrite(CW, LOW);
    delay(100);
    digitalWrite(CCW, HIGH);
    delay(100);
    digitalWrite(CCW, LOW);
    delay(100);
  }


  bootUp(bno); // Run bootup
  return;
}
HAVOC::HAVOC(int CW, int CCW, int frequency, int sampleRate)
{
  this->CW = CW;    // Clockwise thruster pin
  this->CCW = CCW;  // Counter-Clockwise thruster pin
  pinMode(CW, OUTPUT); pinMode(CCW, OUTPUT); // Set thruster pins to output mode
  pinMode(led_red, OUTPUT); pinMode(led_green, OUTPUT);
  
  setFrequency(frequency); // Set thrust PWM duty cycle frequency
  setSampleRate(sampleRate); // Set data recording sampleRate

  
  bootUp(bno); // Run bootup
  return;
}

/* pulse(int throttleLevel, bool dir)
 *    throttleLevel - PID torque output
 *              dir - Direction of thrust
 * 
 *    analogWrite is used to pulse thrusters. Duty cycle frequency is determined by setFrequency() fcn
 *    analogWrite accepepts inputs from 0 - 255
 */

void HAVOC::Pulse(int throttleLevel, bool dir) 
{
  throttleLevel = throttleLevel*(255/torque); // Calculate duty cycle based on payload instantious torque constant and PID output torque
  
  if(throttleLevel <= 0) currentThrottle = 0;  // avoids error if throttle is set below 0%
  else if(throttleLevel >= 255) currentThrottle = 255; // avoids error if throttle is set above 100%
  else currentThrottle = throttleLevel; // If above are not true, set current throttle to new value
  
  //  if(currentThrottle){
  if (dir){ analogWrite(CCW, currentThrottle); analogWrite(CW, 0);}
  else{ analogWrite(CW, currentThrottle); analogWrite(CCW, 0);}  
}


void HAVOC::printData(unsigned long time, double targetOri, bool target, float alt, float pres, float temp, int pitch, int roll, int yaw, float temp_g, bool burst) // Prints data to OpenLog on Serial1
{
  if(time - lasttime >= sampleRate)
  {
    vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    /* Get a new sensor event */  
    bno.getEvent(&event);
    
   
    //Print Data row
    Serial1.print((String)(time/(float)1000.0) + "," + (String)getInput(targetOri) + "," + (String)targetOri + ","  + (String)-vel.z() + "," + (String)-vel.x() + "," + (String)-vel.y() + ",");
    Serial1.print((String)(event.orientation.x) + "," + (String)(event.orientation.y) + "," + (String)(event.orientation.z) + ",");
    //Serial1.print((String)event.acceleration.x + "," + (String)event.acceleration.y + "," + (String)event.acceleration.z + ",");
    Serial1.print((String)pitch + "," + (String)roll + "," + (String)yaw + "," + (String)temp_g + ",");
    Serial1.print((String)temp + "," + (String)pres + "," + (String)alt + "," + (String)target + "," + (String)burst + "\n");
    lasttime = time;
  }
}

void HAVOC::bootUp(Adafruit_BNO055 bno)
{
  Serial1.begin(9600); // Begin Serial1
  Serial.begin(9600);
  Serial1.println("Class Constructed");
  Serial1.println("Standby for Bootup");
    /* Initialise the IMU sensor */
  if(!bno.begin())
  {
    Serial.println(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while(1){
      delay(50);
      analogWrite(led_red, 200);
      analogWrite(led_T, 200); 
      delay(50);
      analogWrite(led_red, 0);
      analogWrite(led_T, 0); 
    }
  }
  bno.setExtCrystalUse(true);
  //CALIBRATION ALERT
  
  lasttime = millis();
  Serial.println("Sensors detected, begin calibration");
  calibrateSensors(bno); // Run sensor Calibration
  

  // Print Data Header for OpenLog
  Serial1.print("Time(s): , Error Ori.(deg): , Desired Ori.(deg): , (Z) Rotation(deg/s): , (X) Rotation(deg/s): , (Y) Rotation(deg/s): ,");
  Serial1.print("(x) Orient.(deg): , (Y) Orient.(deg): , (Z) Orient.(deg): ,");
  //Serial1.print("(x) Accel.(deg/s^2): , (Y) Accel.(deg/s^2): , (Z) Accel.(deg/s^2): ,");
  Serial1.print("Pitch(deg): , Roll(deg): , Yaw(deg): , Gimbal Temperature(C): ,");
  Serial1.print("Housing Temperature(C): , Pressure(mb): , Altitude(m): , Target (bool):  \n");

  /*
   *  Cycle the thrusters a few times to ensure gas flow
   */


  
}
void HAVOC::calibrateSensors(Adafruit_BNO055 bno) // Calibrate IMU and Barometer
{
  uint8_t test, gyro, acel, mag;
  bool light = false;
  do{
    //Get calibration status
    bno.getCalibration(&test, &gyro, &acel, &mag);
    if(light) analogWrite(led_green, 255);
    else analogWrite(led_green, 0);
    light = !light;
    delay(50);
  }while(gyro != 3);
  Serial.print(F("GYRO CALIBRATED!\n\n=======================================\n\n"));
  Serial1.print(F("GYRO CALIBRATED!\n\n=======================================\n\n"));
 
  

  Serial.println("Sensors Calibrated");
  
}

double HAVOC::getAltitude() // Returns payload altitude based on seaLevel reference
{
  //this->altitude = bmp.readAltitude(seaLevel);
  return altitude;
}
double HAVOC::getInput(int targetOri) // Calculates orientation error relative to target heading.
{ 
  bno.getEvent(&event);
  double input = int(targetOri - (360-(event.orientation.x)) + 540 )%360 - 180; // Calculates angle + or - based on position from target
  //setError(input); 
  return input; // Use as input for PID controller
}
double HAVOC::getVelocity()
{
  imu::Vector<3> vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  return -vel.z();
}

void HAVOC::setFrequency(int frequency) //Sets the duty cycle frequency for the thruster output pins.
{
  this->frequency = frequency;
  analogWriteFrequency(CW, frequency); analogWriteFrequency(CCW, frequency);
}
void HAVOC::setSampleRate(int sampleRate) // Sets data recording samplerate
{
  this->sampleRate = sampleRate;
}
void HAVOC::setTorque(double torque) // Sets instantanious torque constant. Is used when calculating throttle percentage (see pulse() function)
{
  this->torque = torque;
}
void HAVOC::activate()
{
  Serial1.println("============================= BEGIN STABILIZATION ==============================");
  analogWrite(led_green, 200);
}
void HAVOC::shutdown() // Shutdown payload stabilization
{
  analogWrite(CW, 0); analogWrite(CCW, 0); // Command both thrusters to close
  analogWrite(led_green, 0);
  Serial1.println("============================= END STABILIZATION ==============================");
}
