#ifndef HAVOC_H
#define HAVOC_H

#if (ARDUINO >=100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>

class HAVOC
{
  public:
    HAVOC(); // Default constructor
    HAVOC(int CW, int CCW, int frequency, int sampleRate);  //Constructor: 
    
    void printData(unsigned long time, double targetOri, bool target, float alt, float pres, float temp, int pitch, int roll, int yaw, float temp_g, bool burst); //Prints data to Serial1 based on sampleRate, can be called continuously from loop();
    void Pulse(int throttle, bool dir);     //Fcn to pulse thrusters based on throttle input (percent)

    /*
     * Getter functions: Allow main code to retrieve variables from the payload
     */
    double getAltitude(); // Returns current payload altitude
    double getInput(int targetOri); // Returns orientation error, used at input for PID controller 
    double getVelocity();

    /*
     * Setter functions: Allow main code to modify payload variables
     */
    void setFrequency(int frequency); // Sets the PWM frequency for thrusters 
    //void setError(double error);
    void setSampleRate(int sampleRate); // Sets data samplerate
    void setTorque(double torque);

    void activate(); // Activate stabilization
    void shutdown(); // Sets thrusters closed
  private:
    /*
     * Private functions, used in some public functions
     */
    void bootUp(Adafruit_BNO055 bno); // Initializes serial data stream and sensors
    void calibrateSensors(Adafruit_BNO055 bno); // Calibrates IMU and barometer
    /*
     * Sensors and related variables
     */
    Adafruit_BNO055 bno = Adafruit_BNO055(55);
    Adafruit_BMP3XX bmp;
    sensors_event_t event;
    imu::Vector<3> vel;
    
    int CW = 7; int CCW = 9; int led_green = 22; int led_red = 23; int led_T = 13;
    int currentThrottle = 0;
    int altitude = 0;
    int temperature;
    int sampleRate = 250; //ms
    int frequency = 15;  //Hz
    float torque = 1;
    float seaLevel = 1028; // reference pressure for sealevel (mb)
    unsigned long lasttime = 0;
    
};

#endif
