#pragma once
#include "mbed.h"
#define PWR_OOB_ERR 0

namespace MotorTeam
{
    // class for controlling and storing values associated with the motors
    class Motor
    {
        public:
            // constructors
            Motor( PinName pin, PinName tachIn, float tachScale, float pwmScale );
            Motor( PinName pin, PinName tachIn, float tachScale,
                   float pwmScale , float pwmFreq );
            Motor( PinName pin, PinName tachIn, float tachScale, 
                   float pwmScale , float pwmFreq,PinName tx,  PinName rx, int baud );
            ~Motor( ) { };
    
            // getters
            float getPower( );
            float getAngF( );
            float getF   ( );
            float getPwm ( );
            
            // setters
            void setPower( float power );
                
            // class functions
            float calcSpeed  ( );   // calculate the linear speed of the car (only works in the frwd direction)
            float readTach   ( );   // returns the angular frequency of the motor's encoder
            float readTachHz ( );   // returns the frequency of the motor's encoder
            float readTachPWM( );   // returns the theoretical PWM of the Motor
    
            void stop      ( );
            void printPwm  ( );
            void printSpeed( );
    
        private:
            float _power;           // The Dutycycle of the PWM
            float _pwmFreq;         // The frequency of the PWM
            float _speed;           // Speed of the car in m/s
            float _angularFreq;     // The angular frequency of the
            float _wheelRad;        // radius of the wheel on the motor
            float _tachScale;       // scale for the tachometer
            float _pwmScale;
          
            AnalogIn _tachIn;       // Analog reading for the tachometer
            PwmOut   _pwmOut;       // instance of pwm to output a pwm
            Serial   _outStream;    // serial output, for writing to the serial monitor/bt module
    };
    
    class PIController
    {
        public:
            PIController( float proportionalGain, float integralGain, float deltaT );
            ~PIController( ) { };
            
            float calcOutput( float desiredVal, float actualVal );
        private:
            float _errorArea;
            float _output;
            float _deltaT;
            float _proportionalGain;
            float _integralGain;
    };
}
