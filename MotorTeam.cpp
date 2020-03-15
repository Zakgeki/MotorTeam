#include "MotorTeam.h"
#include <iostream>

#define DEF_BAUD_RATE 9600
#define DEF_RADIUS    25e-3 
#define DEF_PWM_FREQ  20.0e3
#define TWO_PI        6.2832
#define HZ_SCALE      0.1592    // used to convert rad/s to Hz

using namespace MotorTeam;

// constructors
Motor::Motor( PinName pwmPin, PinName tachPin, float tachScale, float pwmScale )
: _tachIn( tachPin ), _pwmOut( pwmPin ), _outStream( USBTX, USBRX )
{
    _outStream.baud( DEF_BAUD_RATE );
    
    _power = 0;
    _speed = 0;
    
    _pwmFreq = DEF_PWM_FREQ;
    _pwmOut.period( 1.0 / DEF_BAUD_RATE );

    _angularFreq = 0;
    _wheelRad = DEF_RADIUS;
    
    _tachScale = tachScale;
    _pwmOut = 0;
}

Motor::Motor( PinName pwmPin, PinName tachPin, float tachScale,
              float pwmScale , float pwmFreq )
: _tachIn( tachPin ), _pwmOut( pwmPin ), _outStream( USBTX, USBRX )
{
    _outStream.baud( DEF_BAUD_RATE );

    _power = 0;
    _speed = 0;
    
    if ( pwmFreq >= DEF_PWM_FREQ ) 
    { 
        _pwmFreq = pwmFreq; 
        _pwmOut.period( 1.0 / _pwmFreq );
    }
    else 
    { 
        _pwmFreq = DEF_PWM_FREQ;
        _pwmOut.period( 1.0 / DEF_PWM_FREQ );
    }

    _angularFreq = 0;
    _wheelRad = DEF_RADIUS;    
    
    _tachScale = tachScale;
    _pwmOut = 0;
}

Motor::Motor( PinName pwmPin, PinName tachPin, float tachScale, float pwmScale, 
              float pwmFreq,  PinName tx, PinName rx, int baud )
: _tachIn( tachPin), _pwmOut( pwmPin ), _outStream( tx, rx )
{
    _outStream.baud( baud );
    
    _power = 0;
    _speed = 0;

    if ( pwmFreq >= DEF_PWM_FREQ ) 
    { 
        _pwmFreq = pwmFreq; 
        _pwmOut.period( 1.0 / _pwmFreq );
    }
    else 
    { 
        _pwmFreq = DEF_PWM_FREQ;
        _pwmOut.period( 1.0 / DEF_PWM_FREQ );   
    }
    
    _angularFreq = 0;
    _wheelRad = DEF_RADIUS;
    
    _tachScale = tachScale;
    _pwmOut = 0;
}



// getters
float Motor::getPower( ) { return _power; }
//float Motor::getAngF( );
//float Motor::getF( );
//float Motor::getPwm( );

// setters
void Motor::setPower  ( float power ) 
{ 
    if ( power != _power )
    {
//        if ( power <= 1.0 && power >= 0.0 ) 
//        {   
            _power = power;
            _pwmOut.write( power );
//        }
//        else
//        {
//            power = 0;
//            _pwmOut.write( 0 );
//            _outStream.printf( "ERROR: %d\rStopping Motors\n\r", PWR_OOB_ERR );
//        }
    }  
}

// class methods
float Motor::calcSpeed( ) 
{ 
    _speed = _wheelRad * this->readTach( );
    
    return _speed;   
}

float Motor::readTach( )
{
    _angularFreq = _tachIn.read( ) * 3.3 * _tachScale * TWO_PI;
    
    return _angularFreq;
}

float Motor::readTachHz( )
{
    float  freqHz = ( this->readTach( ) ) * HZ_SCALE;
    return freqHz;
}

float Motor::readTachPWM( )
{   
    _pwmOut = _tachIn.read( ) * _pwmScale * 3.3;
    return _pwmOut;    
}

// stop the motors
void Motor::stop( )
{
    if ( _power != 0 ) { _pwmOut.write( 0 ); }
}

// send the current dutycycle of the motor output and send it to the
// outputsream. Formatted as a percent
void Motor::printPwm( )
{
    _outStream.printf( "Duty Cycle: %2f%%\n\r", getPower( ) * 100 );
}

PIController::PIController( float proportionalGain, float integralGain, float deltaT )
{
    _proportionalGain = proportionalGain;
    _integralGain     = integralGain;
    _deltaT           = deltaT;
    _errorArea = 0;
    _output = 0;
}

float PIController::calcOutput( float desiredVal, float actualVal )
{
    float error = desiredVal - actualVal;
    _errorArea = _deltaT * error + _errorArea;
    _output = _proportionalGain * error + _integralGain * _errorArea;
    
    return _output;
}
