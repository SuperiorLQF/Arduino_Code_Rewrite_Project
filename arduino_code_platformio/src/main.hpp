#include <Wire.h> // For I2C communication
#include <SPI.h>  // For controlling external chips
#include <Arduino.h>
#include <stateMachine.hpp>

//This is the I2C Address of the MCP4725, by default (A0 pulled to GND).
//Please note that this breakout is for the MCP4725A0.
//Please note that this breakout is for the MCP4725A0.
#define MCP4725_ADDR 0x60   // DAC
#define MAXDACUNIT 4095       // 2^12-1 (ie 12 bits)
#define MINDACUNIT 0


//===================new code===============================
#define SENSOR_BUF_MAXLEN 2000
#define SENSOR_BUF_SENDLEN 1000
typedef void (*Pin_Chan_Operation)(int PinorChan,bool on);//Pin_Chan_Operation pointer that stands for  channel_driver or pin_driver

void channel_driver(int channel_number,bool on);//to make every channel with unified interface

void pin_driver(int pin_number,bool on);//to make every pin with unified interface

bool digital_auto_action(int delay_ms,int duration_ms,Pin_Chan_Operation IOdriver,int PinorChan_number,int current_ms);

bool trial_output_monit(int execute_ms);

void trial_input_read(int execute_ms);

void enc_buf_reset(void);

void RAISE_ERROR(int);

void RAISE_WARNING(int);

void RAISE_HINT(int);

void writeByte(uint8_t val);

void writeTwoByte(uint16_t val);



void DACWrite( int );

int powerToDACUnits( int );

void setDiPoValue( int );

void checkVars( void );

void configureTrial( void );

void startTrial( void );

void endOfTrial( void );
void initTimer0();

void digitalOn(int);

void digitalOff(int);

void toneOn(int);

void toneOff(int);

void laserOn(int);

void laserOff(int);

void elecOn(int);

void elecOff(int);


void odor1On(int, int);

void odor1Off(int);

void odor2On(int, int);

void odor2Off(int);

void odor3On(int, int);

void odor3Off(int);

void odor4On(int, int);

void odor4Off(int);

void evoutput();

void executeCS(int channel, int duration);

void start_training();

void laserrampoff(int);

void rampoffdur(int);

void takeEncoderReading(timems_t &, int32_t &);

void sendEncoderData( void );

void writeLong(uint32_t);

void writeLong(int32_t);

void flushReceiveBuffer();

void executeUS(int channel, int duration);

void solenoidOn(int channel, int duration);

void solenoidOff(int channel);

void puffereyeOn(int channel, int duration);

void puffereyeOff(int channel);

void pufferotherOn(int channel, int duration);

void pufferotherOff(int channel);
