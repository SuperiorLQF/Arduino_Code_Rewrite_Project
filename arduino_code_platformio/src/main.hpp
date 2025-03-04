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


void RAISE_ERROR(int);

void RAISE_WARNING(int);

void RAISE_HINT(int);

bool trial_output_monit(int);

void trial_input_read(void);

bool digital_auto_action(int delay_ms,int duration_ms,int pin_number,int current_ms);

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
