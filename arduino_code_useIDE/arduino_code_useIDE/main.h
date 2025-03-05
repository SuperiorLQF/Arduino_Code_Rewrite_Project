#ifndef MAIN_H_CODE
#define MAIN_H_CODE

#include <Arduino.h>
#define SENSOR_BUF_MAXLEN 2000
#define SENSOR_BUF_SENDLEN 1000
typedef void (*Pin_Chan_Operation)(int PinorChan,bool on);//Pin_Chan_Operation pointer that stands for  channel_driver or pin_driver

void setup_serial(void);
void setup_pins(void);
void enc_reset(void);

void RAISE_ERROR(int);
void RAISE_WARNING(int);
void RAISE_HINT(int);

void writeByte(uint8_t val);
void writeTwoByte(uint16_t val);

void channel_driver(int channel_number,bool on);//to make every channel with unified interface
void pin_driver(int pin_number,bool on);//to make every pin with unified interface
bool digital_auto_action(int delay_ms,int duration_ms,Pin_Chan_Operation IOdriver,int PinorChan_number,int current_ms);

bool output_controller(int execute_ms);
void sensor_input_reader(int execute_ms);
void startConditioning(void);
void sendEncoderData(void);

void checkVars(void);
int  checkCMD(void);




#endif
