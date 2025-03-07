#ifndef MAIN_H_CODE
#define MAIN_H_CODE

#include <Arduino.h>
#define SENSOR_BUF_MAXLEN 2000
#define SENSOR_BUF_SENDLEN 1000
typedef void (*Pin_Chan_Operation)(int PinorChan,bool on);//Pin_Chan_Operation pointer that stands for  channel_driver or pin_driver

typedef enum {
  DELAY,
  ON,
  FINISH
} Pin_status;

typedef struct{
  Pin_status pin_status;//pin_status:DELAY->ON->FINISH
  bool pin_value;//pin_value 1 or 0 (ON is 1,other is 0)
}Pin_information;

void setup_serial(void);
void setup_pins(void);
void enc_reset(void);

void RAISE_ERROR(int);
void RAISE_WARNING(int);
void RAISE_HINT(int);

void writeByte(uint8_t val);
void writeTwoByte(uint16_t val);

void pin_driver(int pin_number,bool on);//to make every pin with unified interface
Pin_information update_output_info(int delay_ms,int duration_ms,int current_ms);

bool output_controller(int execute_ms);
void sensor_input_reader(int execute_ms);
void startConditioning(void);
void sendEncoderData(void);

void check_matlab_message(void);
void command_exe(int);


#endif
