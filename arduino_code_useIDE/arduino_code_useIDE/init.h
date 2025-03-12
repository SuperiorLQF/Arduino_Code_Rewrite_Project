#ifndef MAIN_H_CODE
#define MAIN_H_CODE

#include <Arduino.h>
#include <sam.h>

#define SENSOR_BUF_MAXLEN 5000



typedef void (*Pin_Chan_Operation)(int PinorChan,bool on);//Pin_Chan_Operation pointer that stands for  channel_driver or pin_driver

typedef enum {
  FRESH,
  DELAY,
  ON,
  FINISH
} Pin_status;

typedef enum {
  NP_N,
  NP_P,
  NP_LOW,
  PN_P,
  PN_N,
  PN_LOW,
} Elec_status;

typedef struct{
  Pin_status pin_laststatus;//stored last 1ms status
  Pin_status pin_status;//pin_status:DELAY->ON->FINISH
  bool pin_value;//pin_value 1 or 0 (ON is 1,other is 0)
}Pin_information;

void setup_serial(void);
void setup_SPI(void);
void setup_pins(void);
void sensor_reset(void);

void RAISE_ERROR(int);
void RAISE_WARNING(int);
void RAISE_HINT(int);

void writeByte(uint8_t val);
void writeTwoByte(uint16_t val);

void pin_driver(int pin_number,int value);//to make every pin with unified interface
Pin_information update_output_info(int delay_ms,int duration_ms,int current_ms);

bool output_controller(int execute_ms);
void sensor_input_reader(int execute_ms);
void startConditioning(void);
void sendSensorData(void);

void check_matlab_message(void);
void command_exe(int);

void tone_TIMER_HANDLER(void);
//void laser_TIMER_HANDLER(void);

void dac8563_init(void);
void dac8563_output(int chan_num,uint16_t data);//chan_num 0:A  1:B
void writeDAC(uint8_t cmd,uint16_t data);

void timer_start(int timer_number);
void timer_reset(int timer_number,int counter_number_set);
void timer_stop(int timer_number);
#endif
