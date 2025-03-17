#ifndef MAIN_H_CODE
#define MAIN_H_CODE

#include <Arduino.h>
#define  MAX_EVENT_NUMBER  10  //NODE:must >= the real value of param_eventnums
#define  SENSOR_BUF_MAXLEN 80000 //in bit
#define  BITS_PER_BYTE     8
#define  BYTES_NEEDED(x) ((x + BITS_PER_BYTE - 1) / BITS_PER_BYTE)


#define LEVER_DATA_HEADER        0x70
#define LICK_DATA_HEADER         0x80


typedef enum {
  FRESH,
  DELAY,
  ON,
  FINISH
} Pin_status;

typedef struct{
  Pin_status pin_laststatus;//stored last 1ms status
  Pin_status pin_status;//pin_status:DELAY->ON->FINISH
  bool pin_value;//pin_value 1 or 0 (ON is 1,other is 0)
}Pin_information;

void setup_serial(void);
void setup_pins(void);
void sensor_reset(void);

void RAISE_ERROR(int);
void RAISE_WARNING(int);
void RAISE_HINT(int);

void writeByte(uint8_t val);
void writeTwoByte(uint16_t val);

void pin_driver(int pin_number,int value);//to make every pin with unified interface
Pin_information update_output_info(int delay_ms,int duration_ms,int current_ms);

bool output_controller(int execute_ms,int current_event_num);
void sensor_input_reader(int current_event_num);
void startLeverTask(void);
void sendSensorData(void);

void check_matlab_message(void);
void command_exe(int);


void setBit(uint8_t *array, uint16_t bitIndex, bool value);
bool check_press(int pin);
bool check_lick(int pin);

void send_empty_data_to_MATLAB(void);

bool verify_params(void);

bool update_event_state(int event_execute_time,int press_result);

void trial_init(void);

bool training_init(void);

#endif
