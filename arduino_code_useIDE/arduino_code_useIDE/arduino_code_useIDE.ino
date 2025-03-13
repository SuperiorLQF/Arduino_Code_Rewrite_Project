#include "init.h"
/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//NOTE-----------------Question Board-------------------------------------
1.[x] the pump dur has deprecated, we use reward dur now //NOTE
2.[ ] what is punish_dur for?
3.[ ] Can we change the data format?we can save pin info in bit
4.[ ] We need to check params value and header 
5.[ ] We need to check the pin number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
#define VIRTUAL_PINOUTPUT_ENABLE 1 //output virtual pin to real pins for debugging

//PARAMS receive From MATLAB
int param_event_lever[MAX_EVENT_NUMBER] = {1,1,1};   //event[i]_lever_number:which lever to choose 1,2,3;0 is unused
int param_eventInterval[MAX_EVENT_NUMBER] ={1000,1000,1000};//500ms
int param_eventnums      = 3;
int param_led_brightness = 255;//0-255
int param_pretimewindow  = 500; //NOTE:old code use 0.2 here, but i am think 16bit cannot transmit float number
int param_posttimewindow = 500;
int param_led_dur        = 500;
int param_reward_dur     = 200; //interval must > reward_dur
int param_ITI            = 1000;
int param_press_Ptrigger = 0; //0:use LOW for press; 1:use HIGH for press;
int param_lick_Ptrigger  = 0; //0:use LOW for lick; 1:use HIGH for lick;

// -- pins in arduino ---
//pin[0]  And pin[1]  are reserved for Serial0

const int pin_LED1            = 5;//2~13 suppoert anlogwrite (PWM)
const int pin_LED2            = 6;//2~13 suppoert anlogwrite (PWM)
const int pin_LED3            = 7;//2~13 suppoert anlogwrite (PWM)

//pin[18] And pin[19] are reserved for Serial1

const int pin_pump            = 22;

// --- sensor pins----
const int pin_lick            = 25;
const int pin_lever1          = 26;
const int pin_lever2          = 27;
const int pin_lever3          = 28;

// --- pin reserved for virtual pin to output for debug ---
const int pin_virtual_dbg0    = 30;
const int pin_virtual_dbg1    = 31;
const int pin_virtual_dbg2    = 32;
const int pin_virtual_dbg3    = 33;
const int pin_virtual_dbg4    = 34;
const int pin_virtual_dbg5    = 35;

int   pin_value_memory[100]; //-1:initial  0:off 1:on   <used by function pin driver>

int   event_pin[MAX_EVENT_NUMBER]; //event_pin[i] stores ith(from 0) event lever pin number
int   event_led[MAX_EVENT_NUMBER]; //event_pin[i] stores ith(from 0) event led   pin number

bool  press_detect_enable = false; //enable signal of press detect
bool  lick_detect_enable  = false; //enable signal of lick  detect
bool  reward_start_flag   = false;
unsigned long  reward_start_ms;
// --- memory for sensors ------
uint8_t   lever_readings[MAX_EVENT_NUMBER][BYTES_NEEDED(SENSOR_BUF_MAXLEN)];//every bits record every ms input, total bits are SENSOR_BUF_MAXLEN
uint8_t   lick_readings[BYTES_NEEDED(SENSOR_BUF_MAXLEN)];//every bits record every ms input, total bits are SENSOR_BUF_MAXLEN

int       lever_data_index  = 0;
int       lick_data_index   = 0;

int       lever_buf_sendlen = 1000;
int       lick_buf_sendlen  = 1000;

int       task_fail_flag    = 1; //1 means have not detect  press yet

// ========== arduino main functions ===============
void setup() {  // put your initialization code here, to run once:
  setup_serial();
  setup_pins();
  //add some test here
  delay(10);
}

void loop() {  // put your main code here, to run repeatedly:

  check_matlab_message();

}

//================ setup functions [begin] ========================//
void setup_serial(void){
  Serial.begin(115200);
  Serial1.begin(115200);
  while(!Serial && !Serial1){ ;}//wait for Serial ready
}


void setup_pins(void){
  //setup pin value memory
  for(int i=0;i<70;i++){
    pin_value_memory[i] = -1;
  }
  //setup all used pins
  pin_driver(pin_LED1,0);
  pin_driver(pin_LED2,0);
  pin_driver(pin_LED3,0);

  pin_driver(pin_pump,0);

  pin_driver(pin_virtual_dbg0,0);  
  pin_driver(pin_virtual_dbg1,0); 
  pin_driver(pin_virtual_dbg2,0); 
  pin_driver(pin_virtual_dbg3,0); 
  pin_driver(pin_virtual_dbg4,0); 
  pin_driver(pin_virtual_dbg5,0); 


  Serial1.println("device set up OK");
}

void sensor_reset(void){//enc only setup in conditioning lab
  pinMode(pin_lick,     INPUT_PULLUP);
  pinMode(pin_lever1,   INPUT_PULLUP);
  pinMode(pin_lever2,   INPUT_PULLUP);
  pinMode(pin_lever3,   INPUT_PULLUP);

  lever_data_index     = 0;
  lick_data_index      = 0;
  
  lever_buf_sendlen    = 1000;//FIXME not very sure
  lick_buf_sendlen     = 1000;//FIXME not very sure

  for(int i=0;i<MAX_EVENT_NUMBER;i++){ //initialize sensor memory data
    for(int j=0;j<BYTES_NEEDED(SENSOR_BUF_MAXLEN);j++){
      if(param_press_Ptrigger == false){
        lever_readings[i][j] = 0xFF;
      }
      else{
        lever_readings[i][j] = 0x00;
      }
    }
  }
  for(int i=0;i<BYTES_NEEDED(SENSOR_BUF_MAXLEN);i++){
      if(param_lick_Ptrigger == false){
        lick_readings[i] = 0xFF;
      }
      else{
        lick_readings[i] = 0x00;
      }
  }
}
//================ setup function [end] ========================//


//================ pin driver function [begin] ==================//
void pin_driver(int pin_number,int value){ // pin 0,1 is used for serial communication. DO NOT drive it (dedicated for Serial)
  // *This func is triggered every 1 ms in conditioning
  // **This func is active only in 2 cases:
  // ***pin_value_memory store the pin value(t-1) with reset value -1
  // [1.first init]  when value(t-1) = -1 (i.e. setup_flag = 1) (i.e. at the setup_pins func)
  // [2.pin value change]  value(t-1) != value(t),  (i.e. value_change_flag = 1)
  bool  setup_flag;
  bool  value_change_flag;
  int   random_value;

  setup_flag        = (pin_value_memory[pin_number] == -1);//if 1 ,we should set pinMode before drive
  value_change_flag = (pin_value_memory[pin_number] != value);//if 1,the value change,we should drive it
  if(pin_number == 0 || pin_number == 1){ // pin 0,1 is used for serial communication. DO NOT drive it 
    return;
  }

  if(setup_flag || value_change_flag){
    pin_value_memory[pin_number] = value; //update value memory
    //++++ pwm led pin++++++++
    if(pin_number == pin_LED1 || pin_number == pin_LED2 || pin_number == pin_LED3){
      if(setup_flag){
        pinMode(pin_number,OUTPUT);
      }
      analogWrite(pin_number,param_led_brightness*value);
    }

    //++++ digital pin++++++++
    else if (pin_number <54){
      if(setup_flag){ 
        pinMode(pin_number,OUTPUT);
      } /*setup*/
        digitalWrite(pin_number,value);
    } 

     
  }
}

Pin_information update_output_info(int delay_ms,int duration_ms,int current_ms){
  Pin_information pin_information_tmp;
  if(duration_ms>0){
    if(current_ms<delay_ms){
      pin_information_tmp.pin_status = DELAY;
      pin_information_tmp.pin_value  = 0;
    }
    else if(current_ms<delay_ms+duration_ms){
      pin_information_tmp.pin_status = ON;
      pin_information_tmp.pin_value  = 1;
    }
    else{
      pin_information_tmp.pin_status = FINISH;
      pin_information_tmp.pin_value  = 0;
    }
  }
  else{//which means this pin or channel is unused
    pin_information_tmp.pin_status = FINISH;
    pin_information_tmp.pin_value  = 0;
  }
  return pin_information_tmp;
}
//================pin driver function [end]======================//


//=============start one trial of training  [begin]=========//
void startLeverTask(void){
  unsigned long slot_start_us,slot_intermediate_us;
  int execute_us;
  unsigned long trial_start_ms,trial_current_ms;
  int execute_ms;
  bool end_flag;
  
  // --- Task initialization ----
  int  current_event_num = 0;
  reward_start_flag  = false;
  sensor_reset(); 
  for(int i=0;i<param_eventnums;i++){//[0] is event1,[1] is event2
    event_pin[i] = (param_event_lever[i] == 1)?pin_lever1:
                   (param_event_lever[i] == 2)?pin_lever2:
                   (param_event_lever[i] == 3)?pin_lever3:0;//pin=0 means closed
    event_led[i] = (param_event_lever[i] == 1)?pin_LED1:
                   (param_event_lever[i] == 2)?pin_LED2:
                   (param_event_lever[i] == 3)?pin_LED3:0;//pin=0 means closed
  }
  while(current_event_num < param_eventnums){
    // --- Event initialization ----
    Serial1.print("Start event: ");
    Serial1.println(current_event_num);
    event_init();
    end_flag = false;
    trial_start_ms = millis();

    //---------------------event LOOP [begin]----------------------//
    while(!end_flag){//a trial is divided into 1ms slots
      slot_start_us = micros();
      trial_current_ms = millis();
      execute_ms = trial_current_ms - trial_start_ms;

      //#### The main process begin ####
      end_flag=output_controller(execute_ms, current_event_num);
      sensor_input_reader(current_event_num);
      //#### The main process end ####

      // LOOP timer begin (1 ms per loop)
      slot_intermediate_us = micros();    
      execute_us =  slot_intermediate_us - slot_start_us;
      if(execute_us<1000){
        delayMicroseconds(1000-execute_us);
      }
      else{
        RAISE_ERROR(__LINE__);//BIG_PROBLEM:time to ececute functions longer than 1ms
      }
    }
    //-----------------------event LOOP [end]-------------------------//

    //check if event fail: no detect of pressing
    if(task_fail_flag == 1){//FAIL to press
      Serial1.print("[");
      Serial1.print("event ");
      Serial1.print(current_event_num+1);    
      Serial1.print("] FAILED ");
      Serial1.println(execute_ms);  
      break;
    }else{
      Serial1.print("[");
      Serial1.print("event ");
      Serial1.print(current_event_num+1);      
      Serial1.println("] SUCCESS");
    }
    current_event_num ++;
  }

  sendSensorData();
  event_init();

  RAISE_HINT(__LINE__);
}
//================ start one trial of training  [end]=======//

//================ training functions [begin]===============//
bool output_controller(int execute_ms,int current_event_num){
  bool exit_flag;
  bool last_event_flag   = (current_event_num == param_eventnums-1);
  if(reward_start_flag == false && last_event_flag && task_fail_flag == 0){
    reward_start_flag = true;
    reward_start_ms   = millis();
  }

  Pin_information led_info;      //Pin_information is a used-defined variable type (it has pin_status and pin_value)
  Pin_information pre_led_info;  //virtual pin //NOTE:we can drive the virtual signal to the pin unused for debug to see the real waveform
  Pin_information post_led_info; //virtual pin //NOTE:we can drive the virtual signal to the pin unused for debug to see the real waveform
  Pin_information interval_info; //virtual pin //NOTE:we can drive the virtual signal to the pin unused for debug to see the real waveform
  Pin_information ITI_info;      //virtual pin //NOTE:we can drive the virtual signal to the pin unused for debug to see the real waveform
  Pin_information reward_info;   //real    pin
  //--------------- 1.get pin output information (status and value) ---------------
  //                                 |delay                                  |duration                                |current_time
  pre_led_info   = update_output_info(0                                      ,param_pretimewindow                     ,execute_ms);
  led_info       = update_output_info(param_pretimewindow                    ,param_led_dur                           ,execute_ms);
  post_led_info  = update_output_info(param_pretimewindow + param_led_dur    ,param_posttimewindow                    ,execute_ms);
  interval_info  = update_output_info(param_pretimewindow + param_led_dur + param_posttimewindow
                                                                             ,param_eventInterval[current_event_num]  
                                                                             ,execute_ms);
  ITI_info       = update_output_info(param_pretimewindow + param_led_dur + param_posttimewindow + param_eventInterval[current_event_num]
                                                                             ,(last_event_flag?param_ITI:0)// only last event has ITI
                                                                             ,execute_ms);

  reward_info    = update_output_info(0                                      ,((last_event_flag && task_fail_flag == 0)?param_reward_dur:0)
                                                                             ,millis() - reward_start_ms);//we add condition expression in duration so it will triggerd suddenly by that condition

  press_detect_enable = pre_led_info.pin_value || led_info.pin_value || post_led_info.pin_value;
  lick_detect_enable  = reward_info.pin_value;

  //--------------  2.drive the pin (on/off control by arduino built-in function, digitalWrite) ---------------
  pin_driver(event_led[current_event_num]         ,led_info        .pin_value);//event led
  pin_driver(pin_pump                             ,reward_info     .pin_value);//event pump
  if(VIRTUAL_PINOUTPUT_ENABLE){  /*you can add virtual pin to drive to debug*/
    pin_driver(pin_virtual_dbg0,pre_led_info .pin_value);
    pin_driver(pin_virtual_dbg1,post_led_info.pin_value);
    pin_driver(pin_virtual_dbg2,interval_info.pin_value);
    pin_driver(pin_virtual_dbg3,ITI_info     .pin_value);
  }


  //--------------  3.caculate exit flag condition ---------------
  bool event_window_finish =  (pre_led_info     .pin_status == FINISH)  &&
                              (led_info         .pin_status == FINISH)  &&
                              (post_led_info    .pin_status == FINISH);
  bool event_interv_finish =  (interval_info    .pin_status == FINISH);
  bool event_ITI_finish    =  (ITI_info         .pin_status == FINISH);
  
  bool event_exit_success  = //exit case 1:press detect so we have interval
                            ( event_window_finish &&
                              event_interv_finish &&
                              event_ITI_finish    
                            );
  bool event_exit_fail     = //exit case 2:press failed so we have don't have interval
                            ( event_window_finish &&
                              (task_fail_flag == 1) //in that case it quit immediately,so no interval or ITI need               
                            );

  exit_flag =   event_exit_success || event_exit_fail;

  if(exit_flag == 1 && VIRTUAL_PINOUTPUT_ENABLE){
    pin_driver(pin_virtual_dbg0,0);
    pin_driver(pin_virtual_dbg1,0);
    pin_driver(pin_virtual_dbg2,0);
    pin_driver(pin_virtual_dbg3,0);
  }

  return exit_flag;
}

void sensor_input_reader(int current_event_num){
  /*if press_detect_enable OR lick_detect_enable*/
  if(press_detect_enable){
    if (lever_data_index < SENSOR_BUF_MAXLEN) {
      bool value = check_press(event_pin[current_event_num]);
      if(value == 1){//press happened
        task_fail_flag = 0;
      }
      setBit(lever_readings[current_event_num], lever_data_index,value);
    }
    else {
      RAISE_ERROR(__LINE__);//SENSOR_BUF_MAXLEN is to short
    }
    lever_data_index ++;
  }
  
  if(lick_detect_enable){
    if (lick_data_index < SENSOR_BUF_MAXLEN) {
      bool value = check_lick(pin_lick);

      setBit(lick_readings, lick_data_index,value);
    }
    else {
      RAISE_ERROR(__LINE__);//SENSOR_BUF_MAXLEN is to short
    }
    lick_data_index ++;
  }

}
//================ training functions [end] =====================//




//================ communicate function (from matlab) [begin] =================//
//All data(params/command) has the same 4 Byte structure to simplify communication with MATLAB
//2Bytes header + 2Bytes body, header=1 implify body is command
void check_matlab_message(void) {
  uint8_t param_block[4];
  int header;
  int value;
  int available_number;
  int command;
  while (Serial.available() > 0 ) {
    //wait for receiving 4Byte data
    delay(5);  //one byte cost 0.1ms at 115200 baudrate
    
    //receive the whole 4Byte data
    if (Serial.available() >= 4) {
      
      Serial.readBytes(param_block,4);
      header = param_block[0] | param_block[1] << 8;
      value = param_block[2] | param_block[3] << 8;
      Serial1.print("header:");
      Serial1.println(header);
      Serial1.print("value:");   
      Serial1.println(value); 
      // If you add a new case don't forget to put a break statement after it; c-style switches run through
      // TODO:And we can check if the header is illegal
      switch (header) {

        case 1: // 1 is a special header for commands from matlab
          command = value;
          command_exe(command);
          break;
  
        case 3:
          break;
        case 4:
          break;
        case 5:
          break;
        case 6:
          break;
        case 7:
          break;
        case 8:
          break;
        case 9:
          break;
        case 10:
          break;
        case 11:
          break;
        case 12:
          break;

        case 24:
          break;
        case 25:
          break;
        case 26:
          break;
        case 27:
        case 28:
          break;
        case 29: 
          break;
        case 30: 
          break; 
        case 31: 
          break; 
        case 32: 
          break;
        case 33: 
          param_press_Ptrigger = value;
          break;
        case 34:
          param_lick_Ptrigger  = value;
          break;
        case 35:
          param_event_lever[0] = value;//event1 lever number
          break;
        case 36:
          param_event_lever[1] = value;//event2 lever number
          break;
        case 37:
          param_event_lever[2] = value;//event3 lever number
          break;
        case 38:
          param_eventnums      = value;
          break;
        case 39:
          param_eventInterval[0] = value;//event1 interval
          break;
        case 40:
          param_eventInterval[1] = value;//event1 interval
          break;          
        case 41:
          param_ITI            = value;//ms
          break;
        case 42:
          param_pretimewindow  = value;//ms
          break;
        case 43:
          param_posttimewindow = value;//ms
          break;
        case 44:
          param_reward_dur     = value;//ms
        case 46:
          param_led_brightness = value;//0-255
          break;
        case 47:
          param_eventInterval[3] = value;//event1 interval
          break;
        case 48:
          break;
        case 49:
          break;
        case 50:
          break;
        case 51:
          break;
        case 52:
          break;

        case 55:
          param_led_dur = value;//ms
          break;
        default:
          break;
      }

    }
    else {
      //illegal data format other than 4 Bytes
      available_number = Serial.available();
      Serial1.println(available_number);
      //for(int j=0;j<available_number;j++){
      //  Serial.read();
      //}
      break;
    } // end of if
  } // end of while
}

void command_exe(int command){
  Serial1.print("get command:");
        Serial1.println(command);

        switch(command) {
      case 1:
          break;
      case 2: 
          break;
      case 3:
          break;
      case 4:
          break;
      case 5: 
          break;
      case 6:
          break;
      case 7:
          break;
      case 8:
          break;
      case 9:
          startLeverTask();
          break;
      default:
          break;
  }
}


void sendSensorData(void) {
    // Serial.write(ENCODER_L);
    // int i;
    // for (i=0; i<sensor_buf_sendlen; i++) {//FIXME:change param_encodernumreadings to SENSOR_BUF_SENDLEN
    //     writeByte(sensor_readings[i]);
    // }
    // Serial1.println(sensor_buf_sendlen);
    // Serial1.println(i);
    // Serial.write(TIME_L);
    // for (i=0; i<4; i++) {//FIXME:change param_encodernumreadings to SENSOR_BUF_SENDLEN
    //     writeByte(sensor_times[i]);
    // }
    RAISE_HINT(__LINE__);
}
//================ communicate function [end] =================//

//================ send data function (to matlab) [begin]===================//
void writeByte(uint8_t val){
  Serial.write(val);
}
void writeTwoByte(uint16_t val){
  uint8_t low_Byte   = val & 0xFF;
  uint8_t high_Byte  = val >> 8; 

  Serial.write(low_Byte);//send low_byte first
  Serial.write(high_Byte);
}
//================ send data function [end]=====================//

//================ print error info function (via Serial1 of arduino) [begin]==================//
void RAISE_ERROR(int line_number){
  Serial1.print("ERROR[");
  Serial1.print(line_number);
  Serial1.println("]");
}
void RAISE_WARNING(int line_number){
  Serial1.print("WARN[");
  Serial1.print(line_number);
  Serial1.println("]");
}
void RAISE_HINT(int line_number){
  Serial1.print("INFO[");
  Serial1.print(line_number);
  Serial1.println("]");
}
//================ print info function  [end]====================//
//check_press:1 for press and 0 for not press
bool check_press(int pin){//FIXME:we can count edge numbers depend on this function
  if (param_press_Ptrigger){//false:use negitive for tirgger; true:use positive for trigger;
    return digitalRead(pin) == HIGH;
  }else {
    return digitalRead(pin) == LOW;
  }
}

//very similar to check_press
bool check_lick(int pin){//FIXME:we can count edge numbers depend on this function
  if (param_lick_Ptrigger){//false:use negitive for tirgger; true:use positive for trigger;
    return digitalRead(pin) == HIGH;
  }else {
    return digitalRead(pin) == LOW;
  }
}

void setBit(uint8_t *array, uint16_t bitIndex, bool value){//set the bit in uint8
    uint16_t byteIndex = bitIndex / BITS_PER_BYTE;
    uint8_t bitOffset = bitIndex % BITS_PER_BYTE;
    if(value == 1){
      array[byteIndex] |= (1 << bitOffset);
    }
    else{
      array[byteIndex] &= ~(1 << bitOffset);
    }
}

void event_init(void){
    press_detect_enable  = false;
    lick_detect_enable   = false;
    lever_data_index     = 0;
    lick_data_index      = 0;
    task_fail_flag       = 1;//if detect press, then we turn task_fail_flag to 0
}
