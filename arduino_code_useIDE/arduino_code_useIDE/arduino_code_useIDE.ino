#include "init.h"
/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//NOTE-----------------Question Board-------------------------------------
1.[x] the pump dur has deprecated, we use reward dur now
2.[ ] what is punish_dur for in old code?
3.[ ] Can we change the data format?we can save pin info in bit.now we use send_empty_data_to_MATLAB to avoid communication fail with MATLAB
4.[ ] We need to check params value and header and in Matlab.to change dataToArduino function in reward.m to use 2B header and 2B data, and also change the value of params(L104,L386)
5.[ ] We need to check the pin number,if the pin number is accepted
6.[x] -> Add param_trial_num
7.[x] ->We can connect UART RX1 from computer to the trigger pins so we can use mouse click to send a lower trigeer(send 150*0 in 115200 baudrate) on Serial RX1(pin 19) line to simulate lever pressing 
8.[ ] Do we still need pause /stop
9.[ ] Confirm the led won't turn off even if a lever press happened 
10.[^] if you dont use interval 3, then you can set param_eventInterval[2] equal to param_reward_dur
11.[ ] interval param configure problem
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
hex data for serial
01000900 : *start lever task*
0C000100 : set lever triggerd by low (default is high)
0E000100 : select lever number 1,2,3 for event1
0F000200 : select lever number 1,2,3 for event2
1B00C409 : set event1 post winodow = 2500ms(09C4)
1C00C409 : set event2 post winodow = 2500ms(09C4)
1D00C409 : set event3 post winodow = 2500ms(09C4)
1800D007 : set event1 post winodow = 2000ms(07D0)
1900D007 : set event2 post winodow = 2000ms(07D0)
1A00D007 : set event3 post winodow = 2000ms(07D0)
1500D007 : set led   duration time = 2000ms(07D0)
1300C800 : set interval3 time      = 200ms (00C8) 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
#define VIRTUAL_PINOUTPUT_ENABLE 1 //output virtual pin to real pins(pin_virtual_dbg0/1/2/3/4/5...) for debugging

//===PARAMS receive From MATLAB===
int param_event_lever   [MAX_EVENT_NUMBER]    = {1,1,1};          //event[i]:which lever to choose 1,2,3;   0 is unused
int param_eventInterval [MAX_EVENT_NUMBER]    = {1000,1000,1000}; //ms
int param_pretimewindow [MAX_EVENT_NUMBER]    = {500,500,500};    //ms
int param_posttimewindow[MAX_EVENT_NUMBER]    = {1000,1000,1000}; //ms

int param_eventnums      = 3;  
int param_press_Ptrigger = 0; //0:use LOW for press; 1:use HIGH for press;
int param_lick_Ptrigger  = 0; //0:use LOW for lick; 1:use HIGH for lick;
int param_led_brightness = 255;//0-255
int param_led_dur        = 500;
int param_reward_dur     = 200; //interval must > reward_dur
int param_ITI            = 1000;
int param_release_dur    = 500;//ms


//=======pins in arduino=========
//pin[0]  And pin[1]  are reserved for Serial0
const int pin_LED[3]          = {5,6,7};//pin 2~13 suppoert anlogwrite (PWM)

//pin[18] And pin[19] are reserved for Serial1 TX RX
const int pin_pump            = 22;

// --- sensor pins----
const int pin_lick            = 25;
const int pin_lever[3]        = {26,27,28};

// --- virtual pins---
const int dbg_window_pin      = 32;

//========memory for sensors=====
uint8_t   lever_readings[3][BYTES_NEEDED(SENSOR_BUF_MAXLEN)];//every bit record every ms every input, total bits are SENSOR_BUF_MAXLEN
uint8_t   lick_readings[BYTES_NEEDED(SENSOR_BUF_MAXLEN)];    //every bit record every ms every input, total bits are SENSOR_BUF_MAXLEN

//=========golbal variables======
int   pin_value_memory[100]; //-1:initial  0:off 1:on   <used by function pin driver>

int   event_pin[MAX_EVENT_NUMBER]; //event_pin[i] stores ith(from 0) event lever pin number
int   event_led[MAX_EVENT_NUMBER]; //event_pin[i] stores ith(from 0) event led   pin number

int   memory_data_index  = 0;

int   memory_buf_sendlen = 10000; //FIXME

int       trial_fail_flag   = 0;
float     current_event_num = 1;//NOTE:we use float because x.5 means interval

unsigned long event_start_ms = 0; //ms
unsigned long trial_start_ms = 0; //ms

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
    pin_value_memory[i] = -1;//-1:haven't inital yet
  }
  //setup all used pins
  pin_driver(pin_LED[0],0);
  pin_driver(pin_LED[1],0);
  pin_driver(pin_LED[2],0);

  pin_driver(pin_pump,0);

  Serial1.println("pins set up OK");
}

void sensor_reset(void){//enc only setup in conditioning lab
  pinMode(pin_lick,       INPUT_PULLUP);
  pinMode(pin_lever[0],   INPUT_PULLUP);
  pinMode(pin_lever[1],   INPUT_PULLUP);
  pinMode(pin_lever[2],   INPUT_PULLUP);

  memory_data_index     = 0;
  
  for(int i=0;i<3;i++){ //initialize sensor memory data
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
    if(pin_number == pin_LED[0] || pin_number == pin_LED[1] || pin_number == pin_LED[2]){
      if(setup_flag){
        pinMode(pin_number,OUTPUT);
        analogWrite(pin_number,255*value);
      }
      else{
        analogWrite(pin_number,param_led_brightness*value);
      }
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


/*============================================================================================
//===================================start training  [begin]==================================
==============================================================================================
*/
void startLeverTask(void){
  unsigned long slot_start_us,slot_intermediate_us;
  int execute_us;
  int trial_execute_ms,event_execute_ms;
  bool exit_flag;

  // --- trial initialization ----
  Serial1.println("Trial start");

  trial_init();
  exit_flag = false;

  //---------------------trial LOOP [begin]----------------------//
  while(!exit_flag){//a trial is divided into 1ms slots
    slot_start_us    = micros();
    trial_execute_ms = millis() - trial_start_ms;
    event_execute_ms = millis() - event_start_ms;
    //################# The main process begin ###################
    bool press_result = sensor_input_reader(current_event_num);
    exit_flag = update_event_state(event_execute_ms,press_result);
    output_controller(event_execute_ms, current_event_num,exit_flag);// trial_fail_flag in this function may become 0 if lever happens
    //################# The main process end #####################

    //delay remaining time(1ms per loop)
    slot_intermediate_us = micros();    
    execute_us =  slot_intermediate_us - slot_start_us;
    if(execute_us<1000){
      delayMicroseconds(1000-execute_us);
    }
    else{
      RAISE_ERROR(__LINE__);//BIG_PROBLEM:time to ececute functions longer than 1ms
    }
  }
  //-----------------------trial LOOP [end]-------------------------//

  //check if event fail: if fail go to next trial
  unsigned long ITI_start_ms = millis();
  int ITI_real_delay_ms; 
  if(trial_fail_flag == 1){//FAIL to press
    Serial1.println("FAILED");
  }else{  
    Serial1.println("SUCCESS");
    sendSensorData();
  }
  ITI_real_delay_ms = param_ITI-(millis() - ITI_start_ms);
  if(ITI_real_delay_ms>0){
    delay(ITI_real_delay_ms);
  }
  else{
    RAISE_ERROR(__LINE__);//ERROR:send data cost too much time
  }

}
/*============================================================================================
//===================================start training  [end]====================================
==============================================================================================
*/

//================ training functions [begin]===============//
void output_controller(int event_execute_ms,float current_event_num,bool exit_flag){
  bool flag_normal_event = (current_event_num==1 || 
                            current_event_num==2 ||
                            current_event_num==3);

  Pin_information led_info;      //Pin_information is a used-defined variable type (it has pin_status and pin_value)
  Pin_information reward_info;  
  //--------------- 1.get pin output information (status and value) ---------------
  //                                 |delay                                               |duration                                |current_time
  led_info       = update_output_info(param_pretimewindow[(int)(current_event_num-1)]     ,param_led_dur                           ,event_execute_ms);
  reward_info    = update_output_info(0/*0 delay from event3.5 start */                   ,param_reward_dur                        ,event_execute_ms);
  //--------------  2.drive the pin (on/off control by arduino built-in function, digitalWrite) ---------------
  if(exit_flag){//close all output before exit
    pin_driver(pin_LED[0]        ,0);//event led
    pin_driver(pin_LED[1]        ,0);//event led
    pin_driver(pin_LED[2]        ,0);//event led
    pin_driver(pin_pump          ,0);//event pump
  }
  else if(flag_normal_event){//1/2/3
    pin_driver(event_led[(int)(current_event_num-1)]         ,led_info        .pin_value);//event led
  }
  else if(current_event_num == 3.5){ //3.5 for reward
    pin_driver(pin_LED[0]        ,0);//event led
    pin_driver(pin_LED[1]        ,0);//event led
    pin_driver(pin_LED[2]        ,0);//event led    
    pin_driver(pin_pump                                      ,reward_info     .pin_value);//event pump
  }
  else{//interval:1.5/2.5
    pin_driver(pin_LED[0]         ,0);//event led
    pin_driver(pin_LED[1]         ,0);//event led
    pin_driver(pin_LED[2]         ,0);//event led    
  }
  //------------------3. drive dbg pin---------------------------
  if(exit_flag){
    pin_driver(dbg_window_pin,0);
  }
  else if(flag_normal_event){
    pin_driver(dbg_window_pin,1);
  }
  else{
    pin_driver(dbg_window_pin,0);
  }
}


bool sensor_input_reader(float current_event_num){
  /*sensor collect input during the whole trial*/
  bool press_value =  false;
  bool press_lever[3];
  bool flag_normal_event = (current_event_num==1 || 
                            current_event_num==2 ||
                            current_event_num==3);
  

  press_lever[0] = check_press(pin_lever[0]);
  press_lever[1] = check_press(pin_lever[1]);
  press_lever[2] = check_press(pin_lever[2]);
  bool lick_value     = check_lick(pin_lick);
  
  if (memory_data_index < SENSOR_BUF_MAXLEN) {
    setBit(lever_readings[0], memory_data_index,press_lever[0]);
    setBit(lever_readings[1], memory_data_index,press_lever[1]);
    setBit(lever_readings[2], memory_data_index,press_lever[2]);
    setBit(lick_readings    , memory_data_index,lick_value    );
  } 
  else{
    RAISE_ERROR(__LINE__);//SENSOR_BUF_MAXLEN is to short
  } 

  press_value = press_lever[param_event_lever[(int)(current_event_num-1)]-1];//get current event's corresponding lever press value

  memory_data_index ++;

  return press_value;
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
      switch (header) {//NODE:this part are corresponding to MATLAB code dataToArduino(reward.m), we use 2B header and 2B  payload

        case 1: // 1 is a special header for commands from matlab
          command = value;
          command_exe(command);
          break;

        //case 3-10 reserverd for special which may needed in future

        case 11://interval between event
          param_eventnums  = value;
          break;
        case 12:
          param_press_Ptrigger = value;//0:use LOW for pressing    |   1:use HIGH for pressing;
          break;
        case 13:
          param_lick_Ptrigger  = value;//0:use LOW for licking     |   1:use HIGH for licking;
          break;
        case 14:
          param_event_lever[0] = value;//which lever number event1 is using, and LED is the same
          break;
        case 15:
          param_event_lever[1] = value;//which lever number event2 is using, and LED is the same
          break;
        case 16:
          param_event_lever[2] = value;//which lever number event3 is using, and LED is the same
          break;
        case 17:
          param_eventInterval[0] = value;//event1 interval  ms, Max 65.535s
          break;
        case 18:
          param_eventInterval[1] = value;//event1 interval  ms, Max 65.535s
          break;   
        case 19:
          param_eventInterval[2] = value;//event1 interval  ms, Max 65.535s
          break;
        case 20:
          param_ITI            = value;//inter-trial interval ms, Max 65.535s
          break;
        case 21:
          param_led_dur        = value;//ms, Max 65.535s
          break;
        case 22:
          param_led_brightness = value;//1-255
          break;
        case 23:
          param_reward_dur     = value;//ms, must smaller than every interval
          break;
        case 24:
          param_pretimewindow[0]  = value;//ms, Max 65.535s
          break;
        case 25:
          param_pretimewindow[1]  = value;//ms, Max 65.535s
          break;
        case 26:
          param_pretimewindow[2]  = value;//ms, Max 65.535s
          break;
        case 27:
          param_posttimewindow[0] = value;//ms, Max 65.535s
          break;
        case 28:
          param_posttimewindow[1] = value;//ms, Max 65.535s
          break;
        case 29:
          param_posttimewindow[2] = value;//ms, Max 65.535s
          break;
        case 30:
          param_release_dur       = value;
          break;

        default:
          if(value != 0){
            RAISE_WARNING(__LINE__);  //the param  of this header is undefined 
          }
          break;
      }

    }
    else {
      //illegal data format other than 4 Bytes
      available_number = Serial.available();
      Serial1.println(available_number);
      break;
    }
  } // end of while
}

void command_exe(int command){
  Serial1.print("get command:");
  Serial1.println(command);

  switch(command) {
    case 9:
        if(training_init() == true){ //before start lever task, we check for the params if they are all legal
          startLeverTask();
        }
        else {
          Serial1.println("training_init failed!");
        }
        break;
    default:
        break;
  }
}


void sendSensorData(void) {
    for(int i=0;i<3;i++){//3 lever
      for(int j=0;j<BYTES_NEEDED(memory_buf_sendlen);j++){
        Serial.write(LEVER_DATA_HEADER+i);//70,71,72
        writeByte(lever_readings[i][j]);
      }
    }
    for(int j=0;j<BYTES_NEEDED(memory_buf_sendlen);j++){
      Serial.write(LICK_DATA_HEADER);//80
      writeByte(lick_readings[j]);
    }    
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


void send_empty_data_to_MATLAB(void){//just to keep communicate works well with current verison of MATLAB
    Serial.print(0); //currenttrialnum
    Serial.print(",");
    Serial.print(0); //correcttrials
    Serial.print(",");
    Serial.print(0); //FailTrialNum
    Serial.print(",");
    Serial.print(0); //lickcounts
    Serial.print(",");
    Serial.print(0); //misslick
    Serial.print(",");
    Serial.print(0.0); //event1_time
    Serial.print(",");
    Serial.print(0.0); //event2_time
    Serial.print(","); 
    Serial.print(0.0);  //event3_time
    Serial.print(",");
    Serial.println(0.0);  //lick_time
}


bool verify_params(void){ //verify params for some wrong value
  Serial1.println(param_eventInterval[0]);
  Serial1.println(param_eventInterval[1]);
  Serial1.println(param_eventInterval[2]);
  bool params_legal_flag = true;

  for(int i=0;i<param_eventnums;i++){
    if(param_event_lever[i] < 1 || param_event_lever[i] >3){
      params_legal_flag = false;
      RAISE_ERROR(__LINE__);// PARAMERR:param_event_lever[i] cannot be number other than # 1,2,3
    }
    if(param_posttimewindow[i]<= param_led_dur){
      params_legal_flag = false;
      RAISE_ERROR(__LINE__);//PARAMERR:param_posttimewindow[i] cannot be smaller than param_led_dur
    }
    if(param_eventInterval[i]<=param_release_dur){
      params_legal_flag = false;
      RAISE_ERROR(__LINE__);//PARAMERR:interval[i] cannot be smaller than param_release_dur
    }
  }
  if(param_eventInterval[param_eventnums-1]<=param_reward_dur){
    params_legal_flag = false;
    RAISE_ERROR(__LINE__);//PARAMERR:last interval should loger than reward_dur
  }
  if(param_led_brightness == 0){
    params_legal_flag = false;
    RAISE_ERROR(__LINE__); //PARAMERR:all led closed by param 25, this param cannot be 0!
  }
  return params_legal_flag;
}


bool training_init(void){
  //1. verify parameters
  bool verify_result = verify_params();
  if(!verify_result){
    return false;
  }
  //2. update event_pin/led
  for(int i=0;i<param_eventnums;i++){//[0] is event1,[1] is event2
    event_pin[i] = (param_event_lever[i] == 1)?pin_lever[0]:
                   (param_event_lever[i] == 2)?pin_lever[1]:
                   (param_event_lever[i] == 3)?pin_lever[2]:0;//pin=0 means closed
    event_led[i] = (param_event_lever[i] == 1)?pin_LED[0]:
                   (param_event_lever[i] == 2)?pin_LED[1]:
                   (param_event_lever[i] == 3)?pin_LED[2]:0;//pin=0 means closed
  }  
  return true;
}

void trial_init(void){
  sensor_reset();
  current_event_num = 1;
  trial_fail_flag   = 0;
  trial_start_ms  = millis();
  event_start_ms  = millis();
}

bool update_event_state(int event_execute_time,int press_result){
  bool flag_normal_event = (current_event_num==1 || 
                            current_event_num==2 ||
                            current_event_num==3);
  if(flag_normal_event){     //current_event_num==1/2/3 -> event1/2/3
    if(press_result){//press happening
      current_event_num += 0.5;
      Serial1.print("event: ");
      Serial1.println(current_event_num);
      event_start_ms = millis();
    }
    else if(event_execute_time > param_pretimewindow[(int)(current_event_num-1)] + param_posttimewindow[(int)(current_event_num-1)]){//no press
      trial_fail_flag  = 1; 
      RAISE_HINT(__LINE__);
    }
  }
  else{ //current_event_num==1.5/2.5/3.5 -> interval1/2/3
    if((event_execute_time > param_release_dur) && press_result){//press outside the release window:detect continuously press
      trial_fail_flag  = 1;
      RAISE_HINT(__LINE__);
    }
    else if(event_execute_time > param_eventInterval[(int)(current_event_num-1)]){
      current_event_num += 0.5;
      event_start_ms = millis();       
    } 
  }

  bool exit_flag  = (current_event_num == 4) || (trial_fail_flag ==1);
  return exit_flag;
}

