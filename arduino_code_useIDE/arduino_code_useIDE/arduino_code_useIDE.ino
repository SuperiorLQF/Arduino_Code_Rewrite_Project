#include "init.h"
//#include <DueTimer.h>
#include <SPI.h>
#include <sam.h>
// communicating headers
const uint8_t ENCODER_L = 100;
const uint8_t TIME_L = 101;

// Stimulus channels (as defined in Matlab code)
const int ch_brightled = 1;
const int ch_puffer_other = 2;
const int ch_puffer_eye = 3;
const int ch_tone = 5;
const int ch_elec = 7;
const int ch_laser = 8;
const int ch_odor1 = 10;
const int ch_odor2 = 11;
const int ch_odor3 = 12;
const int ch_odor4 = 13;
const int ch_solenoid = 14;

//PARAMS receive From MATLAB
int param_campretime  = 200;
int param_camposttime = 800;
int param_csdur       = 500;
int param_cs2dur      = 500;
int param_cs3dur      = 500;
int param_delay1      = 500;
int param_delay2      = 500;
int param_usdur       = 20;
int param_ISI         = 200;
int param_stimdelay   = 0; 
int param_stimdur     = 500;
int param_toneFrequency   = 10;//10kHz
int param_elecFrequency   = 300;//300Hz
int param_elecDur         = 300;//300us
int param_elecAmp         = 1000;//1000mV
int param_elecPolar       = 1;//0:NP  1:PN
int param_laserFrequency  = 300;//300Hz
int param_laserDur        = 300;//300us
int param_laserAmp        = 1000;//1000mV

int param_csch    = ch_brightled;
int param_cs2ch   = ch_odor2;
int param_cs3ch   = ch_odor3;
int param_usch    = ch_puffer_eye;   
int param_stimch  = ch_tone; 

//pins
const int pin_brightled = 7;
const int pin_whisker   = 10;
const int pin_eye_puff  = 13;
const int DAC_PIN       = DAC0;
const int pin_elec      = 34;//FIXME:unused,just a flag
const int pin_laser     = 12;//FIXME:unused,just a flag
const int pin_odor1     = 30;
const int pin_odor2     = 31;
const int pin_odor3     = 32;
const int pin_odor4     = 33;
const int pin_solenoid  = 37;

const int pin_camera    = 8;
const int pin_enc_pin1  = 2;
const int pin_enc_pin2  = 3;
const int pin_pump      = 22;
const int pin_LED1      = 2;
const int pin_LED2      = 3;
const int pin_camera2   = 25;
const int pin_go1       = 26;
const int pin_go2       = 27;
const int pin_fv        = 36;

const int pin_lick      = 41; // after 41, for sensors

//DAC8563 pins
const int dac8563_sync_pin = 35;//DAC SPI CS:active low
const int dac8563_ldac_pin = 38;//always HIGH
const int dac8563_clr_pin  = 39;//active LOW

const int stim2pinMapping[16] {
  0,
  pin_brightled,    //1
  pin_whisker,
  pin_eye_puff,
  0,
  DAC_PIN,          //5
  0,
  pin_elec,         //ELEC_PIN,
  pin_laser,        //8
  0,
  pin_odor1,        //10
  pin_odor2,
  pin_odor3,
  pin_odor4,
  pin_solenoid,
  0
};

uint8_t   sensor_readings[SENSOR_BUF_MAXLEN];
uint8_t   sensor_times[10];
int       sensor_data_index  = 0;
int       sensor_buf_sendlen = 1000;

float     laserAmp_tmp;//32768*param_laserAmp/10000 +32768;
uint16_t  laserAmp;//(laserAmp_tmp>65535)?65535:laserAmp_tmp;//to avoid overflow
bool      laser_positive_flag;
int       laser_period_us;//1000000 / param_laserFrequency;
int       laser_duration_us;//param_laserDur;

float     elecAmp_tmp;//32768*param_elecAmp/10000 +32768;
uint16_t  elecAmp;//(elecAmp_tmp>65535)?65535:elecAmp_tmp;//to avoid overflow
float     elecAmp_negtive_tmp;
uint16_t  elecAmp_negtive;
Elec_status   elec_state_flag;//see in init.h
int       elec_period_us;//1000000 / param_laserFrequency;
int       elec_duration_us;//param_laserDur;

int pin_value_memory[70];//-1:initial  0:off 1:on

bool debug_first_enter_interrupt = false;
int  debug_us_a,debug_us_b,debug_us_c,debug_us_d,debug_us_e;
int  laser_counter_number_set_A,laser_counter_number_set_B;
void setup() {  // put your initialization code here, to run once:
  setup_serial();
  setup_SPI();
  setup_pins();
  dac8563_init();

  randomSeed(micros());//use a random value(like time or adc value)as a random seed
  delay(10);

  //add communication code test for debug here
}

void loop() {  // put your main code here, to run repeatedly:

  check_matlab_message();

}

//================setup function [begin]========================//
void setup_serial(void){
  Serial.begin(115200);
  Serial1.begin(115200);
  while(!Serial && !Serial1){ ;}//wait for Serial ready
}
void setup_SPI(void){
  SPI.begin();
  SPI.beginTransaction(SPISettings(50000000,MSBFIRST,SPI_MODE1));//SPI_speed:10MHz max:50M *Note:when debugging,use 2MHz because logic analyzer cannot capture too fast signal
}
void setup_pins(void){
  //setup pin value memory
  for(int i=0;i<70;i++){
    pin_value_memory[i] = -1;
  }
  //setup all channel pins
  for(int i=0;i<16;i++){
    if(stim2pinMapping[i] != 0){
      pin_driver(stim2pinMapping[i],0);
    }
  }
  //setup dac pins
  pin_driver(dac8563_sync_pin,1);
  pin_driver(dac8563_ldac_pin,0);
  pin_driver(dac8563_clr_pin ,1);
  //setup camera
  pin_driver(pin_camera,0);

  Serial1.println("device set up OK");
}

void sensor_reset(void){//enc only setup in conditioning lab
  pinMode(pin_enc_pin1, INPUT_PULLUP);
  pinMode(pin_enc_pin2, INPUT_PULLUP);
  pinMode(pin_lick,     INPUT_PULLUP);
  sensor_data_index = 0;
  sensor_buf_sendlen = param_campretime + param_camposttime;
  for(int i=0;i<SENSOR_BUF_MAXLEN;i++){ //initialize data
    sensor_readings[i] = 0xFF;
  }
}
//================setup function [end]-========================//


//================pin driver function [begin]==================//
void pin_driver(int pin_number,int value){ // pin 0,1 is used for serial communication. DO NOT driver it other than use Serial
  //only act when [1.first init]  OR [2.pin value change] ,don't act when value is not change
  int   period_us;
  int   duration_us;
  bool  setup_flag = (pin_value_memory[pin_number] == -1);//if 1 ,we should set pinMode before drive
  bool  value_change_flag = (pin_value_memory[pin_number] != value);//if 1,the value change,we should drive it

  if(pin_number == 0 || pin_number == 1){
    RAISE_ERROR(__LINE__);// pin 0,1 is used for serial communication. DO NOT driver it
    return;
  }

  if(setup_flag || value_change_flag){
    pin_value_memory[pin_number] = value;//update value memory

    //++++digital pin++++++++
    if (pin_number <54 && pin_number!=stim2pinMapping[ch_elec] && pin_number!=stim2pinMapping[ch_laser]){
      if(setup_flag){ 
        pinMode(pin_number,OUTPUT);
      } /*setup*/
        digitalWrite(pin_number,value);
    }    

    //++++++tone+++++++++++++
    else if(pin_number == stim2pinMapping[ch_tone]){
      // if(setup_flag){
      //   analogWriteResolution(12);//setup 
      //   analogWrite(pin_number, 0);
      // }
      // else if(value!=0){
      //   Timer1.attachInterrupt(tone_TIMER_HANDLER);//call this function every period_us;
      //   period_us = 1000 / param_toneFrequency;
      //   if(period_us > 9){//because of hardware limitation,DAC Max Freq  bigger than 100kHz
      //     Timer1.start(period_us);//us
      //   }else{
      //     RAISE_ERROR(__LINE__);//ERROR:param_tone Frequency too HIGH!
      //   }
      // }
      // else{
      //   Timer1.stop();
      //   analogWrite(pin_number, 0);
      //   //FIXME:we can turn to digital mode to set value to 0 V
      // }
    }

    //++++++++elec++++++++++++
    else if(pin_number == stim2pinMapping[ch_elec]){//Timer2 and channelA //!!!FIXME:change 1 to 0(port A)
      // if(setup_flag){
      //   dac8563_output(0,32768);//32768 is 0V //!!!
      //   elec_state_flag = (param_elecPolar?PN_P:NP_N);
      // }
      // else if(value!=0){
      //   Timer2.attachInterrupt(elec_TIMER_HANDLER);//call this function every period_us;

      //   elecAmp_tmp         = 32768*param_elecAmp/10000 +32768;
      //   elecAmp             = (elecAmp_tmp>65535)?65535:elecAmp_tmp;//convert from mv to DAC number
      //   elecAmp_negtive_tmp = 32768 - 32768*param_elecAmp/10000;
      //   elecAmp_negtive     = (elecAmp_negtive_tmp<0)?0:elecAmp_negtive_tmp;//convert from mv to DAC number
      //   elec_period_us      = 1000000 / param_elecFrequency;
      //   elec_duration_us    = param_elecDur;    

      //   if((elec_duration_us > 9) && (elec_period_us - 2*elec_duration_us>9)){//because of hardware limitation,avoid Max Freq bigger than 100kHz,you can speed up SPI clock to push this threshold
      //     elec_state_flag = (param_elecPolar?PN_P:NP_N);
      //     dac8563_output(0,((elec_state_flag == NP_N)?elecAmp_negtive:elecAmp));
      //     Timer2.start(elec_duration_us);
      //   }else{
      //     RAISE_ERROR(__LINE__);//ERROR:param_elecFrequency and param_elecDur value are illegue!
      //   }    
      // }
      // else{
      //   Timer2.stop();
      //   dac8563_output(0,32768);//32768 is 0V
      //   elec_state_flag = (param_elecPolar?PN_P:NP_N);
      // }
    }
    
    //laser
    else if(pin_number == stim2pinMapping[ch_laser]){//Timer3 and channelB
      if(setup_flag){
        dac8563_output(1,32768);//32768 is 0V
        laser_positive_flag = true;
      }
      else if(value!=0){
        //Timer3.attachInterrupt(laser_TIMER_HANDLER);//call this function every period_us;

        laserAmp_tmp      = 32768*param_laserAmp/10000 +32768;
        laserAmp          = (laserAmp_tmp>65535)?65535:laserAmp_tmp;//to avoid overflow
        laser_period_us   = 1000000 / param_laserFrequency;
        laser_duration_us = param_laserDur;

        if((laser_duration_us > 9) && (laser_period_us-laser_duration_us>9)){//because of hardware limitation,avoid Max Freq bigger than 100kHz,you can speed up SPI clock to push this threshold
          laser_positive_flag = true;
          //Timer3.start(laser_duration_us);
          timer_init(3);
          dac8563_output(1,laserAmp);
        }else{
          RAISE_ERROR(__LINE__);//ERROR:param_laserFrequency and param_laserDur value are illegue!
        }
      }
      else{
        //Timer3.stop();
        TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;//close the timer
        laser_positive_flag = true;
        dac8563_output(1,32768);//32768 is 0V
      }
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


//=============start one trial of conditioning  [begin]=========//
void startConditioning(void){
  unsigned long slot_start_us,slot_intermediate_us;
  int execute_us;
  unsigned long trial_start_ms,trial_current_ms;
  int execute_ms;
  bool end_flag = false;

  // --- initialization ----
  trial_start_ms = millis();
  sensor_reset();  // set enc pins as input mode, and reset the data buffer

  //--------------Conditioning LOOP [begin]-----------//
  while(!end_flag){//a trial is divided into 1ms slots
    slot_start_us = micros();
    trial_current_ms = millis();
    execute_ms = trial_current_ms - trial_start_ms;

    //#### The main process begin ####
    bool end_flag_1=output_controller(execute_ms);
    sensor_input_reader(execute_ms);

    end_flag = end_flag_1 && (execute_ms > param_campretime+param_camposttime); //
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
  //--------------Conditioning LOOP [end]-------------//

  RAISE_HINT(__LINE__);
}
//================ start one trial of conditioning  [end]=======//

//================ conditioning functions [begin]===============//
bool output_controller(int execute_ms){
  bool exit_flag;
  Pin_information pin_cam_info; //Pin_information is a used-defined variable type (it has pin_status and pin_value)
  Pin_information pin_stim_info;     // see main.h
  Pin_information pin_cs_info;
  Pin_information pin_cs2_info;
  Pin_information pin_cs3_info;
  Pin_information pin_us_info;  
  //--------------- 1.get pin output information (status and value) ---------------
  //                                 |delay                                  |duration                             |current_time
  pin_cam_info   = update_output_info(0                                      ,param_campretime+param_camposttime   ,execute_ms);
  pin_stim_info  = update_output_info(param_campretime+param_stimdelay       ,param_stimdur                        ,execute_ms);
  pin_cs_info    = update_output_info(param_campretime                       ,param_csdur                          ,execute_ms);
  pin_cs2_info   = update_output_info(param_campretime+param_delay1          ,param_cs2dur                         ,execute_ms);
  pin_cs3_info   = update_output_info(param_campretime+param_delay2          ,param_cs3dur                         ,execute_ms);
  pin_us_info    = update_output_info(param_campretime+param_ISI             ,param_usdur                          ,execute_ms); 

  bool pin_fv_value = pin_cs_info.pin_value || pin_cs2_info.pin_value || pin_cs3_info.pin_value;

  //--------------  2.drive the pin (on/off control by arduino built-in function, digitalWrite) ---------------
  pin_driver(pin_camera                   ,pin_cam_info    .pin_value);//camera
  pin_driver(stim2pinMapping[param_stimch],pin_stim_info   .pin_value);//stim
  pin_driver(stim2pinMapping[param_csch]  ,pin_cs_info     .pin_value);//cs
  pin_driver(stim2pinMapping[param_cs2ch] ,pin_cs2_info    .pin_value);//cs2
  pin_driver(stim2pinMapping[param_cs3ch] ,pin_cs3_info    .pin_value);//cs3
  pin_driver(stim2pinMapping[param_usch]  ,pin_us_info     .pin_value);//us
  pin_driver(pin_fv                       ,pin_fv_value              );//fv

  //--------------  3.caculate exit flag condition ---------------
  exit_flag =       (pin_cam_info     .pin_status == FINISH)  && 
                    (pin_stim_info    .pin_status == FINISH)  && 
                    (pin_cs_info      .pin_status == FINISH)  && 
                    (pin_cs2_info     .pin_status == FINISH)  && 
                    (pin_cs3_info     .pin_status == FINISH)  && 
                    (pin_us_info      .pin_status == FINISH);
  return exit_flag;
}

void sensor_input_reader(int execute_ms){
  if (sensor_data_index<SENSOR_BUF_MAXLEN) {
    sensor_readings[sensor_data_index] = digitalRead(pin_lick)*4 + digitalRead(pin_enc_pin2)*2 + digitalRead(pin_enc_pin1);
  }
  else {
    RAISE_ERROR(__LINE__);//SENSOR_BUF_MAXLEN is to short
  }
  if (sensor_data_index<5) {
    sensor_times[sensor_data_index] = execute_ms;  //  (uint16_t)execute_ms;
  }
  sensor_data_index++ ;
}

  /*encode pattern 
  ______________________________________
  **bit[7:0] == 0xFF  | value is invalid
  ______________________________________
  bit [7:3]           | reserved
  bit [2]             | lick data
  bit [1]             | enc_pin2
  bit [0]             | enc_pin1
  ______________________________________
  */
//================conditioning functions [end]=====================//




//================communicate function (from matlab) [begin]=================//
//All data(params/command) has the same 4Byte structure to simplify communication with MATLAB
//2Bytes header + 2Bytes body, header=1 implify body is command
void check_matlab_message(void) {
  uint8_t param_block[4];
  uint8_t header;
  uint8_t value;
  int available_number;
  int command;
  while (Serial.available() > 0 ) {
    //wait for receiving 4Byte data
    delay(10);  //one byte cost 0.1ms at 115200 baudrate
    
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
          param_campretime = value;
          break;
        case 4:
          param_csch = value;
          break;
        case 5:
          param_csdur = value;
          break;
        case 6:
          param_usdur = value;
          break;
        case 7:
          param_ISI = value;
          break;
        case 8:
          param_toneFrequency = value;//kHz
          break;
        case 9:
          param_camposttime = value;
          break;
        case 10:
          param_usch = value;
          break;
        case 11:
          param_stimdelay = value;
          break;
        case 12:
          param_stimdur = value;
          break;
        case 13:
          //param_laserpower = value;
          break;

        case 15:
          //param_laserperiod = value;
          break;
        case 16:
          //param_lasernumpulses = value;
          break;
        // case 20:
        //   param_csperiod = value;
        //   break;
        // case 21:
        //   param_csrepeats = value;
        //   break;
        case 22:
          //param_rampoffdur = value; //ALvaro 10/19/18
          break;
        case 24:
          param_elecDur = value;//us
          break;
        case 25:
          param_elecAmp = value;//mv
          break;
        case 26:
          //param_elecRepeats = value;
          break;
        case 27:
          param_laserAmp = value;//mv
        case 28:
          param_laserDur = value;//us
          break;
        case 29: 
          //param_laserRepeats = value;
          break;
        case 30: 
          //param_toneAmp = value;//mv   NOTE:We use random wave so this is not use anymore
          break; 
        case 31: 
          param_elecFrequency = value;//Hz
          break; 
        case 32: 
          param_laserFrequency = value;//Hz
          break;
        case 33: 
          param_stimch = value;
          break;
        case 34:
          param_elecPolar = value;
          break;
        case 35:
          //event1_pin= value;
          break;
        case 36:
          //event2_pin= value;
          break;
        case 37:
          //event3_pin= value;
          break;
        case 38:
          //trialnums = value;
          break;
        case 39:
          //Interval1 = value;
          break;
        case 40:
          //Interval2 = value;
          break;
        case 41:
          //ITI = value;
          break;
        case 42:
          //pre_timewindows = value;
          break;
        case 43:
          //post_timewindows = value;
          break;
        case 44:
          //rewardDuration = value;
          break;
        case 45:
          break;
        case 46:
          //current_brightness = value;
          break;
        case 47:
          param_cs2ch = value;
          break;
        case 48:
          param_cs2dur = value;
          break;
        case 49:
          param_cs3ch = value;
          break;
        case 50:
          param_cs3dur = value;
          break;
        case 51:
          param_delay1 = value;
          break;
        case 52:
          param_delay2 = value;
          break;
        case 54:
          //pump_dur = value;
          break;
        default:
          break;
      }

    }
    else {
      //illegal data format other than 4Bytes
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
          startConditioning();
          break;
      case 2: 
          sendSensorData();
          break;
      case 3:
          //elecOn(0);
          break;
      case 4:
          //elecOff(0);
          break;
      case 5: 
          //laserOn(0);
          break;
      case 6:
          //laserOff(0);
          break;
      case 7:
          //toneOn(DAC_PIN);
          break;
      case 8:
          //toneOff(DAC_PIN);
          break;
      case 9:
          //startLeverTask();  // previously "startTrial()"
          break;
      default:
          break;
  }
}


void sendSensorData(void) {
    Serial.write(ENCODER_L);
    for (int i=0; i<sensor_buf_sendlen; i++) {//FIXME:change param_encodernumreadings to SENSOR_BUF_SENDLEN
        writeByte(sensor_readings[i]);
    }

    Serial.write(TIME_L);
    for (int i=0; i<4; i++) {//FIXME:change param_encodernumreadings to SENSOR_BUF_SENDLEN
        writeByte(sensor_times[i]);
    }
    RAISE_HINT(__LINE__);
}
//================communicate function [end]=================//

//================send data function (to matlab) [begin]===================//
void writeByte(uint8_t val){
  Serial.write(val);
}
void writeTwoByte(uint16_t val){
  uint8_t low_Byte   = val & 0xFF;
  uint8_t high_Byte  = val >> 8; 

  Serial.write(low_Byte);//send low_byte first
  Serial.write(high_Byte);
}
//================send data function [end]=====================//

//================print info function [begin]==================//
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
//================print info function  [end]====================//

//================TIMER CALLBACK function[begin]================//
// void tone_TIMER_HANDLER(void){
//   int random_value = random(0, 4096);
//   analogWrite(stim2pinMapping[ch_tone],random_value);//
// }
void TC3_Handler(){//predefined function name in chip core lib, Timer3 callback function
  TC1->TC_CHANNEL[0].TC_SR;  // clear interrupt flag

  if(!debug_first_enter_interrupt){  //!!!
    debug_us_a = micros();//!!!
  }  //!!!

  if(laser_positive_flag == false){
    laser_positive_flag = true;
    dac8563_output(1,laserAmp);
    //Timer3.start(laser_duration_us);
    TC1->TC_CHANNEL[0].TC_RC = laser_counter_number_set_A;//reset

  }else{
    debug_us_b = micros();//!!!
    laser_positive_flag = false;
    debug_us_c = micros();//!!!
    dac8563_output(1,32768);//32768 is 0V
    debug_us_d = micros();//!!!
    //Timer3.start(laser_period_us  - laser_duration_us);
    TC1->TC_CHANNEL[0].TC_RC = laser_counter_number_set_B;//reset
    debug_us_e = micros();//!!!
  }

  if(!debug_first_enter_interrupt){//!!!
    debug_us_b = debug_us_b-debug_us_a;//!!!
    debug_us_c = debug_us_c-debug_us_a;//!!!
    debug_us_d = debug_us_d-debug_us_a;//!!!
    debug_us_e = debug_us_e-debug_us_a;//!!!
    Serial1.println(debug_us_b);//!!!
    Serial1.println(debug_us_c);//!!!
    Serial1.println(debug_us_d);//!!!
    Serial1.println(debug_us_e);//!!!
    debug_first_enter_interrupt = true;//!!!
  }//!!!
}
// void elec_TIMER_HANDLER(void){//!!!FIXME:change 1 to 0(port A)
//   if(elec_state_flag == NP_N){
//     elec_state_flag = NP_P;
//     dac8563_output(0,elecAmp);
//     Timer2.start(elec_duration_us);
//   }
//   else if(elec_state_flag == NP_P){
//     elec_state_flag = NP_LOW;
//     dac8563_output(0,32768);
//     Timer2.start(elec_period_us - 2*elec_duration_us);
//   }
//   else if(elec_state_flag == NP_LOW){
//     elec_state_flag = NP_N;
//     dac8563_output(0,elecAmp_negtive);
//     Timer2.start(elec_duration_us);
//   }
//   else if(elec_state_flag == PN_P){
//     elec_state_flag = PN_N;
//     dac8563_output(0,elecAmp_negtive);
//     Timer2.start(elec_duration_us);    
//   }
//   else if(elec_state_flag == PN_N){
//     elec_state_flag = PN_LOW;
//     dac8563_output(0,32768);
//     Timer2.start(elec_period_us - 2*elec_duration_us);  
//   }
//   else if(elec_state_flag == PN_LOW){
//     elec_state_flag = PN_P;
//     dac8563_output(0,elecAmp);
//     Timer2.start(elec_duration_us);    
//   }
// }
//================TIMER CALLBACK function[end]==================//

//================DAC 8563 driverfunction[begin]================//
void dac8563_init(void){
  //setup dac control pins
  digitalWrite(dac8563_clr_pin ,0);//hardware reset
  delay(1);
  digitalWrite(dac8563_clr_pin ,1);
  writeDAC(0x28,0x0001);//soft reset
  delay(1);
  writeDAC(0x20,0x0003);//power on
  delay(1);
  writeDAC(0x38,0x0001);//inner ref enable
  delay(1);
  writeDAC(0x02,0x0000);//set AB gain = 2 so output is ±10V
  delay(1);
  dac8563_output(0,32768);//CHANNEL A 32768 is 0V
  dac8563_output(1,32768);//CHANNEL B 32768 is 0V

}

void dac8563_output(int chan_num,uint16_t data){
  float real_voltage;
  //real_voltage = (data-32768)/3276.8; 
  if(chan_num==0){//PORT_A
    writeDAC(0x18,data);
    // Serial1.print("DAC8563 PORT_A:");
    // Serial1.println(real_voltage);
  }
  else if(chan_num==1){//PORT_B
    writeDAC(0x19,data);
    // Serial1.print("DAC8563 PORT_B:");
    // Serial1.println(real_voltage);
  }
  else{
    ;
  }
}

void writeDAC(uint8_t cmd,uint16_t data){
  digitalWrite(dac8563_sync_pin,0);
  SPI.transfer(cmd);
  SPI.transfer((data >> 8) & 0xFF);//high byte of data
  SPI.transfer(data & 0xFF);  //low byte of data    
  digitalWrite(dac8563_sync_pin,1);  
}
//================DAC 8563 driverfunction[end]==================//

//========================timer function[begin]================//
void timer_init(int timer_number){//!!!current open timer3 for test
  if(timer_number==3){
  //Timer3 is TC1 CHANNEL0
  pmc_enable_periph_clk(ID_TC3);//open timer clock source 

  TC1->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK2 | //configure channel mode register (CMR) CLOCK2 is source(84M)/8 = 10.5MHz(95.238ns)(Spec page 881)
                              TC_CMR_CPCTRG;               //use RC compare interrupt(Spec page 882)

  laser_counter_number_set_A = laser_duration_us * 1000 / 95.238 -1;
  laser_counter_number_set_B = (laser_period_us  - laser_duration_us)*1000 / 95.328 -1;
  TC1->TC_CHANNEL[0].TC_RC  = laser_counter_number_set_A;//setup counter number (Spec page 891)
  
  TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;//open timer interrupt call-back
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);

  TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | //start timer
                              TC_CCR_SWTRG;
  }
};


//========================timer function[end]==================//