#include "main.h"
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
const int ELEC_PIN      = 34;
const int pin_laser     = 12;
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
const int pin_lick      = 28;

const int pin_fv        = 36;

const int stim2pinMapping[16] {
  0,
  pin_brightled,
  pin_whisker,
  pin_eye_puff,
  0,
  DAC_PIN,
  0,
  ELEC_PIN,
  pin_laser,
  0,
  pin_odor1,
  pin_odor2,
  pin_odor3,
  pin_odor4,
  pin_solenoid,
  0
};

uint8_t   enc_readings[SENSOR_BUF_MAXLEN];
uint8_t  enc_times[10];
int       enc_data_index  = 0;


void setup() {
  // put your setup code here, to run once:
  setup_serial();
  setup_pins();
  delay(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(200);//wait for all params transmit into buffer//FIXME:to check if the receive buffer is overflow
  checkVars();
  int command = checkCMD();
  switch(command) {
      case 1:
          startConditioning();
          break;
      case 2: 
          sendEncoderData();
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
//================setup function [begin]========================//
void setup_serial(void){
  Serial.begin(115200);
  Serial1.begin(115200);
  while(!Serial && !Serial1){ ;}//wait for Serial ready
}
void setup_pins(void){
  //setup all channel pins
  for(int i=0;i<16;i++){
    if(stim2pinMapping[i]!=0){
      pinMode(stim2pinMapping[i],OUTPUT);
      digitalWrite(stim2pinMapping[i],LOW);
    }
  }
  //setup camera
  pinMode(pin_camera,OUTPUT);
  digitalWrite(pin_camera,0);

  Serial1.println("device set up OK");
}

void enc_reset(void){//enc onlu setup in conditioning lab
  pinMode(pin_enc_pin1, INPUT_PULLUP);
  pinMode(pin_enc_pin2, INPUT_PULLUP);
  enc_data_index = 0;
  for(int i=0;i<SENSOR_BUF_MAXLEN;i++){//initialize data
    enc_readings[i] = 0xFF;
  }
}
//================setup function [end]-========================//


//================pin driver function [begin]==================//
void pin_driver(int pin_number,bool on){//to make every pin with unified interface,you can add if-conditions if some pins have different or specific relization
  digitalWrite(pin_number,on);//on == true:open;   on ==flase:close
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
    pin_information_tmp.pin_status = DELAY;
    pin_information_tmp.pin_value  = 0;
  }
  return pin_information_tmp;

}
//================pin driver function [end]======================//


//================conditioning function [begin]=====================//
void startConditioning(void){
  unsigned long slot_start_us,slot_intermediate_us;
  int execute_us;
  unsigned long trial_start_ms,trial_current_ms;
  int execute_ms;
  bool end_flag = false;
  //Preprocess code begin
  enc_reset();  // set enc pins as input mode, and reset the data buffer
  //Preprocess code end
  trial_start_ms = millis();

  //================Conditioning LOOP [begin] =========================//
  while(!end_flag){//a trial is divided into 1ms slots
    slot_start_us = micros();
    trial_current_ms = millis();
    execute_ms = trial_current_ms - trial_start_ms;

    //#### The main process begin ####
    bool end_flag_1=output_controller(execute_ms);
    sensor_input_reader(execute_ms);
    //#### The main process end ####

    end_flag = end_flag_1 && (execute_ms > param_campretime+param_camposttime); //
    
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
  //================Conditioning LOOP [end]   =========================//

  RAISE_HINT(__LINE__);
}

bool output_controller(int execute_ms){
  bool exit_flag;
  Pin_information pin_cam_info; //Pin_information is a used-defined variable type (it has pin_status and pin_value)
  Pin_information pin_stim_info;     // see main.h
  Pin_information pin_cs_info;
  Pin_information pin_cs2_info;
  Pin_information pin_cs3_info;
  Pin_information pin_us_info;  
  //======= 1.get pin output information (status and value) =============
  //                                 |delay                                  |duration                             |current_time
  pin_cam_info   = update_output_info(0                                      ,param_campretime+param_camposttime   ,execute_ms);
  pin_stim_info  = update_output_info(param_campretime+param_stimdelay       ,param_stimdur                        ,execute_ms);
  pin_cs_info    = update_output_info(param_campretime                       ,param_csdur                          ,execute_ms);
  pin_cs2_info   = update_output_info(param_campretime+param_delay1          ,param_cs2dur                         ,execute_ms);
  pin_cs3_info   = update_output_info(param_campretime+param_delay2          ,param_cs3dur                         ,execute_ms);
  pin_us_info    = update_output_info(param_campretime+param_ISI             ,param_usdur                          ,execute_ms); 

  bool pin_fv_value = pin_cs_info.pin_value || pin_cs2_info.pin_value || pin_cs3_info.pin_value;

  //======== 2.drive the pin (on/off control by arduino built-in function, digitalWrite) ====================================
  digitalWrite(pin_camera                   ,pin_cam_info    .pin_value);//camera
  digitalWrite(stim2pinMapping[param_stimch],pin_stim_info   .pin_value);//stim
  digitalWrite(stim2pinMapping[param_csch]  ,pin_cs_info     .pin_value);//cs
  digitalWrite(stim2pinMapping[param_cs2ch] ,pin_cs2_info    .pin_value);//cs2
  digitalWrite(stim2pinMapping[param_cs3ch] ,pin_cs3_info    .pin_value);//cs3
  digitalWrite(stim2pinMapping[param_usch]  ,pin_us_info     .pin_value);//us
  digitalWrite(pin_fv                       ,pin_fv_value              );//fv

  //======== 3.caculate exit flag condition =========================
  exit_flag =       (pin_cam_info     .pin_status == FINISH)  && 
                    (pin_stim_info    .pin_status == FINISH)  && 
                    (pin_cs_info      .pin_status == FINISH)  && 
                    (pin_cs2_info     .pin_status == FINISH)  && 
                    (pin_cs3_info     .pin_status == FINISH)  && 
                    (pin_us_info      .pin_status == FINISH);
  return exit_flag;
}

void sensor_input_reader(int execute_ms){
  enc_readings[enc_data_index] = digitalRead(pin_enc_pin2)*2 + digitalRead(pin_enc_pin1);
  /*encode pattern 
  value encoded  |enc_pin1    |  enc_pin2 | Note
  FF             |   ---      |    ---    | *The value is invalid(surpass the trial)
  0              |    0       |     0     |
  1              |    1       |     0     |
  2              |    0       |     1     |
  3              |    1       |     1     |
  */
 if (enc_data_index<5) {
  //enc_times[enc_data_index] = (uint16_t)execute_ms;
  enc_times[enc_data_index] = execute_ms;
}
  enc_data_index++ ;
}

//================conditioning function [end]=====================//

//================communicate function (from matlab) [begin]=================//
void checkVars(void) {
  char param_block[3];
  int header;
  int value;
  // Matlab sends data in 3 byte packets: first byte is header telling which variable to update
  // next two bytes are the new variable data as 16 bit int 
  // Header is coded numerically (0, 1, and 2 are reserved for special functions so don't use them to code variable identities)
  while (Serial.available() >=3 ) {
    Serial.readBytes(param_block,3);
    header = param_block[0];
    value = param_block[1] | param_block[2] << 8;
    Serial1.print("header:");
    Serial1.println(header);
    Serial1.print("value:");   
    Serial1.println(value); 
    // If you add a new case don't forget to put a break statement after it; c-style switches run through
    switch (header) {
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
        //param_tonefreq = value;
        break;
      case 9:
        //param_camposttime = value;
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
      case 45:
        //param_csintensity = value; Alvaro 05/09/19 from sheiney committed on Jan 18, 2017
        // setDiPoValue(param_csintensity);
        // The Matlab code stores intensity values up to 256 because it's nicer to deal with   Alvaro 05/09/19 sheiney committed on Jan 18, 2017
        // multiples of 2 but we can only pass at most 255 so we have to correct that here.  Alvaro 05/09/19 sheiney committed on Jan 18, 2017
        // Zero is a special case because when the user specifies 0 they want it to mean "off"  Alvaro 05/09/19 sheiney committed on Jan 18, 2017
        //param_csintensity = value==0 ? value : value-1; //Alvaro
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
        //param_elecPeriod = value;
        break;
      case 25:
        //param_elecVoltage = value;
        break;
      case 26:
        //param_elecRepeats = value;
        break;
      case 27:
        //param_laserVoltage = value;
      case 28:
        //param_laserPeriod = value;
        break;
      case 29: 
        //param_laserRepeats = value;
        break;
      case 30: 
        //param_toneamp = value;
        break; 
      case 31: 
        //param_elecFrequency = value;
        break; 
      case 32: 
        //param_laserFrequency = value;
        break;
      case 33: 
        param_stimch = value;
        break;
      case 34:
        //reversePolarity = value;
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
        //param_delay1 = value;
        break;
      case 52:
        //param_delay2 = value;
        break;
      case 54:
        //pump_dur = value;
        break;
    }
    // We might be able to remove this delay if Matlab sends the parameters fast enough to buffer
    delay(10); // Delay enough to allow next 3 bytes into buffer (24 bits/115200 bps = ~200 us, so delay 1 ms to be safe).
  } 
}

int checkCMD(void){
  int times_retry=0;
  for(;;){
    if (Serial.available() > 0) {

        int command = Serial.read(); 

        Serial1.print("get command:");
        Serial1.println(command);

        return command;
    }
    else{
        if(times_retry>100){
          RAISE_WARNING(__LINE__);
          Serial1.println("Fail to get command");
          times_retry = 0;
        }
        delay(10);
        times_retry++;
    }
  }
}

void sendEncoderData(void) {
    Serial.write(ENCODER_L);
    for (int i=0; i<SENSOR_BUF_SENDLEN; i++) {//FIXME:change param_encodernumreadings to SENSOR_BUF_SENDLEN
        writeByte(enc_readings[i]);
    }

    Serial.write(TIME_L);
    for (int i=0; i<4; i++) {//FIXME:change param_encodernumreadings to SENSOR_BUF_SENDLEN
        writeByte(enc_times[i]);
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


