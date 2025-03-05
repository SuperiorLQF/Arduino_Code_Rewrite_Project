// TODO: Put Arduino to sleep when not running neuroblinks (e.g. pmc_enable_sleepmode())
// TODO: Deal with overflow of millisecond/microsecond timers
// TODO: Non-blocking Serial IO and better serial communication generally (consider Serial.SerialEvent())

#include "main.hpp"
#include <Encoder.h>

#define CS_PIN 35 

#define CMD_SETA_UPDATEA 0x18 
#define CMD_SETB_UPDATEB 0x19
#define CMD_RESET_ALL_REG 0x28 
#define CMD_PWR_UP_A_B 0x20    
#define CMD_INTERNAL_REF_EN 0x38 
#define CMD_GAIN 0x02  

#define DACC_MAX_VALUE 4095 
#define DACC_INTERFACE DACC

uint8_t   enc_readings[SENSOR_BUF_MAXLEN];
uint16_t  enc_times[SENSOR_BUF_MAXLEN];
int       enc_data_index=0;

const float VREFB = 10;   
const float VREFA = 5.0;

unsigned long param_elecPeriod = 1000;
unsigned long param_laserPeriod = 500;

unsigned long param_elecFrequency = 1000;
unsigned long param_laserFrequency = 1000;


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


// Outputs
const int pin_ss = 4;  // slave select Pin. need one for each external chip you are going to control.
const int pin_brightled = 7;
const int pin_camera = 8;
const int pin_led = 9;
const int pin_whisker = 10;
const int pin_tone = 11;
const int pin_laser = 12;
const int pin_eye_puff = 13;
const int ELEC_PIN = 34;
const int DAC_PIN = DAC0;

const int pin_pump = 22;
const int pin_LED1 = 2;
const int pin_LED2 = 3;
const int pin_camera2 = 25;
const int pin_go1 = 26;
const int pin_go2 = 27;
const int pin_lick = 28;

const int pin_odor1 = 30;
const int pin_odor2 = 31;
const int pin_odor3 = 32;
const int pin_odor4 = 33;
const int pin_fv = 36;

const int pin_solenoid = 37;

const int pin_encoder_pin1 = 2;
const int pin_encoder_pin2 = 3;
// Index into array indicates stimulus number as specified in Matlab, value at that index is corresponding pin number on Arduino
// Index zero should always have zero value because there is no stimulus 0
// Other zeros can be filled in with other values as needed
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

// Task variables (time in ms, freq in hz) - can be updated from Matlab
// All param_* variables must be 16-bit ints or else they will screw up Serial communication
//    and get mangled

float ITI = 5.0;               
float Interval1 = 1.0;          
float Interval2 = 1.0;          
float pre_timewindows = 0.2;   
float post_timewindows = 0.2;   
float rewardDuration = 1.0;     

int trialnums = 10;             
int currenttrialnum = 0;        
int correcttrials = 0;          
int FailTrialNum = 0;           
int lickcounts = 0;             
int misslick = 0;
int pump_dur = 0;              

const int MIN_BRIGHTNESS = 0;  
const int MAX_BRIGHTNESS = 255; 
int current_brightness = 255;
const int brightness_step = 1;  

unsigned long startTime = 0;    
float total_time = 0; 

float event1_time = 0;
float event2_time = 0;
float event3_time = 0;
float lick_time = 0;

unsigned long debounceWindow = 100;
unsigned long event1_lastTrigger = 0;
unsigned long event2_lastTrigger = 0;
unsigned long event3_lastTrigger = 0;


bool cs_executed = false;   // CS 
bool cs2_executed = false;  // CS2 
bool cs3_executed = false;  // CS3 

unsigned long cs_start_time = 0;    
unsigned long cs2_start_time = 0;   
unsigned long cs3_start_time = 0;

bool US_executed = false; // US solenoid
unsigned long US_start_time = 0;

int event1_pin = 0; 
int event2_pin = 0;
int event3_pin = 0;

int param_campretime = 200;
int param_camposttime = 800;

int param_csdur = 500;
int param_cs2dur = 500;
int param_cs3dur = 500;

int param_delay1 = 500;
int param_delay2 = 500;

//int param_evdelay = 500;

int param_ISI = 200;
int odor_ITI = 200;
int param_usdur = 20;

int param_csch = ch_brightled;   // default to LED
int param_cs2ch = ch_odor2;
int param_cs3ch = ch_odor3;

int param_usch = ch_puffer_eye;   // default to ipsi corneal puff
int param_stimch = ch_tone;  // default to tone


int param_tonefreq = 1000; // default to 1kHz
int param_toneamp = 2048; //max value for 4096 DAC

int param_csintensity = 64; // default to max intensity
int param_csrepeats = 1; //number of repetitions of tone for sequence training
int param_csperiod = 0; //period only necessary for repeating CSs

// For laser stim during trials, time values in ms
int param_stimdelay = 0; // delay from CS onset until laser onset

int param_laserperiod = 0; // period of laser pulse
int param_lasernumpulses = 1; // number of laser pulses in train
int param_rampoffdur = 0; //ALvaro 10/19/18
int param_lasergain = 1;
int param_laseroffset = 0;

int param_laserpower = 2048;

int param_stimdur = 500;
int param_stimperiod = 0;
int param_stimrepeats = 1;

float param_elecVoltage = 10;  
float param_laserVoltage = 5;

int param_elecRepeats = 1;
int param_laserRepeats = 1; 

int param_encoderperiod = 5; // in ms
int param_encodernumreadings = (param_campretime + param_camposttime) / param_encoderperiod; // number of readings to take during trial

// Codes for sending arrays to Matlab - consider enum type cast to byte
const byte ENCODER_L = 100;
const byte TIME_L = 101;
// For converting longs to bytes
const uint32_t bit_patterns[4] = { 0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000 };


unsigned long elecStartTime = 0;
unsigned long laserStartTime = 0;
unsigned long toneStartTime = 0;

bool RUNNING = false;
bool elec_running = false;
bool isToneOn = false;
bool training_active = false;
int reversePolarity = 0;

// Default constructors for StateMachine objects
// It's probably more flexible if we create an array of StateMachine objects that we can iterate through in main loop but for now this will work
//    and seems easier to comprehend
Stimulus camera(0, param_campretime + param_camposttime, digitalOn, digitalOff, pin_camera);
Stimulus US(param_campretime + param_ISI, param_usdur, digitalOn, digitalOff, stim2pinMapping[param_usch]);
// Stimulus US(param_campretime + param_ISI, param_usdur, digitalOn, digitalOff, pin_solenoid);

StimulusRepeating CS(param_campretime, param_csdur, digitalOn, digitalOff, stim2pinMapping[param_csch],0, 1);
StimulusRepeating CS2(param_campretime, param_cs2dur, digitalOn, digitalOff, stim2pinMapping[param_cs2ch],0,1);
StimulusRepeating CS3(param_campretime, param_cs3dur, digitalOn, digitalOff, stim2pinMapping[param_cs3ch], 0, 1);
StimulusRepeating stim(param_campretime + param_stimdelay, param_stimdur, digitalOn, digitalOff, stim2pinMapping[param_stimch], 0, 1);

SensorRepeating enc(0, takeEncoderReading, param_encoderperiod, param_encodernumreadings);

Encoder cylEnc(2, 3); // pins used should have interrupts, e.g. 2 and 3

uint16_t voltageToDataA(float voltage) {
  return (uint16_t)(voltage * 65535 / (2 * VREFA));
}

uint16_t voltageToDataB(float voltage) {
  return (uint16_t)((voltage + 10.0) * 65535 / (2 * VREFB));
}


void DAC8563_Write(uint8_t cmd, uint16_t data) {
  digitalWrite(CS_PIN, LOW);  

  SPI.transfer(cmd);   
  SPI.transfer(highByte(data));
  SPI.transfer(lowByte(data)); 
  digitalWrite(CS_PIN, HIGH);  
}


void DAC8563_Init() {
  pinMode(CS_PIN, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE2);  
  SPI.setClockDivider(SPI_CLOCK_DIV4); 

  DAC8563_Write(CMD_RESET_ALL_REG, 0x0001); 
  delay(10);
  
  DAC8563_Write(CMD_PWR_UP_A_B, 0x0003); 
  DAC8563_Write(CMD_INTERNAL_REF_EN, 0x0001);
  DAC8563_Write(CMD_GAIN, 0x0001);
}

void DAC_OutAB(uint16_t data_a, uint16_t data_b) {
  DAC8563_Write(CMD_SETA_UPDATEA, data_a);  
  DAC8563_Write(CMD_SETB_UPDATEB, data_b); 
}

// The setup routine runs once when you press reset or get reset from Serial port
void setup() {
  // Initialize the digital pin as an output.
  pinMode(pin_camera, OUTPUT);
  pinMode(pin_led, OUTPUT);
  pinMode(pin_eye_puff, OUTPUT);
  pinMode(pin_whisker, OUTPUT);
  pinMode(pin_tone, OUTPUT);
  pinMode(pin_brightled, OUTPUT);
  pinMode(pin_laser, OUTPUT);
  pinMode(pin_ss, OUTPUT);
  pinMode(ELEC_PIN, OUTPUT); 
  pinMode(CS_PIN, OUTPUT);
  pinMode(DAC_PIN, OUTPUT);

  pinMode(pin_fv, OUTPUT);
  pinMode(pin_odor1, OUTPUT);
  pinMode(pin_odor2, OUTPUT);
  pinMode(pin_odor3, OUTPUT);
  pinMode(pin_odor4, OUTPUT);

  pinMode(pin_solenoid, OUTPUT); 
  
  pinMode(pin_pump, OUTPUT);
  pinMode(pin_LED1, OUTPUT);
  pinMode(pin_LED2, OUTPUT);
  pinMode(pin_camera2, OUTPUT);

  pinMode(pin_go1, INPUT_PULLUP);
  pinMode(pin_go2, INPUT_PULLUP);
  pinMode(pin_lick, INPUT);

  // Default all output pins to LOW - for some reason they were floating high on the Due before I (Shane) added this
  digitalWrite(pin_camera, LOW);
  digitalWrite(pin_camera2, LOW);
  digitalWrite(pin_pump, LOW);
  
  analogWrite(pin_LED1, 0);
  analogWrite(pin_LED2, 0);

  digitalWrite(pin_led, LOW);
  digitalWrite(pin_eye_puff, LOW);
  digitalWrite(pin_whisker, LOW);
  digitalWrite(pin_tone, LOW);
  digitalWrite(pin_brightled, LOW);
  digitalWrite(pin_laser, LOW);
  digitalWrite(pin_laser, LOW);
  digitalWrite(ELEC_PIN, LOW);
  
  digitalWrite(pin_solenoid, LOW);

  digitalWrite(pin_fv, 0);
  digitalWrite(pin_odor1, 0);
  digitalWrite(pin_odor2, 0);
  digitalWrite(pin_odor3, 0);
  digitalWrite(pin_odor4, 0);

  DAC8563_Init();
  // set your ssPin to LOW too. when you have more external chips to control, you will have to be more careful about this step (ssPin LOW means the chip will respond to SPI commands)
  analogWriteResolution(12);
  //initTimer();

  DAC8563_Write(CMD_SETA_UPDATEA, voltageToDataA(0));
  DAC8563_Write(CMD_SETB_UPDATEB, voltageToDataB(0));

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.flush();
  Serial1.flush();

  Wire.begin();
  toneOff(0);
  DACWrite(0);
}

// The loop routine runs over and over again forever
// In this loop we have our StateMachines check their states and update as necessary
// The StateMachines handle their own timing
// It's critical that this loop runs fast (< 1 ms period) so don't put anything in here that takes time to execute
// if a trial is running (e.g. "blocking" serial port access should only happen when trial isn't RUNNING)

void loop() {

  delay(500);

  if (RUNNING) {
      unsigned long slot_start_us,slot_intermediate_us;
      int execute_us;
      unsigned long trial_start_ms,trial_current_ms;
      int execute_ms;
      bool end_flag = false;
      //Preprocess code begin
      enc_buf_reset();
      //Preprocess code end
      trial_start_ms = millis();
      while(!end_flag){//a trial is divided into 1ms slots
        slot_start_us = micros();
        trial_current_ms = millis();
        execute_ms = trial_current_ms - trial_start_ms;

        end_flag=trial_output_monit(execute_ms);
        trial_input_read(execute_ms);
        
        slot_intermediate_us = micros();
        
        execute_us =  slot_intermediate_us - slot_start_us;
        if(execute_us<1000){
          delayMicroseconds(1000-execute_us);
        }
        else{
          RAISE_ERROR(__LINE__);//BIG_PROBLEM:time to ececute functions longer than 1ms
        }
      }
      RUNNING = false;
      RAISE_HINT(__LINE__);
  }

  else {
      checkVars();
      if (Serial.available() > 0) {

          int command = Serial.read(); 

          Serial1.print("get command:");
          Serial1.println(command);

          switch(command) {
              case 1:
                  startTrial();
                  break;
              case 2: 
                  sendEncoderData();
                  enc.reset();
                  break;
              case 3:
                  elecOn(0);
                  break;
              case 4:
                  elecOff(0);
                  break;
              case 5: 
                  laserOn(0);
                  break;
              case 6:
                  laserOff(0);
                  break;
               case 7:
                  toneOn(DAC_PIN);
                  break;
                case 8:
                  toneOff(DAC_PIN);
                  break;
                case 9:
                  start_training();
                  break;

              default:
                  break;
          }
      }

  }

}

// Check to see if Matlab is trying to send updated variables
// (should we send specific code to indicate that we are sending variables?)
void checkVars() {
  int header;
  int value;
  
  // Matlab sends data in 3 byte packets: first byte is header telling which variable to update,
  // next two bytes are the new variable data as 16 bit int (can only send 16 bit ints for now)
  // Header is coded numerically (0, 1, and 2 are reserved for special functions so don't use them to code variable identities)
  while (Serial.available() > 0) {

      if (Serial.available() < 2) {
      return;
    }

    header = Serial.read();
    value = Serial.read() | Serial.read() << 8;

    if (header == 0) {
      flushReceiveBuffer();     // A way to bail out and start over if the Arduino stops responding due to problem parsing Serial inputs
      break;
    }

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
        param_tonefreq = value;
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
        param_laserpower = value;
        break;
      case 45:
        //param_csintensity = value; Alvaro 05/09/19 from sheiney committed on Jan 18, 2017
        // setDiPoValue(param_csintensity);
        // The Matlab code stores intensity values up to 256 because it's nicer to deal with   Alvaro 05/09/19 sheiney committed on Jan 18, 2017
        // multiples of 2 but we can only pass at most 255 so we have to correct that here.  Alvaro 05/09/19 sheiney committed on Jan 18, 2017
        // Zero is a special case because when the user specifies 0 they want it to mean "off"  Alvaro 05/09/19 sheiney committed on Jan 18, 2017
        param_csintensity = value==0 ? value : value-1; //Alvaro
        break;
      case 15:
        param_laserperiod = value;
        break;
      case 16:
        param_lasernumpulses = value;
        break;
      // case 20:
      //   param_csperiod = value;
      //   break;
      // case 21:
      //   param_csrepeats = value;
      //   break;
      case 22:
        param_rampoffdur = value; //ALvaro 10/19/18
        break;
      case 24:
        param_elecPeriod = value;
        break;
      case 25:
        param_elecVoltage = value;
        break;
      case 26:
        param_elecRepeats = value;
        break;
      case 27:
        param_laserVoltage = value;
      case 28:
        param_laserPeriod = value;
        break;
      case 29: 
        param_laserRepeats = value;
        break;
      case 30: 
        param_toneamp = value;
        break; 
      case 31: 
        param_elecFrequency = value;
        break; 
      case 32: 
        param_laserFrequency = value;
        break;
      case 33: 
        param_stimch = value;
        break;
      case 34:
        reversePolarity = value;
        // Serial1.println("update reverse polarity:");
        // Serial1.println(reversePolarity);
        break;
      case 35:
        event1_pin= value;
        break;
      case 36:
        event2_pin= value;
        break;
      case 37:
        event3_pin= value;
        break;
      case 38:
        trialnums = value;
        break;
      case 39:
        Interval1 = value;
        break;
      case 40:
        Interval2 = value;
        break;
      case 41:
        ITI = value;
        break;
      case 42:
        pre_timewindows = value;
        break;
      case 43:
        post_timewindows = value;
        break;
      case 44:
        rewardDuration = value;
        break;
      case 46:
        current_brightness = value;
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
         pump_dur = value;
        break;
    }
    // We might be able to remove this delay if Matlab sends the parameters fast enough to buffer
    delay(10); // Delay enough to allow next 3 bytes into buffer (24 bits/115200 bps = ~200 us, so delay 1 ms to be safe).
  } 

}

// Update the instantiated StateMachines here with any new values that have been sent from Matlab
void configureTrial() {

    camera.setDuration(param_campretime + param_camposttime);

    US.setDelay(param_campretime + param_ISI);
    US.setDuration(param_usdur);
    US.setFunctionArg(stim2pinMapping[param_usch]);

    stim.setDelay(param_campretime + param_stimdelay);
    stim.setDuration(param_stimdur);
    stim.setFunctionArg(stim2pinMapping[param_stimch]);


    CS.setDelay(param_campretime);
    CS.setDuration(param_csdur);
    CS.setFunctionArg(stim2pinMapping[param_csch]);

    CS2.setDelay(param_campretime);
    CS2.setDuration(param_cs2dur);
    CS2.setFunctionArg(stim2pinMapping[param_cs2ch]);

    CS3.setDelay(param_campretime);
    CS3.setDuration(param_cs3dur);
    CS3.setFunctionArg(stim2pinMapping[param_cs3ch]);

   // Do some error checking for required bounds //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
   // CS intensity can be at most 255 (=2^8-1) because PWM with analogWrite uses 8 bit value //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
   // CS intensity for tone can have at most a value of 127 because the digital potentiometer is 7 bits //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
   // but we don't have to worry about it because most significant bit will be truncated so 255 will look like 127 //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
   if (param_csintensity < 0) { param_csintensity = 0;} //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
   if (param_csintensity > 255) { param_csintensity = 255;} //Alvaro 05/09/19 sheiney committed on Jan 18, 2017

   if (param_csch == ch_tone) { //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
     setDiPoValue(param_csintensity); //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
   } //Alvaro 05/09/19 sheiney committed on Jan 18, 2017

}

// Called by main loop when Arduino receives trigger from Matlab
void startTrial() {

    configureTrial();

    RUNNING = true;

    // Once StateMachines have been started the delay clock is ticking so don't put anything else below the call to start()
    // We want to return to the main loop ASAP after StateMachines have started
    // Each start() method only contains one function call to get current time and two assignment operations so should return quickly
    // The duration of the trial is determined by the camera parameters (delay, duration) -- all timing is relative to it
    camera.start();

    enc.start();

    stim.setDelay(param_campretime + param_stimdelay);
    stim.setDuration(param_stimdur);
    stim.setFunctionArg(stim2pinMapping[param_stimch]);

    if (param_stimdur > 0) { stim.start(); }
    if (param_usdur > 0) { US.start(); }
    
    // duration of zero means it's not supposed to run on this trial so don't bother to start it
    if (param_csdur > 0) 
    { 
      CS.start();   
      //Serial1.println("CS1 started with duration: " + String(param_csdur));
      }
}

// Called by main loop when camera stops
void endOfTrial() {

    RUNNING = false;

    // These should already be stopped if we timed things well but we'll do it again just to be safe
    CS.stop();

    US.stop();
    enc.stop();
    camera.stop(); // Should already be stopped if this function was called

}

// Make sure this code executes fast (< 1 ms) so it doesn't screw up the timing for everything else
void DACWrite(int DACvalue) {

    Wire.beginTransmission(MCP4725_ADDR);
    Wire.write(64);                     // cmd to update the DAC
    Wire.write(DACvalue >> 4);        // the 8 most significant bits...
    Wire.write((DACvalue & 15) << 4); // the 4 least significant bits...
    Wire.endTransmission();

}

int powerToDACUnits(int power) {

    int DACUnits = power * param_lasergain + param_laseroffset;

    if (DACUnits < MAXDACUNIT) {return DACUnits;}
    else {return MAXDACUNIT;}

}

// for working with the MCP4131 digital potentiometer
void setDiPoValue(int value)
{
    SPI.transfer(0);
    SPI.transfer(value);
}

// Tone is a special case of digitalWrite because it uses a timer to cycle at requested frequency
// We also have a special case if CS is LED to use CS intensity to regulate brightness
void digitalOn(int pin) {
    // if (pin == pin_tone) { //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
    switch (pin) { //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
     case pin_led: //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
       analogWrite(pin, param_csintensity); //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
       break; //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
      case pin_brightled: //Alvaro 05/09/19
      analogWrite(pin, param_csintensity); //Alvaro 05/09/19
      break;
      case DAC_PIN: 
        toneOn(pin);
        break;
      case pin_laser: 
        laserOn(pin);
        break;
      case ELEC_PIN: 
        elecOn(pin);
        break;
      // case pin_odor1:
      //   odor1On(pin);
      //   break;
      // case pin_odor2:
      //   odor2On(pin);
      //   break;
      // case pin_odor3:
      //   odor3On(pin);
      //   break;
      // case pin_odor4:
      //   odor4On(pin);
      //   break;
     default: //Alvaro 05/09/19 sheiney committed on Jan 18, 2017
        digitalWrite(pin, HIGH);
    }
}

void digitalOff(int pin) {
    switch (pin) {
      case DAC_PIN: 
        toneOff(pin);
        break;
      case pin_laser: 
        laserOff(pin);
        break;
      case ELEC_PIN: 
        elecOff(pin);
        break;
      // case pin_odor1:
      //   odor1Off(pin);
      //   break;
      // case pin_odor2:
      //   odor2Off(pin);
      //   break;
      // case pin_odor3:
      //   odor3Off(pin);
      //   break;
      // case pin_odor4:
      //   odor4Off(pin);
      //   break;
     default: 
        digitalWrite(pin, LOW);
    }
}

void toneOn(int pin) {
  isToneOn = true;

  toneStartTime = millis(); 

  pmc_set_writeprotect(false);             
  pmc_enable_periph_clk(ID_TC3);

  TC_Configure(TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1); 

  float rc = static_cast<float>(VARIANT_MCK) / 4.0 / param_tonefreq;
  TC_SetRA(TC1, 0, static_cast<int>(rc / 2)); 
  TC_SetRC(TC1, 0, static_cast<int>(rc));

  TC_Start(TC1, 0);                                              

  TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;   
  TC1->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;  
  NVIC_EnableIRQ(TC3_IRQn);                 
}

void toneOff(int pin) {
    isToneOn = false;

    TC_Stop(TC1, 0);
    NVIC_DisableIRQ(TC3_IRQn);

    analogWrite(DAC_PIN, 0);
}

void laserOn(int dummy) { // Function signature requires int but we don't need it so call it "dummy"
    uint16_t dac_a_value;
    laserStartTime = millis();
    unsigned long lastUpdateTime = laserStartTime;
    unsigned long currentTime = laserStartTime;

    while (currentTime - laserStartTime < param_stimdur) {
      currentTime = millis();
      if (currentTime - lastUpdateTime >= (1000/param_laserFrequency)) {
        lastUpdateTime += (1000/param_laserFrequency);
        dac_a_value = voltageToDataA(param_laserVoltage / 10);
        DAC8563_Write(CMD_SETA_UPDATEA, dac_a_value);
        delayMicroseconds(param_laserPeriod);
        DAC8563_Write(CMD_SETA_UPDATEA, voltageToDataA(0.0)); 
      }
    }
    laserOff(0);
  }

void laserOff(int dummy) { // Function signature requires int but we don't need it so call it "dummy"
  uint16_t dac_a_value = voltageToDataA(0.0); 
  DAC8563_Write(CMD_SETA_UPDATEA, dac_a_value);
  }

void elecOn(int dummy) {
    elec_running = true;
    uint16_t dac_b_value;
    elecStartTime = millis();
    unsigned long lastUpdateTime = elecStartTime;
    unsigned long currentTime = elecStartTime;

    while (currentTime - elecStartTime < param_stimdur) 
    {
      currentTime = millis();
      if (currentTime - lastUpdateTime >= (1000/param_elecFrequency))
      {
        lastUpdateTime += (1000/param_elecFrequency);
        dac_b_value = voltageToDataB(param_elecVoltage/10);
        if (reversePolarity == 0) {
        DAC8563_Write(CMD_SETB_UPDATEB, voltageToDataB(-(param_elecVoltage/10)));
        delayMicroseconds(param_elecPeriod);
        DAC8563_Write(CMD_SETB_UPDATEB, dac_b_value);
        delayMicroseconds(param_elecPeriod);
        DAC8563_Write(CMD_SETB_UPDATEB, voltageToDataB(0));
        } 
        else if (reversePolarity == 1) {
          DAC8563_Write(CMD_SETB_UPDATEB, dac_b_value);
          delayMicroseconds(param_elecPeriod);
          DAC8563_Write(CMD_SETB_UPDATEB, voltageToDataB(-(param_elecVoltage/10)));
          delayMicroseconds(param_elecPeriod);
          DAC8563_Write(CMD_SETB_UPDATEB, voltageToDataB(0));
        }
      }
    }
    elecOff(0);
}

void elecOff(int dummy) {
  uint16_t dac_b_value = voltageToDataB(0.0);  
  DAC8563_Write(CMD_SETB_UPDATEB, dac_b_value);
  elec_running = false;
}

void odor1On(int channel, int duration) {
   //Serial1.println("enter odor1on\n");
 if (channel == ch_odor1) {
    digitalWrite(pin_fv, 255);
    digitalWrite(pin_odor1, 255);
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
      
    }
    odor1Off(channel);
  }
  //Serial1.println("odor1On complete\n");
}

void odor1Off(int channel) {
  // Serial1.println("enter odor1off\n");
  
  if (channel == ch_odor1) {
    digitalWrite(pin_fv, 0);
    digitalWrite(pin_odor1, 0);
  }
  // Serial1.println("odor1Off complete\n");
}

void odor2On(int channel, int duration) {
  
 if (channel == ch_odor2) {
    digitalWrite(pin_fv,255);
    digitalWrite(pin_odor2, 255);
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
      
    }
    odor2Off(channel);
  }
  
}

void odor2Off(int channel) {
  
  if (channel == ch_odor2) {
    digitalWrite(pin_fv, 0);
    digitalWrite(pin_odor2, 0);
  }
  
}

void odor3On(int channel, int duration) {
  
  if (channel == ch_odor3) {
    digitalWrite(pin_fv, 255);
    digitalWrite(pin_odor3, 255);
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
      
    }
    odor3Off(channel);
  }
  
}

void odor3Off(int channel) {
  
   if (channel == ch_odor3) {
    digitalWrite(pin_fv, 0);
    digitalWrite(pin_odor3, 0);
  }
  
} 

void odor4On(int channel, int duration) {
  
  if (channel == ch_odor4) {
    digitalWrite(pin_fv,255);
    digitalWrite(pin_odor4, 255);
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
      
    }
    odor4Off(channel);
  }
  
}

void odor4Off(int channel) {
  
  if (channel == ch_odor4) {
    digitalWrite(pin_fv, 0);
    digitalWrite(pin_odor4, 0);
  }
  
} 

void solenoidOn(int channel, int duration) {

  if (channel == ch_solenoid) {
    digitalWrite(pin_solenoid, HIGH);
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
    }
    solenoidOff(channel);
  }
}

void solenoidOff(int channel) {
  
  if (channel == ch_solenoid) {
    digitalWrite(pin_solenoid, LOW);
  }
}

void puffereyeOn(int channel, int duration){
  if (channel == ch_puffer_eye) {
    digitalWrite(pin_eye_puff, HIGH);
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {

    }
    puffereyeOff(channel);
  }
}

void puffereyeOff(int channel) {
  if (channel == ch_puffer_eye) {
    digitalWrite(pin_eye_puff, LOW);
  }
}

void pufferotherOn(int channel, int duration){
  if (channel == ch_puffer_other) {
    digitalWrite(pin_whisker, HIGH);
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {

    }
    pufferotherOff(channel);
  }
}

void pufferotherOff(int channel) {
  if (channel == ch_puffer_other) {
    digitalWrite(pin_whisker, LOW);
  }
}

// We call by reference so we can update the local variables in "reading_function" of StateMachine object
void takeEncoderReading(timems_t &time, int32_t &reading) {

    time = millis();
     reading = cylEnc.read();
    // reading = 5000-random(10000);  // for testing

}

void sendEncoderData() {

    // Consider using Serial.availableForWrite() if the code below is blocking

    Serial.write(ENCODER_L);
    // Maybe also send number of values so Matlab knows how many to expect (will have to be 2 bytes though)?
    for (int i=0; i<param_encodernumreadings; i++) {//FIXME:change param_encodernumreadings to SENSOR_BUF_SENDLEN
        writeByte(enc_readings[i]);
    }

    Serial.write(TIME_L);
    // Maybe also send number of values so Matlab knows how many to expect (will have to be 2 bytes though)?
    for (int i=0; i<param_encodernumreadings; i++) {//FIXME:change param_encodernumreadings to SENSOR_BUF_SENDLEN
        writeTwoByte(enc_times[i]);
    }
    RAISE_HINT(__LINE__);
}

// We have to send bytes over the serial port, so break the 32-bit integer into 4 bytes by ANDing only the byte we want
// and shifting that byte into the first 8 bits
// Unsigned longs
void writeByte(uint8_t val){
  Serial.write(val);
}
void writeTwoByte(uint16_t val){
  byte low_Byte   = val & 0xFF;
  byte high_Byte  = val >> 8; 

  Serial.write(low_Byte);//send low_byte first
  Serial.write(high_Byte);

}
void writeLong(uint32_t long_value) {
    for (int i=0; i<4; i++) {
        // Can we do this instead: (byte)(long_value >> 24) [replacing 24 with appropriate shift]?
        // Cast to byte will truncate to first 8 bits as side effect
        byte val = ( long_value & bit_patterns[i] ) >> 8*i;
        Serial.write(val);
    }
}

// Overloaded for signed longs
void writeLong(int32_t long_value) {
    for (int i=0; i<4; i++) {
        byte val = ( long_value & bit_patterns[i] ) >> 8*i;
        Serial.write(val);
    }
}

void flushReceiveBuffer() {
    while(Serial.available()) {
        Serial.read();
    }
    Serial.flush();
}


void TC3_Handler() {
  Tc *tc = TC1;  
  int channel = 0;

  if (tc->TC_CHANNEL[channel].TC_SR & TC_SR_CPCS) {
    if (isToneOn) {
      static bool state = false;
      analogWrite(DAC_PIN, state ? param_toneamp : 0);  
      state = !state;
       if (millis() - toneStartTime >= param_stimdur) 
       {
          toneOff(DAC_PIN);
       }
    } 
  }
}


void executeCS(int channel, int duration) {
  switch (channel) {
    case ch_odor1:
      odor1On(channel, duration);
      break;
    case ch_odor2:
      odor2On(channel, duration);
      break;
    case ch_odor3:
      odor3On(channel, duration);
      break;
    case ch_odor4:
      odor4On(channel, duration);
      break;
    default:
      break;
  }
}

void executeUS(int channel, int duration) {
    switch (channel){
      case ch_solenoid:
        solenoidOn(channel, duration);
        break;
      case ch_puffer_eye:
        puffereyeOn(channel, duration);
        break;
      case ch_puffer_other:
        pufferotherOn(channel, duration);
        break;
    }
}

void sendDataToMATLAB() {
    Serial.print(currenttrialnum);
    Serial.print(",");
    Serial.print(correcttrials);
    Serial.print(",");
    Serial.print(FailTrialNum);
    Serial.print(",");
    Serial.print(lickcounts);
    Serial.print(",");
    Serial.print(misslick);
    Serial.print(",");
    Serial.print(event1_time);
    Serial.print(",");
    Serial.print(event2_time);
    Serial.print(",");
    Serial.print(event3_time);
    Serial.print(",");
    Serial.println(lick_time);
}

void setLEDBrightness(int pin, int brightness) {
    if (brightness > 0) {
        brightness = map(brightness, 0, 255, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    } else{
        brightness = 0;
    }
    analogWrite(pin, brightness);
}

void start_training() {
  training_active = true;
  currenttrialnum = 0;
  correcttrials = 0;
  FailTrialNum = 0;
  lickcounts = 0;
  misslick = 0;
  


  event1_pin = (event1_pin == 1) ? pin_go1 : (event1_pin == 2) ? pin_go2 : 0;
  event2_pin = (event2_pin == 1) ? pin_go1 : (event2_pin == 2) ? pin_go2 : 0;
  event3_pin = (event3_pin == 1) ? pin_go1 : (event3_pin == 2) ? pin_go2 : 0;

  Serial1.println("Starting training...");
  startTime = millis();

  while (currenttrialnum < trialnums && training_active) {
    currenttrialnum++;
    event1_time = 0;
    event2_time = 0;
    event3_time = 0;
    lick_time = 0;
    setLEDBrightness(pin_LED1, 0);
    setLEDBrightness(pin_LED2, 0);
    bool trial_success = true;


    if (event1_pin != 0) {
      int event1_led = (event1_pin == pin_go1) ? pin_LED1 : pin_LED2;
      setLEDBrightness(event1_led, current_brightness);
      Serial1.println("Event 1 LED ON");
      Serial1.println("Waiting for Event 1...");

      unsigned long event1_start = millis();
      bool event1_triggered = false;

      while (!event1_triggered) {
        if (digitalRead(event1_pin) == LOW && millis() - event1_lastTrigger > debounceWindow) {
          event1_time = (millis() - startTime) / 1000;  
          event1_triggered = true;
          setLEDBrightness(event1_led, 0);
          Serial1.println("Event 1 triggered at: " + String(event1_time) + "s");
          break;
        }
      }
    }


    bool event2_triggered = false;
    if (event2_pin != 0) {
      int event2_led = (event2_pin == pin_go1) ? pin_LED1 : pin_LED2;
      Serial1.println("Waiting for Event 2...");
      unsigned long event2_start = millis();

      
      while (millis() - event2_start < Interval1 - pre_timewindows) {}
      setLEDBrightness(event2_led, current_brightness);
      Serial1.println("Event 2 LED ON");

      while (millis() - event2_start < (Interval1 + post_timewindows)) {
        if (digitalRead(event2_pin) == LOW && millis() - event2_lastTrigger > debounceWindow) {
          event2_time = (millis() - startTime) /1000;  
          event2_triggered = true;
          setLEDBrightness(event2_led, 0);
          Serial1.println("Event 2 triggered at: " + String(event2_time) + "s");
          break;
        }
      }

      if (!event2_triggered) {
        Serial1.println("Event 2 timeout.");
        setLEDBrightness(event2_led, 0);
        trial_success = false;
        FailTrialNum++;
      }
    }


    bool event3_triggered = false;
    if (event3_pin != 0 && event2_triggered){
      int event3_led = (event3_pin == pin_go1) ? pin_LED1 : pin_LED2;
      Serial1.println("Waiting for Event 3...");
      unsigned long event3_start = millis();


      while (millis() - event3_start < Interval2 - pre_timewindows) {}
      setLEDBrightness(event3_led, current_brightness);
      Serial.println("Event 3 LED ON");

      while (millis() - event3_start < (Interval2 + post_timewindows)) {
        if (digitalRead(event3_pin) == LOW && millis() - event3_lastTrigger > debounceWindow) {
          event3_time = (millis() - startTime) /1000;
          event3_triggered = true;
          setLEDBrightness(event3_led, 0);
          Serial1.println("Event 3 triggered at: " + String(event3_time) + "s");
          break;
        }
      }

      if (!event3_triggered) {
        Serial1.println("Event 3 timeout.");
        setLEDBrightness(event3_led, 0);
        FailTrialNum++;
        trial_success = false;
      }
    }

      if (trial_success && event2_triggered && event3_triggered) {
          digitalWrite(pin_pump, HIGH);
          delay(pump_dur);
          digitalWrite(pin_pump, LOW);

    unsigned long reward_start = millis();
    bool licked = false;
    lick_time = 0;  

    while (millis() - reward_start < (rewardDuration * 1000)) {
      if (digitalRead(pin_lick) == HIGH) {
        lickcounts++;
        lick_time = (millis() - startTime) /1000; 
        
        Serial1.println("Lick detected at: " + String(lick_time) + "s");
        break;
      }
       delay(1); 
    }
  
    if (!licked) {
      misslick++;
      Serial1.println("Missed lick.");
    }
    correcttrials++;
  }
     setLEDBrightness(pin_LED1, 0);
     setLEDBrightness(pin_LED2, 0);

    sendDataToMATLAB();
    
    delay(ITI * 1000);
  }

  training_active = false;
  total_time = (millis() - startTime) / 1000.0;
  Serial1.print("Total time: ");
  Serial1.println(total_time, 2);
}

//Serial 1 print err/warn/hint info for debug
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
  Serial1.print("[");
  Serial1.print(line_number);
  Serial1.println("]");
}

void pin_driver(int pin_number,bool on){//to make every pin with unified interface
  if(pin_number==pin_camera){
    if(on){
      digitalOn(pin_number);
    }
    else{
      digitalOff(pin_number);
    }
  }
  else{
    RAISE_WARNING(__LINE__);//the pin_number 's action has not specified yet,we need to add this case    
  }
}
void channel_driver(int channel_number,bool on){//to make every channel with unified interface
  int target_pin = stim2pinMapping[channel_number];
  if(channel_number == param_csch){
    //======CS=========
    if(channel_number==ch_brightled){//CS:ch_brightled
      if(on){
        digitalOn(target_pin);
      }
      else{
        digitalOff(target_pin);
      }
    }
    else{
      if(on){
        digitalWrite(pin_fv, HIGH);
        digitalWrite(target_pin,HIGH);
      }
      else{
        digitalWrite(pin_fv, LOW);
        digitalWrite(target_pin,LOW);
      }
    }
  }
  else if(channel_number == param_cs2ch || channel_number == param_cs3ch){
    //====CS2,3==========
    if(on){
      digitalWrite(pin_fv, HIGH);
      digitalWrite(target_pin,HIGH);
    }
    else{
      digitalWrite(pin_fv, LOW);
      digitalWrite(target_pin,LOW);
    }
  }
  else if(channel_number == param_stimch || channel_number == param_usch){
    //======stim,US=======
    //US:ch_solenoid,ch_puffer_eye,ch_puffer_other
    if(on){
      digitalWrite(target_pin,HIGH);
    }
    else{
      digitalWrite(target_pin,LOW);
    }
  }
  else {
    RAISE_WARNING(__LINE__);//the channel_number 's action has not specified yet,we need to add this case
  }
}

//trial main logic:return true means it's over
bool digital_auto_action(int delay_ms,int duration_ms,Pin_Chan_Operation IOdriver,int PinorChan_number,int current_ms){
  if(duration_ms>0){
    if(current_ms<delay_ms){
      IOdriver(PinorChan_number,/*false means pin off*/false);
      return false;
    }
    else if(current_ms<delay_ms+duration_ms){
      IOdriver(PinorChan_number,/*true means pin on*/true);
      return false;
    }
    else{
      IOdriver(PinorChan_number,/*false means pin off*/false);
      return true;
    }
  }
  else{//which means this pin or channel is unused
    return true;
  }
};

bool trial_output_monit(int execute_ms){
  bool exit_condition;
  bool camera_status;
  bool stim_status;
  bool cs_status;
  bool cs2_status;
  bool cs3_status;
  bool us_status;  
  //---------------------------------|delay                                 |duration                             |IOdriver         |Pin_Chan_number----------------//
  camera_status = digital_auto_action(0                                     ,param_campretime+param_camposttime   ,pin_driver       ,pin_camera       ,execute_ms );//
  stim_status   = digital_auto_action(param_campretime+param_stimdelay      ,param_stimdur                        ,channel_driver   ,param_stimch     ,execute_ms );//
  cs_status     = digital_auto_action(param_campretime                      ,param_csdur                          ,channel_driver   ,param_csch       ,execute_ms );//
  cs2_status    = digital_auto_action(param_campretime+param_delay1         ,param_cs2dur                         ,channel_driver   ,param_cs2ch      ,execute_ms );//
  cs3_status    = digital_auto_action(param_campretime+param_delay2         ,param_cs3dur                         ,channel_driver   ,param_cs3ch      ,execute_ms );//
  us_status     = digital_auto_action(param_campretime+param_ISI            ,param_usdur                          ,channel_driver   ,param_usch       ,execute_ms );//
  
  exit_condition = camera_status && stim_status && cs_status && cs2_status && cs3_status && us_status;
  return exit_condition;
}

void enc_buf_reset(void){
  //make all data tobe ff to inform that the data is valid,add to the main loop,before a trial //FIXME
  pinMode(pin_encoder_pin1, INPUT_PULLUP);
  pinMode(pin_encoder_pin1, INPUT_PULLUP);
  enc_data_index = 0;
  for(int i=0;i<SENSOR_BUF_MAXLEN;i++){//initialize data
    enc_readings[i] = 0xFF;
  }
}

void trial_input_read(int execute_ms){
  enc_readings[enc_data_index] = digitalRead(pin_encoder_pin2)*2 + digitalRead(pin_encoder_pin1);
  /*encode pattern 
  value encoded  |enc_pin1    |  enc_pin2 | Note
  FF             |   ---      |    ---    | *The value is invalid(surpass the trial)
  0              |    0       |     0     |
  1              |    1       |     0     |
  2              |    0       |     1     |
  3              |    1       |     1     |
  */
  enc_times[enc_data_index] = (uint16_t)execute_ms;
  enc_data_index++ ;
}