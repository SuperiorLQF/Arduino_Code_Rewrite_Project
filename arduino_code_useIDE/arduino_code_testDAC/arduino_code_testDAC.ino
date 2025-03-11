#include <SPI.h>
#define DAC8563_LDAC_pin 11 //always HIGH
#define DAC8563_SYNC_pin 12 //active LOW
#define DAC8563_CLR_pin  13 //active LOW
void DAC8563_Output(int chan_num,uint16_t data){
  float real_voltage;
  real_voltage = (data-32768)/3276.8;
  if(chan_num==0){
    writeDAC(0x18,data);
    Serial.print("DAC8563 PORT_A:");
    Serial.println(real_voltage);
  }
  else if(chan_num==1){
    writeDAC(0x19,data);
    Serial.print("DAC8563 PORT_B:");
    Serial.println(real_voltage);
  }
  else{
    ;
  }
}
void writeDAC(uint8_t cmd,uint16_t data){
  digitalWrite(DAC8563_SYNC_pin,0);
  SPI.transfer(cmd);
  SPI.transfer((data >> 8) & 0xFF);//high byte of data
  SPI.transfer(data & 0xFF);  //low byte of data    
  digitalWrite(DAC8563_SYNC_pin,1);  
}
void dac8563_init(void){
  digitalWrite(DAC8563_CLR_pin ,0);//hardware reset
  delay(1);
  digitalWrite(DAC8563_CLR_pin ,1);
  writeDAC(0x28,0x0001);//soft reset
  delay(1);
  writeDAC(0x20,0x0003);//power on
  delay(1);
  writeDAC(0x38,0x0001);//inner ref enable
  delay(1);
  writeDAC(0x02,0x0000);//set AB gain = 2
  delay(1);
  Serial.println("DAC8563 init OK");
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial){;}
  pinMode(DAC8563_LDAC_pin,OUTPUT);
  pinMode(DAC8563_SYNC_pin,OUTPUT);
  pinMode(DAC8563_CLR_pin ,OUTPUT);
  digitalWrite(DAC8563_LDAC_pin,1);
  digitalWrite(DAC8563_SYNC_pin,1);
  digitalWrite(DAC8563_CLR_pin ,1);
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000,MSBFIRST,SPI_MODE1));//10Mhz max:50M
  dac8563_init();
  DAC8563_Output(1,40000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
