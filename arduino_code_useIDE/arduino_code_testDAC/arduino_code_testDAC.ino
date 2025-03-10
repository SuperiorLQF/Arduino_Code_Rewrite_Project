#include <DueTimer.h>
int counter=50;
bool state = true;
void Handler(){
  state = !state;

  if(counter>0){
    digitalWrite(8,state);
    counter --;
  }
  else {
    digitalWrite(8,1);
    Timer0.stop();
  }

}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial){;}
  delay(3000);
  pinMode(8,OUTPUT);
  digitalWrite(8,state);
  // Serial.print("OK");
  // delay(1);//wait for print over
  Timer0.attachInterrupt(Handler);
  Timer0.start(50);//calls every 1_000_000ms

}

void loop() {
  // put your main code here, to run repeatedly:

}
