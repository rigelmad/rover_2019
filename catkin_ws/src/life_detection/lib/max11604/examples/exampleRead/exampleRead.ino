#include <max11604.h>

Max11604 adc = Max11604(10);
int results[10];

void setup() {
  // put your setup code here, to run once:
  while(adc.begin()){
    Serial.println("check connection");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(adc.readSingleChannel(0));
  adc.readAllChannels(results);
  for(int i = 0; i < 10; i ++) {
    Serial.print(results[i]);
    Serial.print(" | ");
   }
   Serial.println("");
   delay(50);
}
