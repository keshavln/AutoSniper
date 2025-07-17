#include <ESP32Servo.h>
Servo pan;
Servo tilt;
Servo trigger;
int pcurrent = 90;
int tcurrent = 60;
 void setup() {
  Serial.begin(115200);
  pan.attach(15);
  pan.write(pcurrent);
  tilt.attach(19);
  tilt.write(tcurrent);
  trigger.attach(18);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "LEFT" && pcurrent < 180) {
      pan.write(pcurrent+1);
      pcurrent++;
    } else if (command == "RIGHT" && pcurrent > 0) {
      pan.write(pcurrent - 1);
      pcurrent--;
    } else if (command == "UP" && tcurrent > 0) {
      tilt.write(tcurrent - 1);
      tcurrent--;
    } else if (command == "DOWN" && tcurrent < 90) {
      tilt.write(tcurrent + 1);
      tcurrent++;
    } else if (command == "SHOOT") {
        trigger.write(60);
        delay(300);
        trigger.write(100);
        delay(300);
        trigger.write(90);
        delay(500);
    }
    
  }
}
