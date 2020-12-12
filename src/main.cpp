#include <Arduino.h>
#include "Behaviors.h"
#include "Speed_controller.h"

Behaviors parkour;
SpeedController robot;

void setup() {
  parkour.Init();
}

void loop() {
  parkour.Run();
  //robot.WallFollow(12);
  
}