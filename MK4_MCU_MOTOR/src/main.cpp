#include <Arduino.h>
#include <vector>
#include <RoboClaw.h>
#include <main_function.h>

RoboClaw roboclaw(&Serial2, 10000);
int id_controler_A         = 0x80;
Motor m1(1, 1.32, 0.18, 0.00, 15000, id_controler_A);
Motor m4(4, 1.32, 0.18, 0.00, 15000, id_controler_A);
std::vector<double> vect_motor_cmd;

// hw_timer_t * timer = NULL;
// void IRAM_ATTR onTimer(){
//   ping_inboard_led(500);
//   vect_motor_cmd.clear();
//   vect_motor_cmd.push_back(serial_msg_vect[2].toFloat());
//   vect_motor_cmd.push_back(serial_msg_vect[3].toFloat());

//   send_cmd_motor_to_roboclaw(&roboclaw, id_controler_A, vect_motor_cmd);
//   timerAlarmDisable(timer);
// }

void setup() {
  roboclaw.begin(115200);
  roboclaw.flush();
  pinMode(ONBOARD_LED, OUTPUT);

  roboclaw.ForwardM1(id_controler_A, 0);
  roboclaw.ForwardM2(id_controler_A, 0);
  roboclaw.ResetEncoders(id_controler_A);
  roboclaw.SetM1VelocityPID(id_controler_A, m1.kp, m1.ki, m1.kd, m1.QPPS);
  roboclaw.SetM2VelocityPID(id_controler_A, m4.kp, m4.ki, m4.kd, m4.QPPS);


  ping_inboard_led(150);
  delay(500);
  ping_inboard_led(150);
  delay(500);

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(10);

  // timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(timer, &onTimer, true);
  // timerAlarmWrite(timer, 1000000, true);
}


std::vector<String> serial_msg_vect;
int flag;

int emergency_stop = 0;

void loop() 
{
  if(Serial.available() > 0) 
  {
    flag = read_serial_msg(&Serial, serial_msg_vect);
    if(flag == 0)
    {
      ping_inboard_led(10);
      emergency_stop = 0;

      vect_motor_cmd.clear();
      vect_motor_cmd.push_back(serial_msg_vect[2].toFloat());
      vect_motor_cmd.push_back(serial_msg_vect[3].toFloat());

      send_cmd_motor_to_roboclaw(&roboclaw, id_controler_A, vect_motor_cmd);

      // timerAlarmEnable(timer);
    }
  }
  else
  {
    if(emergency_stop > 20) 
    {
      // ~200ms
      emergency_stop = 0;
      vect_motor_cmd.clear();
      vect_motor_cmd.push_back(0.0);
      vect_motor_cmd.push_back(0.0);
      send_cmd_motor_to_roboclaw(&roboclaw, id_controler_A, vect_motor_cmd);

    } else {emergency_stop++;}
    Serial.println("A|0|"); 
    delay(10);
  }
}