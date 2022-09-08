#include <Arduino.h>
#include <vector>

#include <RoboClaw.h>
#include "main_function.h"

int read_serial_msg(HardwareSerial* Serial, std::vector<String>& serial_msg_vect)
{
    /*
     * Info return:
     * -1 = input serial is not a standard msg.
     *  0 = command motor.
     */

    serial_msg_vect.clear();

    String msg_brut;
    msg_brut   = Serial->readStringUntil('\n');

    int idx_A  = -1;
    int idx_B  = 0;

    String data = "";

    while(msg_brut.indexOf('|', idx_A+1 ) != -1)
    {
        idx_B = msg_brut.indexOf('|', idx_A+1 );
        data  = msg_brut.substring(idx_A+1, idx_B);
        serial_msg_vect.push_back(data);
        idx_A = idx_B;
    }

    if(serial_msg_vect[0] == "M")
    {
        if(serial_msg_vect[1] == "0")
        {
            if(serial_msg_vect.size() == 4) return 0;
        }
    }

    return -1;
}

void ping_inboard_led(int delay_ms)
{
    digitalWrite(ONBOARD_LED, true);
    delay(delay_ms);
    digitalWrite(ONBOARD_LED, false);
}

void send_cmd_motor_to_roboclaw(RoboClaw* roboclaw, int roboclaw_adress, std::vector<double> vect_cmd_motor)
{
    // //!\\ Motor Inversion.

    roboclaw->SpeedAccelDistanceM1(roboclaw_adress, 50000, cm_to_pulse(vect_cmd_motor[1]),  10000, 1);
    roboclaw->SpeedAccelDistanceM2(roboclaw_adress, 50000, cm_to_pulse(vect_cmd_motor[0]),  10000, 1);
}

uint32_t cm_to_pulse(float cm_s)
{
    return cm_s * M_TIC_QUAD;
}