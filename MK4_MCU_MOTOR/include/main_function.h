#include <Arduino.h>

#define ONBOARD_LED 2
#define M_TIC_QUAD 20000
#define TIC_PER_MOTOR_REVOLUTION 4480

struct Motor
{
    int motor_id;
    float kp, ki, kd;
    int QPPS;
    int adress;

    Motor(int a, float b, float c, float d, int e, int f)
        : motor_id(a)
        , kp(b)
        , ki(c)
        , kd(d)
        , QPPS(e)
        , adress(f)
        {}
};

int read_serial_msg(HardwareSerial* Serial, std::vector<String>& serial_msg_vect);
void ping_inboard_led(int delay_ms);
void send_cmd_motor_to_roboclaw(RoboClaw* roboclaw, int roboclaw_adress, std::vector<double> vect_cmd_motor);
uint32_t cm_to_pulse(float cm_s);