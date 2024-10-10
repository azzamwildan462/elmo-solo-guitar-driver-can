#include <elmo_can.h>
#include <math.h>
#include <rd-kits/keyboard_input.h>

#define DEG2RAD *0.017452925
#define RAD2DEG *57.295780
#define RAD2CNTS (18000.0f * 0.5 / M_PI)
#define MOTOR_RADIUS 6.3

float vel_x_robot, vel_y_robot, vel_theta_robot;
int32_t vel_left_motor, vel_right_motor, vel_rear_motor;
int8_t node_id[3] = {0x09, 0x0A, 0x0B};
int16_t node_id_sword[3] = {-1, -1, -1};

float cps2dps(float cps)
{
    return cps * 360.0 / (13074.4);
}

float dps2cps(float dps)
{
    return dps * (13074.4) / 360.0;
}

float rpm2cps(float rpm)
{
    return rpm * 360.0 / 60.0;
}

int main()
{
    int8_t s = elmo_can_init("can0");

    printf("Press 'k' to init motor\n");
    while (1)
    {
        elmo_can_clear_recv_buffer(s);

        if (kbhit())
        {
            char c = std::getchar();

            switch (c)
            {
            case 'k':
            {
                int8_t stts0 = elmo_init_motor(s, node_id[0]);
                int8_t stts1 = elmo_init_motor(s, node_id[1]);
                int8_t stts2 = elmo_init_motor(s, node_id[2]);
                printf("Init motor: %d %d %d\n", stts0, stts1, stts2);
                break;
            }

            case '0':
                elmo_can_send_NMT(s, 0x81, node_id[0]);
                elmo_can_send_NMT(s, 0x81, node_id[1]);
                elmo_can_send_NMT(s, 0x81, node_id[2]);
                break;
            case '1':
                elmo_can_send_NMT(s, 0x00, node_id[0]);
                elmo_can_send_NMT(s, 0x00, node_id[1]);
                elmo_can_send_NMT(s, 0x00, node_id[2]);
                break;
            case '2':
                elmo_can_send_NMT(s, 0x80, node_id[0]);
                elmo_can_send_NMT(s, 0x80, node_id[1]);
                elmo_can_send_NMT(s, 0x80, node_id[2]);
                break;
            case '3':
                elmo_can_send_NMT(s, 0x01, node_id[0]);
                elmo_can_send_NMT(s, 0x01, node_id[1]);
                elmo_can_send_NMT(s, 0x01, node_id[2]);
                break;

            case 'l':
            {
                node_id_sword[0] = elmo_can_set_s_word(s, node_id[0]);
                node_id_sword[1] = elmo_can_set_s_word(s, node_id[1]);
                node_id_sword[2] = elmo_can_set_s_word(s, node_id[2]);
                printf("S-Word: %d %d %d\n", node_id_sword[0], node_id_sword[1], node_id_sword[2]);
                break;
            }

            case ' ':
            {
                vel_y_robot = 0;
                vel_x_robot = 0;
                vel_theta_robot = 0;
                break;
            }

            case 'o':
            {
                vel_y_robot = 0;
                vel_x_robot = 0;
                vel_theta_robot = 20;
                break;
            }

            case 'p':
            {
                vel_y_robot = 0;
                vel_x_robot = 0;
                vel_theta_robot = -20;
                break;
            }

            case 'w':
            {
                vel_y_robot = 20;
                vel_x_robot = 0;
                vel_theta_robot = 0;
                break;
            }

            case 'a':
            {
                vel_y_robot = 0;
                vel_x_robot = -20;
                vel_theta_robot = 0;
                break;
            }

            case 's':
            {
                vel_y_robot = -20;
                vel_x_robot = 0;
                vel_theta_robot = 0;
                break;
            }

            case 'd':
            {
                vel_y_robot = 0;
                vel_x_robot = 20;
                vel_theta_robot = 0;
                break;
            }
            }
        }

        vel_left_motor = vel_theta_robot + vel_x_robot * cosf(240 DEG2RAD) + vel_y_robot * sinf(240 DEG2RAD);
        vel_right_motor = vel_theta_robot + vel_x_robot * cosf(120 DEG2RAD) + vel_y_robot * sinf(120 DEG2RAD);
        vel_rear_motor = vel_theta_robot + vel_x_robot;

        vel_left_motor = vel_left_motor / MOTOR_RADIUS * RAD2CNTS * 1;
        vel_right_motor = vel_right_motor / MOTOR_RADIUS * RAD2CNTS * 1;
        vel_rear_motor = vel_rear_motor / MOTOR_RADIUS * RAD2CNTS * 1; // 6959

        printf("%.2f %.2f %.2f | %d %d %d || %d %d %d\n", vel_x_robot, vel_y_robot, vel_theta_robot, vel_left_motor, vel_right_motor, vel_rear_motor,
               node_id_sword[0], node_id_sword[1], node_id_sword[2]);

        if (node_id_sword[0] > 0)
        {
            elmo_can_set_target_velocity(s, node_id[0], vel_left_motor);
        }
        if (node_id_sword[1] > 0)
        {
            elmo_can_set_target_velocity(s, node_id[1], vel_right_motor);
        }
        if (node_id_sword[2] > 0)
        {
            elmo_can_set_target_velocity(s, node_id[2], vel_rear_motor);
        }

        usleep(20000); // 50 hz (WTF RMS lite?)
    }

    return 0;
}

// TESTING REHAN
/*

p1
6108
19176

d = 19176 - 6108 = 13068

p2
19176
32224

d = 32224 - 19176 = 13048

p3
32224
45224

d = 45224 - 32224 = 13000

p4
45224
58225

d = 58225 - 45224 = 13001

p5
58225
71360

d = 71360 - 58225 = 13135

mean = 13074.4

*/