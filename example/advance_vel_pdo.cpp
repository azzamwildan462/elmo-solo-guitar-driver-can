#include <elmo_can.h>
#include <math.h>
#include <rd-kits/keyboard_input.h>

#include "rd-kits/keyboard_input.h"
#include "rd-kits/stdout_with_colour.h"

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
    LogWithColor(COLOR_CYAN, "Press 'k' to init motor\n");

    int8_t s = elmo_can_init("can0");

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
                elmo_can_setup_TPDO(s, node_id[0], 0x200, 0x01, 0x60690020);
                usleep(5000);
                elmo_can_setup_TPDO(s, node_id[1], 0x200, 0x01, 0x60690020);
                usleep(5000);
                elmo_can_setup_TPDO(s, node_id[2], 0x200, 0x01, 0x60690020);
                usleep(5000);

                elmo_can_setup_RPDO(s, node_id[0], 0x200, 0x01, 0x60FF0020);
                usleep(5000);
                elmo_can_setup_RPDO(s, node_id[1], 0x200, 0x01, 0x60FF0020);
                usleep(5000);
                elmo_can_setup_RPDO(s, node_id[2], 0x200, 0x01, 0x60FF0020);
                usleep(5000);

                int8_t stts0 = elmo_init_motor(s, node_id[0], ELMO_VELOCITY_MODE);
                int8_t stts1 = elmo_init_motor(s, node_id[1], ELMO_VELOCITY_MODE);
                int8_t stts2 = elmo_init_motor(s, node_id[2], ELMO_VELOCITY_MODE);
                printf("Init motor: %d \n", stts0);
                break;
            }

            case 'c':
            {
                uint16_t stts = elmo_can_read_req(s, node_id[0], 0x1800, 2);
                elmo_can_read(s, node_id[0], stts, 1000);
                printf("Transmission type: %x\n", stts);
                break;
            }

            case 'v':
            {
                uint16_t stts = elmo_can_read_req(s, node_id[0], 0x1800, 2);
                elmo_can_read(s, node_id[0], stts, 1000);
                printf("COB-ID used: %d\n", stts);
                break;
            }

            case 'x':
            {
                uint32_t stts = elmo_can_read_req(s, node_id[0], 0x1005, 0);
                elmo_can_read(s, node_id[0], stts, 1000);
                printf("COB-ID SYNC: %x\n", stts);
                break;
            }
            case 'm':
                elmo_can_send_sync(s);
                break;

            case 'n':
                elmo_can_setup_TPDO(s, node_id[0], 0x200, 0x01, 0x60690020);
                break;

            case 'b':
                elmo_can_setup_RPDO(s, node_id[0], 0x200, 0, 0x60FF0020);
                break;

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

            case 'j':
                elmo_can_set_target_velocity_sync(s, node_id[0], 4000);
                elmo_can_set_target_velocity_sync(s, node_id[1], 4000);
                elmo_can_set_target_velocity_sync(s, node_id[2], 4000);

                break;

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
                printf("S-Word: %d\n", node_id_sword[0]);
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
            elmo_can_set_target_velocity_sync(s, node_id[0], vel_left_motor);
        }
        if (node_id_sword[1] > 0)
        {
            elmo_can_set_target_velocity_sync(s, node_id[1], vel_right_motor);
        }
        if (node_id_sword[2] > 0)
        {
            elmo_can_set_target_velocity_sync(s, node_id[2], vel_rear_motor);
        }

        elmo_can_send_sync(s);

        usleep(20000); // 50 hz (WTF RMS lite?) // 50 hz (WTF RMS lite?)
    }
    return 0;
}