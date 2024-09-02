/**
 *
 * Motor Rate: 25000
 */

#include <elmo_can.h>

#include <rd-kits/keyboard_input.h>

#define KP 0.001
#define KI 0.00001

int32_t enc_now = 0;
float target_vel = 0;

int32_t set_velocity()
{
    static float error = 0;
    static float integral = 0;
    static float output = 0;

    error = target_vel - enc_now;

    integral += error * KI;

    if (integral > 100)
        integral = 100;
    else if (integral < -100)
        integral = -100;

    // printf("%.2f %.2f %.2f\n", error, integral, output);

    output = KP * error + integral;

    if (output > 50)
        output = 50;
    else if (output < -50)
        output = -50;

    return (int32_t)output;
}

int main()
{
    int8_t node_id = 0x07;
    int8_t s = elmo_can_init("can2");

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
                int8_t stts = elmo_init_motor(s, node_id, ELMO_TORQUE_MODE);
                printf("Init motor: %d\n", stts);
                break;
            }
            case 'q':
            {
                uint8_t stts = elmo_can_read_req(s, node_id, ELMO_ERROR_CODE, 0);
                elmo_can_read(s, node_id, stts, 10);
                printf("ER1001: %d\n", stts);
                break;
            }
            case 'w':
            {
                uint32_t stts = elmo_can_read_req(s, node_id, ELMO_ERROR_REGISTER, 0x01);
                elmo_can_read(s, node_id, stts, 1000);
                printf("ER1003: %x\n", stts);
                break;
            }
            case 'e':
            {
                uint16_t stts = elmo_can_read_req(s, node_id, ELMO_ERROR_CODE_402, 0);
                elmo_can_read(s, node_id, stts, 1000);
                printf("ER603f: %x\n", stts);
                break;
            }

            case 'g':
            {
                uint32_t motor_rate = elmo_can_get_motor_rate_current(s, node_id);
                printf("Motor Rate: %d\n", motor_rate);
                break;
            }

            case 't':
                elmo_can_write(s, node_id, ELMO_TORQUE_SLOPE, 0, 40);
                break;

            case ' ':
                elmo_can_set_target_torque(s, node_id, (int16_t)0);
                target_vel = 0;
                break;

            case 'y':
                elmo_can_set_target_torque(s, node_id, (int16_t)40);
                break;
            case 'h':
                elmo_can_set_target_torque(s, node_id, (int16_t)-40);
                break;
            case 'u':
                target_vel = 22000;
                break;
            case 'j':
                target_vel = -22000;
                break;
            case 'n':
                elmo_can_set_quick_stop(s, node_id, 0x0000);
                break;

            case 'b':
                elmo_can_set_quick_stop(s, node_id, 0x0005);
                break;

            case 'p':
                elmo_can_set_target_torque(s, node_id, 1000);
                break;

            case 'l':
            {
                uint16_t s_word = elmo_can_set_s_word(s, node_id);
                printf("S-Word: %d\n", s_word);
                break;
            }

            case '0':
                elmo_can_send_NMT(s, 0x81, node_id);
                break;

                return 0;
            }
        }

        enc_now = elmo_can_get_enc_vx(s, node_id);
        int32_t target_torque = set_velocity();
        elmo_can_set_target_torque(s, node_id, target_torque);

        {
            int16_t torq_6077 = elmo_can_read_req(s, node_id, ELMO_ACTUAL_TORQUE, 0);
            elmo_can_read(s, node_id, torq_6077, 1000);

            int16_t cur_6078 = elmo_can_read_req(s, node_id, ELMO_ACTUAL_CURRENT, 0);
            elmo_can_read(s, node_id, cur_6078, 1000);

            printf("%d %d %d || %d\n", enc_now, torq_6077, cur_6078, target_torque);
        }

        usleep(100 * 1e3);
    }

    return 0;
}