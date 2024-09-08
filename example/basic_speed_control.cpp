#include <elmo_can.h>

#include <rd-kits/keyboard_input.h>

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
    int8_t node_id = 0x09;
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
                int8_t stts = elmo_init_motor(s, node_id);
                printf("Init motor: %d\n", stts);
                break;
            }

            case 'q':
                elmo_can_set_c_word(s, node_id, 0b0110);
                break;

            case 'w':
                elmo_can_set_c_word(s, node_id, 0b0111);
                break;

            case ' ':
                elmo_can_set_c_word(s, node_id, 0b0111);
                break;

            case 'u':
                elmo_can_set_target_velocity(s, node_id, 70000);
                break;
            case 'j':
                elmo_can_set_target_velocity(s, node_id, 1000);
                break;
            case 'n':
                elmo_can_set_target_velocity(s, node_id, 0);
                break;

            case 'p':
                elmo_can_set_target_velocity(s, node_id, dps2cps(120));
                break;

            case '0':
                elmo_can_send_NMT(s, 0x81, node_id);
                break;
            case '1':
                elmo_can_send_NMT(s, 0x00, node_id);
                break;
            case '2':
                elmo_can_send_NMT(s, 0x80, node_id);
                break;
            case '3':
                elmo_can_send_NMT(s, 0x01, node_id);
                break;

            case 's':
                elmo_can_set_c_word(s, node_id, 0x0111);
                break;

            case 'l':
            {
                uint16_t s_word = elmo_can_set_s_word(s, node_id);
                printf("S-Word: %d\n", s_word);
                break;
            }

            case 'e':
            {
                int32_t enc_px = elmo_can_get_enc_px(s, node_id);
                printf("Enc Px: %d\n", enc_px);
                break;
            }

            case 'v':
            {
                int32_t enc_vx = elmo_can_get_enc_vx(s, node_id);
                printf("Enc Vx: %d\n", enc_vx);
                break;
            }

            case 'z':
            {
                uint8_t stts = elmo_can_read_req(s, node_id, ELMO_ERROR_CODE, 0);
                elmo_can_read(s, node_id, stts, 10);
                printf("ER1001: %d\n", stts);
                break;
            }
            case 'x':
            {
                uint32_t stts = elmo_can_read_req(s, node_id, ELMO_ERROR_REGISTER, 0x01);
                elmo_can_read(s, node_id, stts, 1000);
                printf("ER1003: %x\n", stts);
                break;
            }
            case 'c':
            {
                uint16_t stts = elmo_can_read_req(s, node_id, ELMO_ERROR_CODE_402, 0);
                elmo_can_read(s, node_id, stts, 1000);
                printf("ER603f: %x\n", stts);
                break;
            }
            case 'r':
            {
                int16_t stts = elmo_can_read_req(s, node_id, 0x6076, 0);
                elmo_can_read(s, node_id, stts, 1000);
                printf("stts: %d\n", stts);
                break;
            }
            case 'g':
            {
                int16_t stts = elmo_can_read_req(s, node_id, 0x6073, 0);
                elmo_can_read(s, node_id, stts, 1000);
                printf("stts: %d\n", stts);
                break;
            }
            case 't':
            {
                uint32_t stts = elmo_can_read_req(s, node_id, ELMO_MOTOR_RATE_CURRENT, 0);
                elmo_can_read(s, node_id, stts, 1000);
                printf("stts: %d\n", stts);
                break;
            }
            case 'b':
            {
                elmo_can_write(s, node_id, 0x6073, 0x00, 280);
                break;
            }
            }
        }

        // int16_t torq_6077 = elmo_can_read_req(s, node_id, ELMO_ACTUAL_TORQUE, 0);
        // elmo_can_read(s, node_id, torq_6077, 1000);

        // uint32_t torq_6077 = elmo_can_read_req(s, node_id, 0x6502, 0);
        // elmo_can_read(s, node_id, torq_6077, 1000);

        // int16_t cur_6078 = elmo_can_read_req(s, node_id, ELMO_ACTUAL_CURRENT, 0);
        // elmo_can_read(s, node_id, cur_6078, 1000);

        // int32_t cur_6069 = elmo_can_read_req(s, node_id, 0x6069, 0);
        // elmo_can_read(s, node_id, cur_6069, 1000);

        // int32_t enc_px = elmo_can_get_enc_px(s, node_id);
        // printf("Enc Px: %d %d|| %d %d \n", enc_px, cur_6069, torq_6077, cur_6078);

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