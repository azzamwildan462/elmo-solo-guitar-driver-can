#include "elmo_can.h"

#include "rd-kits/keyboard_input.h"
#include "rd-kits/stdout_with_colour.h"

int8_t node_id[3] = {0x09, 0x0A, 0x0B};
int16_t node_id_sword[3] = {-1, -1, -1};

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
            case 'a':
                elmo_can_send_sync(s);
                break;

            case 's':
                elmo_can_setup_TPDO(s, node_id[0], 0x200, 0x01, 0x60690020);
                break;

            case 'd':
                elmo_can_setup_RPDO(s, node_id[0], 0x200, 0, 0x60FF0020);
                break;

            case 'w':
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
                // elmo_can_send_NMT(s, 0x00, node_id[1]);
                // elmo_can_send_NMT(s, 0x00, node_id[2]);
                break;
            case '2':
                elmo_can_send_NMT(s, 0x80, node_id[0]);
                // elmo_can_send_NMT(s, 0x80, node_id[1]);
                // elmo_can_send_NMT(s, 0x80, node_id[2]);
                break;
            case '3':
                elmo_can_send_NMT(s, 0x01, node_id[0]);
                // elmo_can_send_NMT(s, 0x01, node_id[1]);
                // elmo_can_send_NMT(s, 0x01, node_id[2]);
                break;

            case 'l':
            {
                node_id_sword[0] = elmo_can_set_s_word(s, node_id[0]);
                // node_id_sword[1] = elmo_can_set_s_word(s, node_id[1]);
                // node_id_sword[2] = elmo_can_set_s_word(s, node_id[2]);
                printf("S-Word: %d\n", node_id_sword[0]);
                break;
            }
            }
        }
        usleep(10000); // 50 hz (WTF RMS lite?)
    }
    return 0;
}