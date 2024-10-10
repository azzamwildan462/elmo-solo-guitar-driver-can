/**
 * @file basic_configure.cpp
 *
 * This is the example to set a node id and baudrate for ELMO
 *
 * Just focus on letter '3', 'q', and 'w'
 *
 * Make sure you press '3' before pressing 'q' and 'w'
 * The target node id and target baudrate is hardcoded, you an change it manually
 */

#include <elmo_can.h>

#include <rd-kits/keyboard_input.h>

int main()
{
    int8_t current_node_id = 0x09;
    int8_t target_node_id = 0x10;
    int8_t target_baudrate = ELMO_BITRATE_1M;
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
            case 'q':
            {
                elmo_can_set_node_id(s, current_node_id, target_node_id);
                elmo_can_save_config(s, current_node_id);
                printf("Node ID changed to 0x%x, waiting...\n", target_node_id);
                usleep(100000);
                elmo_can_send_NMT(s, 0x82, current_node_id); // Reset communication refer to MAN
                printf("Node ID changed to 0x%x, done\n", target_node_id);
            }
            break;

            case 'x':
            {
                uint32_t stts = elmo_can_read_req(s, current_node_id, ELMO_ERROR_REGISTER, 0x01);
                elmo_can_read(s, current_node_id, stts, 1000);
                printf("ER1003: %x\n", stts);
                break;
            }
            case 'c':
            {
                uint16_t stts = elmo_can_read_req(s, current_node_id, ELMO_ERROR_CODE_402, 0);
                elmo_can_read(s, current_node_id, stts, 1000);
                printf("ER603f: %x\n", stts);
                break;
            }

            case 'w':
                elmo_can_set_bitrate(s, current_node_id, ELMO_BITRATE_1M);
                elmo_can_save_config(s, current_node_id);
                printf("WAIT...\n");
                usleep(1000000);
                elmo_can_send_NMT(s, ELMO_NMT_RESET_COMM, current_node_id); // Reset communication refer to MAN for take effect
                printf("DONE\n");

                break;

            case 'a':
            {
                uint8_t actual_node_id = elmo_can_read_req(s, current_node_id, 0x100B, 0x00);
                elmo_can_read(s, current_node_id, actual_node_id, 1000);
                printf("Actual Node ID: %d\n", actual_node_id);
                break;
            }

            case 'p':
                elmo_can_ping(s, current_node_id);
                break;

            case 'l':
            {
                uint16_t s_word = elmo_can_set_s_word(s, current_node_id);
                printf("S-Word: %d\n", s_word);
                break;
            }

            case '0':
                elmo_can_send_NMT(s, 0x81, current_node_id);
                break;
            case '1':
                elmo_can_send_NMT(s, 0x00, current_node_id);
                break;
            case '2':
                elmo_can_send_NMT(s, 0x80, current_node_id);
                break;
            case '3':
                elmo_can_send_NMT(s, 0x01, current_node_id);
                break;
            }
        }

        usleep(20000); // 50 hz (WTF RMS lite?)
    }

    return 0;
}
