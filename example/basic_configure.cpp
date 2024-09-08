#include <elmo_can.h>

#include <rd-kits/keyboard_input.h>

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
            case 'q':
            {
                elmo_can_set_node_id(s, node_id, 0x09);
                elmo_can_save_config(s, node_id);
                printf("Node ID changed to 0x10, waiting...\n");
                usleep(100000);
                elmo_can_send_NMT(s, 0x82, node_id);
                printf("Node ID changed to 0x10, done\n");
            }
            // elmo_can_write(s, 0x10, 0x1010, 0x01, 0x65766173);

            break;

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

            case 'k':
            {
                struct can_frame frame;
                frame.can_id = ELMO_COBID_RPDO2 + node_id; // RPDO2 request to node ID
                frame.can_dlc = 8;                         // Data length for RPDO2 request is 8

                // PP[13] = new_node_id
                frame.data[0] = 0x53; // 'P'
                frame.data[1] = 0x56; // 'P'
                frame.data[2] = 0x00; // LSB index
                frame.data[3] = 0x40; // MSB index
                frame.data[4] = 0x00; // Value (low byte)
                frame.data[5] = 0x00; // Value (second byte)
                frame.data[6] = 0x00; // Value (third byte)
                frame.data[7] = 0x00; // Value (high byte)

                // Send the RPDO2 write request
                if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
                {
                    perror("Write");
                    return -1;
                }
                break;
            }

            case 'n':
                elmo_can_write(s, node_id, 0x1010, 0x01, 0x65766173);
                break;

            case 'w':
                // elmo_can_set_c_word(s, node_id, 0b0111);
                elmo_can_set_bitrate(s, node_id, 0x00);
                elmo_can_save_config(s, node_id);
                printf("WAIT...\n");
                usleep(1000000);
                elmo_can_send_NMT(s, ELMO_NMT_RESET_COMM, node_id);
                printf("DONE\n");

                break;

            case 'a':
            {
                uint8_t actual_node_id = elmo_can_read_req(s, node_id, 0x100B, 0x00);
                elmo_can_read(s, node_id, actual_node_id, 1000);
                printf("Actual Node ID: %d\n", actual_node_id);
                break;
            }

            case 'p':
                elmo_can_ping(s, node_id);
                break;

                // case 'z':
                //     elmo_can_write(s, node_id, 0x100B, 0x00, 0x09);
                //     break;

            case 'l':
            {
                uint16_t s_word = elmo_can_set_s_word(s, node_id);
                printf("S-Word: %d\n", s_word);
                break;
            }

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
            }
        }

        usleep(20000); // 50 hz (WTF RMS lite?)
    }

    return 0;
}
