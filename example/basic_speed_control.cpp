#include <elmo_can.h>

#include <rd-kits/keyboard_input.h>

#define NODE_ID 0x07

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
                int8_t stts = elmo_init_motor(s, node_id);
                printf("Init motor: %d\n", stts);
                break;
            }

            case 'j':
                elmo_can_set_target_velocity(s, node_id, 5000);
                break;
            case 'n':
                elmo_can_set_target_velocity(s, node_id, 0);
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

            case 'l':
            {
                uint16_t s_word = elmo_can_set_s_word(s, node_id);
                printf("S-Word: %d\n", s_word);
                break;
            }
            }
        }

        usleep(20000); // 50 hz (WTF RMS lite?)
    }

    return 0;
}