#include <elmo_can.h>

int main()
{
    int8_t s = elmo_can_init("can2");

    int8_t active_nodes[128] = {0};

    int8_t active_nodes_count = elmo_can_ping_all(s, active_nodes, 200);

    for (int i = 0; i < active_nodes_count; i++)
    {
        printf("Node %d is active\n", i);
    }

    return 0;
}