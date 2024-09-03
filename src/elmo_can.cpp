#include "elmo_can.h"

int8_t elmo_can_clear_recv_buffer(int8_t s)
{
    struct can_frame frame;
    int flags = fcntl(s, F_GETFL, 0);
    int nbytes;

    // Set socket to non-blocking mode
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    // Read and discard all frames
    while ((nbytes = read(s, &frame, sizeof(struct can_frame))) > 0)
    {
        // Frame read and discarded
    }

    // Check for read error
    if (nbytes < 0 && errno != EAGAIN)
    {
        perror("Error clearing CAN receive buffer");
        return -1;
    }

    // Restore original socket flags (back to blocking mode if it was originally)
    fcntl(s, F_SETFL, flags);

    return 0;
}

int8_t elmo_can_init(const char *interface)
{
    int8_t s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, interface);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
        return -1;
    }

    return s;
}

int8_t elmo_can_send_NMT(int8_t s, uint8_t command, uint8_t node_id)
{
    struct can_frame frame;
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = command;
    frame.data[1] = node_id;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Error while sending NMT");
        return -1;
    }

    return 0;
}

int8_t elmo_can_set_heartbeat(int8_t s, uint8_t node_id, uint16_t ms)
{
    struct can_frame frame;
    frame.can_id = ELMO_COBID_SDO_REQUEST + node_id;
    frame.can_dlc = 8;

    frame.data[0] = 0x22;             // Command byte for "Write 2 bytes"
    frame.data[1] = 0x17;             // Index low byte (0x1017 - heartbeat producer time)
    frame.data[2] = 0x10;             // Index high byte
    frame.data[3] = 0x00;             // Subindex
    frame.data[4] = ms & 0xFF;        // Value low byte
    frame.data[5] = (ms >> 8) & 0xFF; // Value high byte
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Error while setting heartbeat");
        return -1;
    }

    return 0;
}

// Function to handle received heartbeat messages
uint8_t elmo_can_handle_heartbeat(const struct can_frame *frame)
{
    uint8_t node_id = frame->can_id & 0x7F; // Extract node ID (lower 7 bits)
    uint8_t nmt_state = frame->data[0];     // NMT state is in the first byte

    switch (nmt_state)
    {
    case 0x00:
        printf("Node %d: Bootup\n", node_id);
        break;
    case 0x04:
        printf("Node %d: Stopped\n", node_id);
        break;
    case 0x05:
        printf("Node %d: Operational\n", node_id);
        break;
    case 0x7F:
        printf("Node %d: Pre-operational\n", node_id);
        break;
    default:
        printf("Node %d: Unknown NMT state 0x%02X\n", node_id, nmt_state);
        break;
    }

    return nmt_state;
}

int8_t elmo_can_check_NMT(int8_t s, uint8_t node_id, uint16_t timeout_ms)
{
    if (elmo_can_set_heartbeat(s, node_id, 50) < 0)
    {
        return -1;
    }

    printf("Waiting for heartbeat from node %d...\n", node_id);
    usleep(timeout_ms * 1e3); // Wait for the heartbeat to be set

    struct can_frame frame;
    fd_set readfds;
    struct timeval timeout;

    // Set up the timeout for the receive operation
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&readfds);
    FD_SET(s, &readfds);

    // Wait for data or timeout
    int ret = select(s + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(s, &readfds))
    {
        // Read the CAN frame
        if (read(s, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("Read");
            return 0xFF;
        }

        if ((frame.can_id & 0x700) == 0x700)
        { // Check if it's a heartbeat message
            elmo_can_set_heartbeat(s, (frame.can_id & 0x7F), 0);
            elmo_can_clear_recv_buffer(s);
            return elmo_can_handle_heartbeat(&frame);
        }
    }

    return 0;
}

int8_t elmo_can_ping(int8_t s, uint8_t node_id, uint16_t timeout_ms)
{
    printf("Pinging node %d...\n", node_id);
    if (elmo_can_send_NMT(s, node_id, 0x01) < 0)
    {
        return -1;
    }

    if (elmo_can_check_NMT(s, node_id, timeout_ms) != 0x05)
    {
        return -1;
    }

    if (elmo_can_set_heartbeat(s, node_id, 0) < 0)
    {
        return -1;
    }

    return 0;
}

int8_t elmo_can_ping_all(int8_t s, int8_t *nodes, uint16_t timeout_ms)
{
    int8_t node_active = 0;
    for (int i = 1; i <= 127; i++)
    {
        elmo_can_clear_recv_buffer(s);
        if (elmo_can_ping(s, i, timeout_ms) == 0)
        {
            nodes[node_active] = 1;
            node_active++;
        }
        usleep(1000);
    }

    return node_active;
}

int8_t elmo_can_read_req(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex)
{
    struct can_frame frame;
    frame.can_id = ELMO_COBID_SDO_REQUEST + node_id; // SDO request to node ID
    frame.can_dlc = 8;                               // Data length for SDO request is 8

    // SDO request to read the specified object
    frame.data[0] = 0x40;                // Command byte: Initiate SDO upload (read)
    frame.data[1] = index & 0xFF;        // Index (low byte)
    frame.data[2] = (index >> 8) & 0xFF; // Index (high byte)
    frame.data[3] = subindex;            // Sub-index
    frame.data[4] = 0x00;                // Reserved
    frame.data[5] = 0x00;                // Reserved
    frame.data[6] = 0x00;                // Reserved
    frame.data[7] = 0x00;                // Reserved

    // Send the SDO request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}

int8_t elmo_can_read(int8_t s, int8_t node_id, uint32_t &value, uint16_t timeout_ms)
{
    struct can_frame frame;
    fd_set readfds;
    struct timeval timeout;
    memset(&frame, 0, sizeof(struct can_frame));

    // Set up the timeout for the receive operation
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&readfds);
    FD_SET(s, &readfds);

    // Wait for data or timeout
    int ret = select(s + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(s, &readfds))
    {
        // Read the CAN frame
        if (read(s, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("Read");
            return -1;
        }

        // Check if the response is from the expected node
        if (frame.can_id == ELMO_COBID_SDO_RESPONSE + node_id)
        { // SDO response from node ID
            // Extract the data from the response
            value = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
            return 0;
        }
    }

    return -1;
}

int8_t elmo_can_read(int8_t s, int8_t node_id, int32_t &value, uint16_t timeout_ms)
{
    struct can_frame frame;
    fd_set readfds;
    struct timeval timeout;
    memset(&frame, 0, sizeof(struct can_frame));

    // Set up the timeout for the receive operation
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&readfds);
    FD_SET(s, &readfds);

    // Wait for data or timeout
    int ret = select(s + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(s, &readfds))
    {
        // Read the CAN frame
        if (read(s, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("Read");
            return -1;
        }

        // Check if the response is from the expected node
        if (frame.can_id == ELMO_COBID_SDO_RESPONSE + node_id)
        { // SDO response from node ID
            // Extract the data from the response
            value = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
            return 0;
        }
    }

    return -1;
}

int8_t elmo_can_read(int8_t s, int8_t node_id, uint16_t &value, uint16_t timeout_ms)
{
    struct can_frame frame;
    fd_set readfds;
    struct timeval timeout;
    memset(&frame, 0, sizeof(struct can_frame));

    // Set up the timeout for the receive operation
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&readfds);
    FD_SET(s, &readfds);

    // Wait for data or timeout
    int ret = select(s + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(s, &readfds))
    {
        // Read the CAN frame
        if (read(s, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("Read");
            return -1;
        }

        // Check if the response is from the expected node
        if (frame.can_id == ELMO_COBID_SDO_RESPONSE + node_id)
        { // SDO response from node ID
            // Extract the data from the response
            value = frame.data[4] | (frame.data[5] << 8);
            return 0;
        }
    }

    return -1;
}

int8_t elmo_can_read(int8_t s, int8_t node_id, int16_t &value, uint16_t timeout_ms)
{
    struct can_frame frame;
    fd_set readfds;
    struct timeval timeout;
    memset(&frame, 0, sizeof(struct can_frame));

    // Set up the timeout for the receive operation
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&readfds);
    FD_SET(s, &readfds);

    // Wait for data or timeout
    int ret = select(s + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(s, &readfds))
    {
        // Read the CAN frame
        if (read(s, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("Read");
            return -1;
        }

        // Check if the response is from the expected node
        if (frame.can_id == ELMO_COBID_SDO_RESPONSE + node_id)
        { // SDO response from node ID
            // Extract the data from the response
            value = frame.data[4] | (frame.data[5] << 8);
            return 0;
        }
    }

    return -1;
}

int8_t elmo_can_read(int8_t s, int8_t node_id, uint8_t &value, uint16_t timeout_ms)
{
    struct can_frame frame;
    fd_set readfds;
    struct timeval timeout;
    memset(&frame, 0, sizeof(struct can_frame));

    // Set up the timeout for the receive operation
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&readfds);
    FD_SET(s, &readfds);

    // Wait for data or timeout
    int ret = select(s + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(s, &readfds))
    {
        // Read the CAN frame
        if (read(s, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("Read");
            return -1;
        }

        // Check if the response is from the expected node
        if (frame.can_id == ELMO_COBID_SDO_RESPONSE + node_id)
        { // SDO response from node ID
            // Extract the data from the response
            value = frame.data[4];
            return 0;
        }
    }

    return -1;
}

int8_t elmo_can_read(int8_t s, int8_t node_id, int8_t &value, uint16_t timeout_ms)
{
    struct can_frame frame;
    fd_set readfds;
    struct timeval timeout;
    memset(&frame, 0, sizeof(struct can_frame));

    // Set up the timeout for the receive operation
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&readfds);
    FD_SET(s, &readfds);

    // Wait for data or timeout
    int ret = select(s + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(s, &readfds))
    {
        // Read the CAN frame
        if (read(s, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("Read");
            return -1;
        }

        // Check if the response is from the expected node
        if (frame.can_id == ELMO_COBID_SDO_RESPONSE + node_id)
        { // SDO response from node ID
            // Extract the data from the response
            value = frame.data[4];
            return 0;
        }
    }

    return -1;
}

int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, uint32_t value)
{
    struct can_frame frame;
    frame.can_id = ELMO_COBID_SDO_REQUEST + node_id; // SDO request to node ID
    frame.can_dlc = 8;                               // Data length for SDO request is 8

    // SDO request to write the specified object
    frame.data[0] = 0x23;                 // Command byte: Initiate SDO download (write)
    frame.data[1] = index & 0xFF;         // Index (low byte)
    frame.data[2] = (index >> 8) & 0xFF;  // Index (high byte)
    frame.data[3] = subindex;             // Sub-index
    frame.data[4] = value & 0xFF;         // Value (byte 0)
    frame.data[5] = (value >> 8) & 0xFF;  // Value (byte 1)
    frame.data[6] = (value >> 16) & 0xFF; // Value (byte 2)
    frame.data[7] = (value >> 24) & 0xFF; // Value (byte 3)

    // Send the SDO request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}

int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, int32_t value)
{
    struct can_frame frame;
    frame.can_id = ELMO_COBID_SDO_REQUEST + node_id; // SDO request to node ID
    frame.can_dlc = 8;                               // Data length for SDO request is 8

    // SDO request to write the specified object
    frame.data[0] = 0x23;                 // Command byte: Initiate SDO download (write)
    frame.data[1] = index & 0xFF;         // Index (low byte)
    frame.data[2] = (index >> 8) & 0xFF;  // Index (high byte)
    frame.data[3] = subindex;             // Sub-index
    frame.data[4] = value & 0xFF;         // Value (byte 0)
    frame.data[5] = (value >> 8) & 0xFF;  // Value (byte 1)
    frame.data[6] = (value >> 16) & 0xFF; // Value (byte 2)
    frame.data[7] = (value >> 24) & 0xFF; // Value (byte 3)

    // Send the SDO request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}

int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, uint16_t value)
{
    struct can_frame frame;
    frame.can_id = ELMO_COBID_SDO_REQUEST + node_id; // SDO request to node ID
    frame.can_dlc = 6;                               // Data length for SDO request is 8

    // SDO request to write the specified object
    frame.data[0] = 0x23;                // Command byte: Initiate SDO download (write)
    frame.data[1] = index & 0xFF;        // Index (low byte)
    frame.data[2] = (index >> 8) & 0xFF; // Index (high byte)
    frame.data[3] = subindex;            // Sub-index
    frame.data[4] = value & 0xFF;        // Value (byte 0)
    frame.data[5] = (value >> 8) & 0xFF; // Value (byte 1)

    // Send the SDO request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}

int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, int16_t value)
{
    struct can_frame frame;
    frame.can_id = ELMO_COBID_SDO_REQUEST + node_id; // SDO request to node ID
    frame.can_dlc = 6;                               // Data length for SDO request is 8

    // SDO request to write the specified object
    frame.data[0] = 0x23;                // Command byte: Initiate SDO download (write)
    frame.data[1] = index & 0xFF;        // Index (low byte)
    frame.data[2] = (index >> 8) & 0xFF; // Index (high byte)
    frame.data[3] = subindex;            // Sub-index
    frame.data[4] = value & 0xFF;        // Value (byte 0)
    frame.data[5] = (value >> 8) & 0xFF; // Value (byte 1)

    // Send the SDO request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}

int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, uint8_t value)
{
    struct can_frame frame;
    frame.can_id = ELMO_COBID_SDO_REQUEST + node_id; // SDO request to node ID
    frame.can_dlc = 5;                               // Data length for SDO request is 8

    // SDO request to write the specified object
    frame.data[0] = 0x23;                // Command byte: Initiate SDO download (write)
    frame.data[1] = index & 0xFF;        // Index (low byte)
    frame.data[2] = (index >> 8) & 0xFF; // Index (high byte)
    frame.data[3] = subindex;            // Sub-index
    frame.data[4] = value;               // Value

    // Send the SDO request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}

int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, int8_t value)
{
    struct can_frame frame;
    frame.can_id = ELMO_COBID_SDO_REQUEST + node_id; // SDO request to node ID
    frame.can_dlc = 5;                               // Data length for SDO request is 8

    // SDO request to write the specified object
    frame.data[0] = 0x23;                // Command byte: Initiate SDO download (write)
    frame.data[1] = index & 0xFF;        // Index (low byte)
    frame.data[2] = (index >> 8) & 0xFF; // Index (high byte)
    frame.data[3] = subindex;            // Sub-index
    frame.data[4] = value;               // Value

    // Send the SDO request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}

int8_t elmo_can_send_BG_DLC4(int8_t s, int8_t node_id)
{
    struct can_frame frame;
    frame.can_id = ELMO_COBID_SDO_REQUEST + node_id; // SDO request to node ID
    frame.can_dlc = 4;                               // Data length for SDO request is 8

    // SDO write request to set the specified object
    frame.data[0] = 0x42; // Command byte: Initiate SDO download (write)
    frame.data[1] = 0x47; // Index (low byte)
    frame.data[2] = 0x00; // Index (high byte)
    frame.data[3] = 0x00; // Sub-index

    // Send the SDO write request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}

int8_t elmo_can_set_target_velocity(int8_t s, int8_t node_id, uint32_t target_vel)
{
    return elmo_can_write(s, node_id, ELMO_TARGET_VELOCITY, 0x00, target_vel);
}

int8_t elmo_can_set_target_position(int8_t s, int8_t node_id, uint32_t target_pos)
{
    return elmo_can_write(s, node_id, ELMO_TARGET_POSITION, 0x00, target_pos);
}

int8_t elmo_can_set_mode_op(int8_t s, int8_t node_id, int8_t mode_op)
{
    return elmo_can_write(s, node_id, ELMO_MODES_OF_OPERATION, 0x00, mode_op);
}

int8_t elmo_can_send_config_obj(int8_t s, int8_t node_id, uint32_t conf_obj)
{
    return elmo_can_write(s, node_id, ELMO_CONFIG_OBJECT, 0x00, conf_obj);
}

int32_t elmo_can_get_enc_px(int8_t s, int8_t node_id)
{
    if (elmo_can_read_req(s, node_id, ELMO_ACTUAL_POSITION, 0x00) < 0)
    {
        return -1;
    }

    int32_t enc_px;
    if (elmo_can_read(s, node_id, enc_px, 1000) < 0)
    {
        return -1;
    }

    return enc_px;
}

int32_t elmo_can_get_enc_vx(int8_t s, int8_t node_id)
{
    if (elmo_can_read_req(s, node_id, ELMO_ACTUAL_VELOCITY, 0x00) < 0)
    {
        return -1;
    }

    int32_t enc_vx;
    if (elmo_can_read(s, node_id, enc_vx, 1000) < 0)
    {
        return -1;
    }

    return enc_vx;
}

int8_t elmo_can_ignore_ls(int8_t s, int8_t node_id) // Ignore limit switch
{
    // Baca SIMPLIQ COMMAND IL[N] IP IB[N]
    // Menggunakan RPDO2 (COB-ID 0x300)
    struct can_frame frame;
    frame.can_id = ELMO_COBID_RPDO2 + node_id; // RPDO2 request to node ID
    frame.can_dlc = 8;                         // Data length for RPDO2 request is 8

    // IL[4]=4
    frame.data[0] = 0x49; // i
    frame.data[1] = 0x4c; // l
    frame.data[2] = 0x04; // Index (Low byte)
    frame.data[3] = 0x00; // Index (High byte)
    frame.data[4] = 0x04; // Data (low byte)
    frame.data[5] = 0x00; // Data (second byte)
    frame.data[6] = 0x00; // Data (third byte)
    frame.data[7] = 0x00; // Data (high byte)

    // Send the RPDO2 write request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    usleep(1000);

    struct can_frame frame2;
    frame2.can_id = ELMO_COBID_RPDO2 + node_id; // RPDO2 request to node ID
    frame2.can_dlc = 8;                         // Data length for RPDO2 request is 8

    // IL[3]=4
    frame2.data[0] = 0x49; // 'i'
    frame2.data[1] = 0x4c; // 'l'
    frame2.data[2] = 0x03; // LSB index
    frame2.data[3] = 0x00; // MSB index
    frame2.data[4] = 0x04; // Value (low byte)
    frame2.data[5] = 0x00; // Value (second byte)
    frame2.data[6] = 0x00; // Value (third byte)
    frame2.data[7] = 0x00; // Value (high byte)

    // Send the RPDO2 trigger (lihat pada Elmo DS-301)
    if (write(s, &frame2, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -2;
    }

    return 0;
}

uint16_t elmo_can_set_s_word(int8_t s, int8_t node_id)
{
    if (elmo_can_read_req(s, node_id, ELMO_STATUSWORD, 0x00) < 0)
    {
        return -1;
    }

    uint16_t s_word;
    if (elmo_can_read(s, node_id, s_word, 1000) < 0)
    {
        return -2;
    }

    return s_word;
}

int8_t elmo_can_set_c_word(int8_t s, int8_t node_id, uint16_t c_word)
{
    return elmo_can_write(s, node_id, ELMO_CONTROLWORD, 0x00, c_word);
}

int8_t elmo_init_motor(int8_t s, int8_t node_id, uint8_t mode)
{
    // Set NMT to operational (Baca Basic CANopen NMT DS-301)
    if (elmo_can_send_NMT(s, 0x00, node_id) < 0)
    {
        return -1;
    }
    usleep(5000);

    if (elmo_can_send_NMT(s, 0x80, node_id) < 0)
    {
        return -2;
    }
    usleep(5000);

    if (elmo_can_send_NMT(s, 0x01, node_id) < 0)
    {
        return -3;
    }
    usleep(5000);

    // IGNORE LIMIT SWITCH (IL[3]=4, IL[4]=4) (BACA SIMPLIQ COMMAND IL[N] IP IB[N])
    if (elmo_can_ignore_ls(s, node_id) < 0)
    {
        return -4;
    }
    usleep(5000);

    // Set Control word (Baca DS-402) (Power-up driver dan apply high voltage) (Untuk lebih jelas baca State Machine pada DS-402)
    if (elmo_can_set_c_word(s, node_id, 0b0110) < 0)
    {
        return -5;
    }
    usleep(5000);

    // Set Control word (Baca DS-402) (Memasuki state idle) (Untuk lebih jelas baca State Machine pada DS-402)
    if (elmo_can_set_c_word(s, node_id, 0b0111) < 0)
    {
        return -6;
    }
    usleep(5000);

    // Set mode kontrol driver (Baca DS-402) (uint32 0x6052 untuk cek mode yang disupport)
    if (elmo_can_set_mode_op(s, node_id, mode) < 0)
    {
        return -7;
    }
    usleep(5000);

    // Set Control word (Baca DS-402) (Mengaktifkan driver untuk kontrol motor)
    if (mode == ELMO_VELOCITY_MODE)
    {
        if (elmo_can_set_c_word(s, node_id, 0b1111) < 0)
        {
            return -8;
        }
    }
    else if (mode == ELMO_TORQUE_MODE)
    {
        if (elmo_can_set_c_word(s, node_id, 0b000001111) < 0) // + halt
        {
            return -8;
        }
    }
    else if (mode == ELMO_POSITION_MODE)
    {
        // I'm too lazy to create this mode, just refer to the manual DS-401 in Chapter 11 especially 11.1
        return -8;
    }
    usleep(5000);

    // SAFETY
    if (mode == ELMO_VELOCITY_MODE)
    {
        elmo_can_set_target_velocity(s, node_id, 0);
    }
    usleep(5000);

    // Untuk menjalankan Motion
    elmo_can_send_BG_DLC4(s, node_id);

    return 0;
}

int8_t elmo_can_set_target_torque(int8_t s, int8_t node_id, int16_t torque)
{
    return elmo_can_write(s, node_id, ELMO_TARGET_TORQUE, 0x00, torque);
}

uint32_t elmo_can_get_motor_rate_current(int8_t s, int8_t node_id)
{
    if (elmo_can_read_req(s, node_id, ELMO_MOTOR_RATE_CURRENT, 0x00) < 0)
    {
        return -1;
    }

    uint32_t motor_rate_current;
    if (elmo_can_read(s, node_id, motor_rate_current, 1000) < 0)
    {
        return -2;
    }

    return motor_rate_current;
}

int8_t elmo_can_set_quick_stop(int8_t s, int8_t node_id, int16_t quick_stop)
{
    return elmo_can_write(s, node_id, ELMO_QUICK_STOP_OPTION_CODE, 0x00, quick_stop);
}

int8_t elmo_can_set_node_id(int8_t s, int8_t node_id, int8_t new_node_id)
{
    // PP[13] = new_node_id
    // Menggunakan RPDO2 (COB-ID 0x300)
    struct can_frame frame;
    frame.can_id = ELMO_COBID_RPDO2 + node_id; // RPDO2 request to node ID
    frame.can_dlc = 8;                         // Data length for RPDO2 request is 8

    // PP[13] = new_node_id
    frame.data[0] = 0x50;        // 'P'
    frame.data[1] = 0x50;        // 'P'
    frame.data[2] = 0x0D;        // LSB index
    frame.data[3] = 0x00;        // MSB index
    frame.data[4] = new_node_id; // Value (low byte)
    frame.data[5] = 0x00;        // Value (second byte)
    frame.data[6] = 0x00;        // Value (third byte)
    frame.data[7] = 0x00;        // Value (high byte)

    // Send the RPDO2 write request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}
int8_t elmo_can_set_bitrate(int8_t s, int8_t node_id, int8_t bitrate)
{
    // PP[14] = bitrate (0: 1000 kbps, 1: 800 kbps, 2: 500 kbps, 3: 250 kbps, 4: 125 kbps, 5: 50 kbps, 6: 20 kbps, 7: 10 kbps)
    // Menggunakan RPDO2 (COB-ID 0x300)
    struct can_frame frame;
    frame.can_id = ELMO_COBID_RPDO2 + node_id; // RPDO2 request to node ID
    frame.can_dlc = 8;                         // Data length for RPDO2 request is 8

    // PP[14] = bitrate
    frame.data[0] = 0x50;    // 'P'
    frame.data[1] = 0x50;    // 'P'
    frame.data[2] = 0x0E;    // LSB index
    frame.data[3] = 0x00;    // MSB index
    frame.data[4] = bitrate; // Value (low byte)
    frame.data[5] = 0x00;    // Value (second byte)
    frame.data[6] = 0x00;    // Value (third byte)
    frame.data[7] = 0x00;    // Value (high byte)

    // Send the RPDO2 write request
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return -1;
    }

    return 0;
}

int8_t elmo_can_save_config(int8_t s, int8_t node_id)
{
    {
        struct can_frame frame;
        frame.can_id = ELMO_COBID_RPDO2 + node_id; // RPDO2 request to node ID
        frame.can_dlc = 4;                         // Data length for RPDO2 request is 8

        // HP
        frame.data[0] = 0x48; // 'H'
        frame.data[1] = 0x50; // 'P'
        frame.data[2] = 0x00; // LSB index
        frame.data[3] = 0x00; // MSB index

        // Send the CAN frame
        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            perror("Write");
            return 1;
        }
    }

    {
        struct can_frame frame;
        frame.can_id = ELMO_COBID_RPDO2 + node_id; // RPDO2 request to node ID
        frame.can_dlc = 4;                         // Data length for RPDO2 request is 8

        // SV
        frame.data[0] = 0x53; // 'S'
        frame.data[1] = 0x56; // 'V'
        frame.data[2] = 0x00; // LSB index
        frame.data[3] = 0x00; // MSB index

        // Send the CAN frame
        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            perror("Write");
            return 1;
        }
    }

    return 0;
}
