#include <elmo_can.h>
#include <math.h>
#include <rd-kits/keyboard_input.h>

#define DEG2RAD *0.017452925
#define RAD2DEG *57.295780
#define RAD2CNTS (18000.0f * 0.5 / M_PI)
#define MOTOR_RADIUS 6.3

#define KP 0.0075 // 0.005
#define KI 0.0001 // 0.00042
#define MAX_I 700
#define MAX_O 1000

int32_t enc_now[6] = {0, 0, 0, 0, 0, 0};

int32_t enc_now_left = 0, enc_now_right = 0, enc_now_rear = 0;
float vel_x_robot, vel_y_robot, vel_theta_robot;
float vel_left_motor = 0, vel_right_motor = 0, vel_rear_motor = 0;
int32_t target_torque_left = 0, target_torque_right = 0, target_torque_rear = 0;

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

int32_t set_velocity_left()
{
    static float error = 0;
    static float integral = 0;
    static float output = 0;

    error = vel_left_motor - enc_now_left;

    integral += error * KI;

    if (integral > MAX_I)
        integral = MAX_I;
    else if (integral < -MAX_I)
        integral = -MAX_I;

    // printf("%.2f %.2f %.2f\n", error, integral, output);

    output = KP * error + integral;

    if (output > MAX_O)
        output = MAX_O;
    else if (output < -MAX_O)
        output = -MAX_O;

    return (int32_t)output;
}

int32_t set_velocity_right()
{
    static float error = 0;
    static float integral = 0;
    static float output = 0;

    error = vel_right_motor - enc_now_right;

    integral += error * KI;

    if (integral > MAX_I)
        integral = MAX_I;
    else if (integral < -MAX_I)
        integral = -MAX_I;

    // printf("%.2f %.2f %.2f\n", error, integral, output);

    output = KP * error + integral;

    if (output > MAX_O)
        output = MAX_O;
    else if (output < -MAX_O)
        output = -MAX_O;

    return (int32_t)output;
}

int32_t set_velocity_rear()
{
    static float error = 0;
    static float integral = 0;
    static float output = 0;

    error = vel_rear_motor - enc_now_rear;

    integral += error * KI;

    if (integral > MAX_I)
        integral = MAX_I;
    else if (integral < -MAX_I)
        integral = -MAX_I;

    // printf("%.2f %.2f %.2f\n", error, integral, output);

    output = KP * error + integral;

    if (output > MAX_O)
        output = MAX_O;
    else if (output < -MAX_O)
        output = -MAX_O;

    return (int32_t)output;
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
                int8_t stts0 = elmo_init_motor(s, node_id[0], ELMO_TORQUE_MODE);
                int8_t stts1 = elmo_init_motor(s, node_id[1], ELMO_TORQUE_MODE);
                int8_t stts2 = elmo_init_motor(s, node_id[2], ELMO_TORQUE_MODE);
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

            case 't':
                elmo_can_write(s, node_id[0], ELMO_TORQUE_SLOPE, 0, 2001);
                elmo_can_write(s, node_id[1], ELMO_TORQUE_SLOPE, 0, 2001);
                elmo_can_write(s, node_id[2], ELMO_TORQUE_SLOPE, 0, 2001);
                break;

            case 'x':
            {
                // uint32_t stts = elmo_can_read_req(s, node_id[0], 0x607F, 0);
                // elmo_can_read(s, node_id[0], stts, 1000);
                elmo_can_write(s, node_id[0], 0x607F, 0, (int32_t)40000000);
                // printf("ER603f: %d\n", stts);
                break;
            }

            case 'c':
            {
                uint16_t stts = elmo_can_read_req(s, node_id[0], ELMO_ERROR_CODE_402, 0);
                elmo_can_read(s, node_id[0], stts, 1000);
                printf("ER603f: %x\n", stts);
                break;
            }

            case 'v':
            {
                uint16_t stts = elmo_can_read_req(s, node_id[1], ELMO_ERROR_CODE_402, 0);
                elmo_can_read(s, node_id[1], stts, 1000);
                printf("ER603f: %x\n", stts);
                break;
            }

            case 'b':
            {
                uint16_t stts = elmo_can_read_req(s, node_id[2], ELMO_ERROR_CODE_402, 0);
                elmo_can_read(s, node_id[2], stts, 1000);
                printf("ER603f: %x\n", stts);
                break;
            }

            case ' ':
            {
                // elmo_can_set_target_torque(s, node_id[0], (int16_t)0);
                // elmo_can_set_target_torque(s, node_id[1], (int16_t)0);
                // elmo_can_set_target_torque(s, node_id[2], (int16_t)0);
                vel_y_robot = 0;
                vel_x_robot = 0;
                vel_theta_robot = 0;

                break;
            }

            case 'y':
                elmo_can_set_target_torque(s, node_id[0], (int16_t)40);
                elmo_can_set_target_torque(s, node_id[1], (int16_t)40);
                elmo_can_set_target_torque(s, node_id[2], (int16_t)40);
                break;

            case 'o':
            {
                vel_y_robot = 0;
                vel_x_robot = 0;
                vel_theta_robot = 160;
                break;
            }

            case 'p':
            {
                vel_y_robot = 0;
                vel_x_robot = 0;
                vel_theta_robot = -160;
                break;
            }

            case 'w':
            {
                vel_y_robot = 160;
                vel_x_robot = 0;
                vel_theta_robot = 0;
                break;
            }

            case 'a':
            {
                vel_y_robot = 0;
                vel_x_robot = -160;
                vel_theta_robot = 0;
                break;
            }

            case 's':
            {
                vel_y_robot = -160;
                vel_x_robot = 0;
                vel_theta_robot = 0;
                break;
            }

            case 'd':
            {
                vel_y_robot = 0;
                vel_x_robot = 160;
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

        elmo_can_clear_recv_buffer(s);

        elmo_can_read_req(s, node_id[0], ELMO_ACTUAL_VELOCITY, 0x00);
        elmo_can_read_req(s, node_id[1], ELMO_ACTUAL_VELOCITY, 0x00);
        elmo_can_read_req(s, node_id[2], ELMO_ACTUAL_VELOCITY, 0x00);

        // Ensure that encoder is always received
        {
            uint8_t cntr_ditemukan = 0;
            uint32_t cntr_jumlah = 0;
            node_id_sword[0] = 0;
            node_id_sword[1] = 0;
            node_id_sword[2] = 0;
            while (cntr_ditemukan < 3)
            {
                // printf("hehe: %d %d\n", cntr_jumlah, cntr_ditemukan);
                if (cntr_jumlah > 20)
                {
                    printf("Timeout\n");
                    break;
                }

                // Read the CAN frame
                struct can_frame frame;
                fd_set read_fds;
                struct timeval timeout;
                int retval;

                // Set up the file descriptor set
                FD_ZERO(&read_fds);
                FD_SET(s, &read_fds);

                // Set timeout values
                timeout.tv_sec = 0;
                timeout.tv_usec = 100000;

                // Wait for data to be available on the set
                retval = select(s + 1, &read_fds, NULL, NULL, &timeout);

                if (retval == -1 || retval == 0)
                {
                    cntr_jumlah += 9;
                    continue;
                }

                if (read(s, &frame, sizeof(struct can_frame)) < 0)
                {
                    perror("Read");
                }

                // Check if the response is from the expected node
                if (frame.can_id == ELMO_COBID_SDO_RESPONSE + node_id[0])
                {
                    node_id_sword[0] = 99;
                    enc_now_left = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
                    cntr_ditemukan++;
                }
                else if (frame.can_id == ELMO_COBID_SDO_RESPONSE + node_id[1])
                {
                    node_id_sword[1] = 99;
                    enc_now_right = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
                    cntr_ditemukan++;
                }
                else if (frame.can_id == ELMO_COBID_SDO_RESPONSE + node_id[2])
                {
                    node_id_sword[2] = 99;
                    enc_now_rear = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
                    cntr_ditemukan++;
                }
                cntr_jumlah++;
            }
        }

        if (node_id_sword[0] > 0)
        {

            target_torque_left = set_velocity_left();
            elmo_can_clear_write_buffer(s);
            elmo_can_set_target_torque(s, node_id[0], target_torque_left);
        }
        if (node_id_sword[1] > 0)
        {

            target_torque_right = set_velocity_right();
            elmo_can_clear_write_buffer(s);
            elmo_can_set_target_torque(s, node_id[1], target_torque_right);
        }
        if (node_id_sword[2] > 0)
        {

            target_torque_rear = set_velocity_rear();
            elmo_can_clear_write_buffer(s);
            elmo_can_set_target_torque(s, node_id[2], target_torque_rear);
        }

        usleep(10000); // 100 hz (WTF RMS lite?)
    }
    return 0;
}