/**
 *
 * ELMO DS-301                   https://www.collegesidekick.com/study-docs/2385622
 * ELMO DS-401                   https://ia601707.us.archive.org/34/items/manualsonline-id-e996ba7a-35f3-48b7-a31a-28b5ef8c735b/e996ba7a-35f3-48b7-a31a-28b5ef8c735b.pdf
 * ELMO BASIC SIMPLIQ COMMAND    https://www.pk-rus.ru/fileadmin/download/simpleiq_command_referense_manual.pdf
 * ELMO DRIVER                   https://www.elmomc.com/product/solo-guitar/
 */

#ifndef ELMO_CAN_H
#define ELMO_CAN_H

#include <cstring>
#include <cstdio>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "elmo_reg.h"

#define ELMO_POSITION_MODE 0x01
#define ELMO_VELOCITY_MODE 0x03
#define ELMO_TORQUE_MODE 0x04
#define ELMO_HOMING_MODE 0x06

/* Tools */
int8_t elmo_can_ping_all(int8_t s, int8_t *nodes, uint16_t timeout_ms = 1000);
int8_t elmo_can_ping(int8_t s, uint8_t node_id, uint16_t timeout_ms = 1000);

/* NMT */
int8_t elmo_can_send_NMT(int8_t s, uint8_t command, uint8_t node_id);
int8_t elmo_can_check_NMT(int8_t s, uint8_t node_id);
int8_t elmo_can_set_heartbeat(int8_t s, uint8_t node_id, uint16_t ms);
uint8_t elmo_can_handle_heartbeat(const struct can_frame *frame);

/* MISC */
int8_t elmo_can_clear_recv_buffer(int8_t s);

/* CAN */
int8_t elmo_can_init(const char *interface);

/* SDO READ */
int8_t elmo_can_read_req(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex);
int8_t elmo_can_read(int8_t s, int8_t node_id, uint32_t &value, uint16_t timeout_ms);
int8_t elmo_can_read(int8_t s, int8_t node_id, int32_t &value, uint16_t timeout_ms);
int8_t elmo_can_read(int8_t s, int8_t node_id, uint16_t &value, uint16_t timeout_ms);
int8_t elmo_can_read(int8_t s, int8_t node_id, int16_t &value, uint16_t timeout_ms);
int8_t elmo_can_read(int8_t s, int8_t node_id, uint8_t &value, uint16_t timeout_ms);
int8_t elmo_can_read(int8_t s, int8_t node_id, int8_t &value, uint16_t timeout_ms);

/* SDO WRITE */
int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, uint32_t value);
int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, int32_t value);
int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, uint16_t value);
int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, int16_t value);
int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, uint8_t value);
int8_t elmo_can_write(int8_t s, int8_t node_id, uint16_t index, uint8_t subindex, int8_t value);

/* High-Level API */
//==============================================================================

int8_t elmo_can_send_BG_DLC4(int8_t s, int8_t node_id);
int8_t elmo_init_motor(int8_t s, int8_t node_id, uint8_t mode = ELMO_VELOCITY_MODE);
int8_t elmo_can_set_mode_op(int8_t s, int8_t node_id, int8_t mode_op);
int8_t elmo_can_send_config_obj(int8_t s, int8_t node_id, uint32_t conf_obj);
int8_t elmo_can_ignore_ls(int8_t s, int8_t node_id); // Ignore limit switch

int8_t elmo_can_set_quick_stop(int8_t s, int8_t node_id, int16_t quick_stop);
int8_t elmo_can_set_c_word(int8_t s, int8_t node_id, uint16_t c_word);
uint16_t elmo_can_set_s_word(int8_t s, int8_t node_id);

/* Postion Mode */
int8_t elmo_can_set_target_position(int8_t s, int8_t node_id, uint32_t target_pos);
int32_t elmo_can_get_enc_px(int8_t s, int8_t node_id);

/* Velocity Mode */
int8_t elmo_can_set_target_velocity(int8_t s, int8_t node_id, uint32_t target_vel);
int32_t elmo_can_get_enc_vx(int8_t s, int8_t node_id);

/* Torque Mode */
int8_t elmo_can_set_target_torque(int8_t s, int8_t node_id, int16_t torque);
uint32_t elmo_can_get_motor_rate_current(int8_t s, int8_t node_id);

#endif // ELMO_CAN_H