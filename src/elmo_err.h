#ifndef ELMO_ERR_H
#define ELMO_ERR_H

/*
Based on ELMO DS-301
Karena web nya gratis dan harus scroll kebawah, jadi saya copy paste disini
*/

#define ELMO_MOTOR_FAULT_RESOLVER_ENCODER_FEEDBACK_FAILED 0x7300
#define ELMO_MOTOR_FAULT_RESERVED_1 0x7305
#define ELMO_MOTOR_FAULT_RESERVED_2 0x7306
#define ELMO_MOTOR_FAULT_FEEDBACK_LOSS 0x7380
#define ELMO_MOTOR_FAULT_PEAK_CURRENT_EXCEEDED 0x8311
#define ELMO_MOTOR_FAULT_DISABLED_BY_LIMIT_SWITCH 0x5441
#define ELMO_MOTOR_FAULT_ECAM_TABLE_PROBLEM 0x5280
#define ELMO_MOTOR_FAULT_TWO_DIGITAL_HALL_SENSORS_CHANGED_AT_ONCE 0x7381
#define ELMO_MOTOR_FAULT_SPEED_TRACKING_ERROR 0x8480
#define ELMO_MOTOR_FAULT_POSITION_TRACKING_ERROR 0x8611
#define ELMO_MOTOR_FAULT_CANNOT_START_DUE_TO_INCONSISTENT_DATABASE 0x6320
#define ELMO_MOTOR_FAULT_TOO_LARGE_A_DIFFERENCE_IN_ECAM_TABLE_ENTRIES 0x5280
#define ELMO_MOTOR_FAULT_HEARTBEAT_FAILURE 0x8130
#define ELMO_MOTOR_FAULT_CANNOT_FIND_ELECTRICAL_ZERO_OF_MOTOR 0x8380
#define ELMO_MOTOR_FAULT_SPEED_LIMIT_EXCEEDED 0x8481
#define ELMO_MOTOR_FAULT_STACK_OVERFLOW 0x6180
#define ELMO_MOTOR_FAULT_CPU_EXCEPTION 0x6181
#define ELMO_MOTOR_FAULT_TIMING_ERROR 0x5281
#define ELMO_MOTOR_FAULT_MOTOR_STUCK 0x7121
#define ELMO_MOTOR_FAULT_POSITION_LIMIT_EXCEEDED 0x8680
#define ELMO_MOTOR_FAULT_RESERVED_1 0x1000
#define ELMO_MOTOR_FAULT_CANNOT_TUNE_CURRENT_OFFSETS 0x8381
#define ELMO_MOTOR_FAULT_CANNOT_START_MOTOR 0xFF10
#define ELMO_MOTOR_FAULT_RESERVED_2 0x8680
#define ELMO_MOTOR_FAULT_RESERVED_3 0x8312
#define ELMO_MOTOR_FAULT_UNDER_VOLTAGE 0x3120
#define ELMO_MOTOR_FAULT_OVER_VOLTAGE 0x3310
#define ELMO_MOTOR_FAULT_RESERVED_4 0x3100
#define ELMO_MOTOR_FAULT_RESERVED_5 0x2311
#define ELMO_MOTOR_FAULT_SHORT_CIRCUIT 0x2340
#define ELMO_MOTOR_FAULT_TEMPERATURE 0x4310
#define ELMO_MOTOR_FAULT_RESERVED_6 0x5282

#endif // ELMO_ERR_H