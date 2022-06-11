/**
 * GCM Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#ifndef GCM_H
#define GCM_H

#include "../common/CAN.h"
#include "../common/FSAE_adc.h"
#include "../common/FSAE_can.h"
#include "../common/FSAE_config.h"
#include "../common/errno.h"
#include <sys/types.h>

/* Types */

// Enum for interval override control
typedef enum { NO_OVERRIDE = 0, OVERRIDE = 1 } override_t;
// Generic enum for defining shift up/down
typedef enum { SHIFT_UP, SHIFT_DOWN } shift_direction_t;
// Enum for defining GCM mode
typedef enum { NORMAL = 0, AUTO_UPSHIFTING = 1 } gcm_mode_t;

/* Gear Thresholds */

// ECU ADC SEES DIFFERENT VOLTAGES FOR SOME REASON
// TUNE THESE CUTOFFS INDEPENDENTLY FOR BOTH DEVICES

// These voltage thresholds are coded as [low, high] pairs for each gear
#define N_GEAR_VOLTAGE_THRESHOLDS 7

const double gear_voltage_low_thresholds[N_GEAR_VOLTAGE_THRESHOLDS] = {
    0.99, 0, 1.375, 1.95, 2.6925, 3.5225, 4.09};

const double gear_voltage_high_thresholds[N_GEAR_VOLTAGE_THRESHOLDS] = {
    1.375, 0.99, 1.95, 2.6925, 3.5225, 4.09, 6};

#define GEAR_NEUT 0
#define GEAR_FAIL 7

// Gear ratio of standard Yamaha R6 YZF 08 transmission
const double gear_ratio[7] = {1.0, 2.583, 2.000, 1.667, 1.444, 1.286, 1.150};

/* Sample timing constants (ms) */

// Debounce time for paddle inputs
#define SHIFT_DEBOUNCE_INTERVAL 10
// Sample time for internal temperature sensors
#define TEMP_SAMPLE_INTERVAL 5000
// Sample time for gear position sensor
#define GEAR_SAMPLE_INTERVAL 5

/* CAN timing constants */

// Interval for sending diagnostic CAN message
#define DIAG_CAN_SEND_INTERVAL 5000
// Interval for sending state CAN message
#define STATE_CAN_SEND_INTERVAL 20
// Interval for sending ECU gearcut CAN message
#define MS6_GEARCUT_CAN_SEND_INTERVAL 1
// Interval for sending autoupshifting state CAN message
#define AUTOUPSHIFTING_STATUS_CAN_SEND_INTERVAL 20

// Interval for diagnosing stale CAN bus
#define CAN_STALE_INTERVAL 10000
#define CAN_MODE_MSG_STALE_INTERVAL 1000

/* Shift control loop constants */

#define PADDLE_LOCKOUT_DURATION 25
#define MAX_SHIFT_DURATION 300
#define RELAX_WAIT_DURATION 10
#define AUTOUPSHIFTING_LOCKOUT_DURATION 100

// Comment to disable sending gearcut message over CAN
// #define SEND_GEARCUT_CAN

/* Paddle switches */

// Debounced shift paddle states, unpacked
// (use these for everything)
#define SHIFT_UP_SW (((shift_debounced_packed >> 2) & 0x1))
#define SHIFT_DN_SW (((shift_debounced_packed >> 1) & 0x1))
#define SHIFT_NT_SW 0

/* Ignition cut flags */

#define IGNITION_CUT_ENABLE 1
#define IGNITION_CUT_DISABLE 0

/* CAN IDs */

// Sending
#define DIAG_MESSAGE_CAN_ID 0x200
#define GEAR_MODE_MESSAGE_CAN_ID 0x201
#define QUEUE_STATE_MESSAGE_CAN_ID 0x202
#define AUTOUPSHIFTING_STATUS_CAN_ID 0x203
#define MS6_GEARCUT_CAN_ID 0x610

// Receiving
#define ECU_AUTOUPSHIFTING_1_ID 0x3FB
#define ECU_AUTOUPSHIFTING_2_ID 0x3FC
#define ECU_AUTOUPSHIFTING_TARGETS_1 0x250
#define ECU_AUTOUPSHIFTING_TARGETS_2 0x251
#define ECU_AUTOUPSHIFTING_TARGETS_3 0x252

/* Pin definitions */

#define SHIFT_NT_TRIS TRISEbits.TRISE5
#define SHIFT_NT_ANSEL ANSELEbits.ANSE5
#define SHIFT_NT_PORT PORTEbits.RE5
#define SHIFT_UP_TRIS TRISGbits.TRISG15
#define SHIFT_UP_ANSEL ANSELGbits.ANSG15
#define SHIFT_UP_PORT PORTGbits.RG15
#define SHIFT_DN_TRIS TRISAbits.TRISA5
#define SHIFT_DN_ANSEL ANSELAbits.ANSA5
#define SHIFT_DN_PORT PORTAbits.RA5

#define ACT_UP_TRIS TRISEbits.TRISE7
#define ACT_UP_LAT LATEbits.LATE7
#define ACT_DN_TRIS TRISCbits.TRISC1
#define ACT_DN_LAT LATCbits.LATC1

#define ADC_GEAR_TRIS TRISGbits.TRISG6
#define ADC_GEAR_ANSEL ANSELGbits.ANSG6
#define ADC_GEAR_CSS ADCCSS1bits.CSS14
#define ADC_GEAR_CHN 14

#define ADC_FORCE_TRIS TRISEbits.TRISE8
#define ADC_FORCE_ANSEL ANSELEbits.ANSE8
#define ADC_FORCE_CSS ADCCSS1bits.CSS25
#define ADC_FORCE_CHN 25

#define ACTUATOR_ON 0
#define ACTUATOR_OFF 1

/* Function definitions */

void init(void);
void main(void);

// ADC sampling
void sample_temp(void);
void sample_gear_position(void);

// CAN receiving/sending
void process_CAN_messages(CAN_message);
void send_diag_can(void);
void send_state_can(override_t);
void send_ignition_cut_status_can(override_t);
void send_autoupshifting_status_can();

// Logic functions
void process_paddle_press(shift_direction_t);
uint8_t check_shift_conditions(shift_direction_t);

uint16_t get_target_upshift_rpm(void);
double get_target_upshift_speed(void);
void process_auto_upshift(void);

void do_shift(shift_direction_t);

// Utility functions
void update_gcm_mode(void);
void relax_wait(void);
void main_loop_misc(void);
void debounce_shift_inputs(void);
#define abs(n) (((n) >= 0) ? (n) : -(n))

#endif