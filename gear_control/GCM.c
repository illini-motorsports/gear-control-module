/**
 * GCM
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "GCM.h"

/* Global variables */

// Internal (milli)second timers
volatile uint32_t seconds = 0;
volatile uint32_t millis = 0;

// Mode
volatile gcm_mode_t mode = NORMAL;

// Gear
volatile double gear_voltage = -1.0;
volatile uint8_t gear = GEAR_FAIL;

// Internal temperatures
volatile int16_t pcb_temp = 0;
volatile int16_t junc_temp = 0;

// Shift force
volatile double shift_force = 0.0;

// Paddles
volatile uint8_t shift_debounced_packed = 0;
volatile uint8_t prev_switch_up = 0;
volatile uint8_t prev_switch_dn = 0;

// Shift control loop
volatile uint8_t queue_up = 0;
volatile uint8_t queue_dn = 0;
volatile uint8_t queue_nt = 0;

// Auto-upshifting
volatile uint8_t autoupshift_switch = 0;

volatile double wheel_speed = 0.0;
volatile int16_t eng_rpm = 0;

// Shift RPMs for 1-->2, 2-->3, 3-->4, 4-->5, 5-->6, 6-->???
// These can be set via CAN
volatile uint16_t upshift_target_rpms[6] = {20001, 20002, 20003,
                                            20004, 20005, 20006};

// Shift speeds for 1-->2, 2-->3, 3-->4, 4-->5, 5-->6, 6-->???
// These can be set via CAN
volatile double upshift_target_speeds[6] = {101, 102, 103, 104, 105};

// Shift RPM and speed delay adjustments
// These can be set via CAN
volatile uint8_t eng_rpm_delay_adjustment = 0;
volatile float wheel_speed_delay_adjustment = 0.0;

// Ignition cut flags
volatile uint8_t ignition_cut_upshift = IGNITION_CUT_DISABLE;
volatile uint8_t ignition_cut_downshift = IGNITION_CUT_DISABLE;

// Timers
volatile uint32_t paddle_debounce_timer = 0;
volatile uint32_t paddle_lockout_timer = 0;

volatile uint32_t temp_sample_timer = 0;
volatile uint32_t gear_sample_timer = 0;

volatile uint32_t actuator_fire_timer = 0;

volatile uint32_t autoupshifting_lockout_timer = 0;

volatile uint32_t CAN_mode_message_receive_timer = 0;
volatile uint32_t ignition_cut_can_send_timer = 0;
volatile uint32_t diag_can_send_timer = 0;
volatile uint32_t state_can_send_timer = 0;
volatile uint32_t autoupshifting_status_can_send_timer = 0;

/* Main Functions */

/**
 * Initializes SoC, peripheral, inputs, and interrupts
 */
void init(void) {
  // Set general runtime configuration bits
  init_general();
  // Set all I/O pins to low outputs
  init_gpio_pins();
  // Disable unused peripheral modules
  // init_peripheral_modules();
  // Initialize oscillator configuration bits
  init_oscillator(0);
  // Initialize timer2 (millis)
  init_timer2();
  // Initialize ADC module
  init_adc(NULL);
  // Initialize programmable CAN termination
  init_termination(NOT_TERMINATING);
  // Initialize CAN
  init_can();

  // Initialize input pins
  SHIFT_UP_TRIS = INPUT;
  SHIFT_UP_ANSEL = DIG_INPUT;
  SHIFT_DN_TRIS = INPUT;
  SHIFT_DN_ANSEL = DIG_INPUT;
  SHIFT_NT_TRIS = INPUT;
  SHIFT_NT_ANSEL = DIG_INPUT;
  ADC_GEAR_ANSEL = AN_INPUT;
  ADC_GEAR_CSS = 1;
  ADC_FORCE_TRIS = INPUT;
  ADC_FORCE_ANSEL = AN_INPUT;
  ADC_FORCE_CSS = 1;

  // Initialize output pins
  ACT_UP_TRIS = OUTPUT;
  ACT_UP_LAT = ACTUATOR_OFF;
  ACT_DN_TRIS = OUTPUT;
  ACT_DN_LAT = ACTUATOR_OFF;
  ADC_GEAR_TRIS = INPUT;

  // Trigger initial ADC conversion
  ADCCON3bits.GSWTRG = 1;

  // Enable interrupts
  STI();
}

/**
 * Main function
 */
void main(void) {
  // Initialize
  init();

  // Superloop
  while (1) {
    // Enable interrupts (in case anything disabled them)
    STI();

    // Debounce shift inputs
    debounce_shift_inputs();

    // Sample internal temperature
    sample_temp();

    // Sample gear position
    sample_gear_position();

    // Perform shifts if they are queued
    if (queue_up > 0)
      do_shift(SHIFT_UP);

    if (queue_dn > 0)
      do_shift(SHIFT_DOWN);

    // Update GCM mode if needed
    update_gcm_mode();

    // Process auto-upshift if needed
    process_auto_upshift();

    // Send state CAN messages if intervals have passed
    send_state_can(NO_OVERRIDE);
    send_ignition_cut_status_can(NO_OVERRIDE);
    send_diag_can();
    send_autoupshifting_status_can();
  }
}

//=========================== INTERRUPT HANDLERS ===============================

/**
 * CAN Interrupt Handler
 */
void __attribute__((vector(_CAN1_VECTOR), interrupt(IPL4SRS)))
can_inthnd(void) {
  // Process all available CAN messages
  if (C1INTbits.RBIF) {
    CAN_recv_messages(process_CAN_messages);
  }

  // CAN overflow error
  if (C1INTbits.RBOVIF) {
    CAN_rx_ovf++;
  }

  // Clear CAN1 Interrupt Flag
  IFS4CLR = _IFS4_CAN1IF_MASK;
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 *
 * Checks for and handles shifting-related inputs.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL5SRS)))
timer2_inthnd(void) {

  // Update timing counters
  ++millis;
  if (millis % 1000 == 0)
    ++seconds;

  // Trigger an ADC conversion if a new sample is ready
  if (ADCCON2bits.EOSRDY)
    ADCCON3bits.GSWTRG = 1;

  // Check if the debounced paddle up switch status has changed
  if (SHIFT_UP_SW && !prev_switch_up) {
    process_paddle_press(SHIFT_UP);
  }
  prev_switch_up = SHIFT_UP_SW;

  // Check if the debounced paddle down switch status has changed
  if (SHIFT_DN_SW && !prev_switch_dn) {
    process_paddle_press(SHIFT_DOWN);
  }
  prev_switch_dn = SHIFT_DN_SW;

  // Clear TMR2 Interrupt Flag
  IFS0CLR = _IFS0_T2IF_MASK;
}

/**
 * NMI Handler
 *
 * This interrupt handler will reset the device when a clock failure occurs.
 */
void _nmi_handler(void) {
  // Perform a software reset
  unlock_config();
  RSWRSTSET = 1;
  uint16_t dummy = RSWRST;
  while (1)
    ;
  asm volatile("eret;"); // Should never be called
}

//============================= ADC FUNCTIONS ==================================

/**
 * Samples the PCB temp sensor and internal die temp sensor, then updates
 * variables if the interval has passed.
 */
void sample_temp(void) {
  if (millis - temp_sample_timer >= TEMP_SAMPLE_INTERVAL) {

    /**
     * PCB Temp [C] = (Sample [V] - 0.75 [V]) / 10 [mV/C]
     * PCB Temp [C] = ((3.3 * (pcb_temp_samp / 4095)) [V] - 0.75 [V]) / 0.01
     * [V/C] PCB Temp [C] = (3.3 * (pcb_temp_samp / 40.95)) - 75) [C] PCB Temp
     * [C] = (pcb_temp_samp * 0.080586080586) - 75 [C] PCB Temp [C / 0.005] =
     * 200 * ((pcb_temp_samp * 0.080586080586) - 75) [C / 0.005] PCB Temp [C /
     * 0.005] = (temp_samp * 16.1172161172) - 15000 [C / 0.005]
     */
    uint32_t pcb_temp_samp = read_adc_chn(ADC_PTEMP_CHN);
    pcb_temp = (((double)pcb_temp_samp) * 16.1172161172) - 15000.0;

    /**
     * Junc Temp [C] = 200 [C/V] * (1 [V] - Sample [V])
     * Junc Temp [C] = 200 [C/V] * (1 - (3.3 * (junc_temp_samp / 4095))) [V]
     * Junc Temp [C] = 200 [C/V] * (1 - (junc_temp_samp / 1240.9090909)) [V]
     * Junc Temp [C] = 200 - (junc_temp_samp * 0.161172161172) [C]
     * Junc Temp [C / 0.005] = 40000 - (junc_temp_samp * 32.234432234432) [C /
     * 0.005]
     */
    uint32_t junc_temp_samp = read_adc_chn(ADC_JTEMP_CHN);
    junc_temp =
        (int16_t)(40000.0 - (((double)junc_temp_samp) * 32.234432234432));

    temp_sample_timer = millis;
  }
}

/**
 * Samples the gear position sensor and updates variables if the interval has
 * passed.
 */
void sample_gear_position(void) {
  if (millis - gear_sample_timer >= GEAR_SAMPLE_INTERVAL) {

    // Read ADC
    uint32_t gear_samp = read_adc_chn(ADC_GEAR_CHN);

    // Convert ADC reading to raw voltage
    gear_voltage = ((((double)gear_samp) / 4095.0) * 5.0);

    // Compute actual gear position from thresholds
    for (uint8_t i = 0; i < N_GEAR_VOLTAGE_THRESHOLDS; i++) {
      double low = gear_voltage_low_thresholds[i];
      double high = gear_voltage_high_thresholds[i];
      if (gear_voltage > low && gear_voltage <= high) {
        gear = i;
        break;
      }
    }

    gear_sample_timer = millis;
  }
}

//============================= CAN FUNCTIONS ==================================

/**
 * Handler function for each received CAN message.
 *
 * @param msg The received CAN message
 */
void process_CAN_messages(CAN_message msg) {

  switch (msg.id) {

  case ECU_AUTOUPSHIFTING_1_ID:
    wheel_speed =
        ((double)((((uint32_t)msg.data[1]) << 8) | msg.data[0])) * 0.01;

    eng_rpm =
        (int16_t)(((double)((((uint32_t)msg.data[3]) << 8) | msg.data[2])) *
                  1.0);

    break;

  case ECU_AUTOUPSHIFTING_2_ID:

    autoupshift_switch = msg.data[0];

    CAN_mode_message_receive_timer = millis;

    break;

  case ECU_AUTOUPSHIFTING_TARGETS_1:
    for (uint8_t i = 0; i < 4; i++) {
      upshift_target_rpms[i] =
          (int16_t)((((uint32_t)msg.data[(i * 2) + 1]) << 8) | msg.data[i * 2]);
    }

    break;

  case ECU_AUTOUPSHIFTING_TARGETS_2:
    for (uint8_t i = 0; i < 4; i++) {
      upshift_target_speeds[i] =
          ((double)((((uint32_t)msg.data[(i * 2) + 1]) << 8) |
                    msg.data[i * 2])) *
          0.01;
    }

    break;

  case ECU_AUTOUPSHIFTING_TARGETS_3:
    upshift_target_rpms[4] =
        (int16_t)((((uint32_t)msg.data[1]) << 8) | msg.data[0]);

    upshift_target_speeds[4] =
        ((double)((((uint32_t)msg.data[3]) << 8) | msg.data[2])) * 0.01;

    eng_rpm_delay_adjustment = msg.data[4];
    wheel_speed_delay_adjustment = ((float)msg.data[5]) * 0.01;

    break;

  default:
    break;
  }
}

/**
 * Sends the diagnostic CAN message if the interval has passed.
 */
void send_diag_can(void) {
  static CAN_data data = {0};

  if (millis - diag_can_send_timer >= DIAG_CAN_SEND_INTERVAL) {
    // Uptime
    data.halfword0 = (uint16_t)seconds;
    // Temperatures
    data.halfword1 = pcb_temp;
    data.halfword2 = junc_temp;

    CAN_send_message(DIAG_MESSAGE_CAN_ID, 6, data);

    diag_can_send_timer = millis;
  }
}

/**
 * Sends the switch, queue, and gear state CAN messages if the interval has
 * passed.
 * @param override - Whether to override the interval
 */
void send_state_can(override_t override) {
  static CAN_data data = {0};
  if (override || (millis - state_can_send_timer >= STATE_CAN_SEND_INTERVAL)) {

    // Bits for up, down, neutral shift paddles
    data.byte0 = 0x0 | (SHIFT_NT_SW << 2) | (SHIFT_DN_SW << 1) | SHIFT_UP_SW;
    // Queue up, down, neutral lengths
    data.byte1 = queue_up;
    data.byte2 = queue_dn;
    data.byte3 = queue_nt;
    CAN_send_message(QUEUE_STATE_MESSAGE_CAN_ID, 4, data);

    uint16_t gear_voltage_can = (uint16_t)(gear_voltage * 10000.0);
    // Gear
    data.byte0 = gear;
    // Mode (0 = normal, 1 = auto-upshifting)
    data.byte1 = mode;
    // Raw gear sensor voltage
    data.halfword1 = gear_voltage_can;
    // Raw shift force voltage
    data.halfword2 = 0;
    CAN_send_message(GEAR_MODE_MESSAGE_CAN_ID, 6, data);

    state_can_send_timer = millis;
  }
}

/**
 * Sends the ignition cut status to the ECU
 * @param override - Whether to override the interval
 */
void send_ignition_cut_status_can(override_t override) {
#ifdef SEND_GEARCUT_CAN
  // Shift signals were moved to analog as of 5/31/2022,
  // so we no longer need to send this message to the ECU.

  static CAN_data data = {0};
  if (override ||
      millis - ignition_cut_can_send_timer >= MS6_GEARCUT_CAN_SEND_INTERVAL) {
    data.doubleword = (ignition_cut_upshift << 1) | ignition_cut_downshift;
    CAN_send_message(MS6_GEARCUT_CAN_ID, 8, data);
    ignition_cut_can_send_timer = millis;
  }
#endif
}

void send_autoupshifting_status_can() {
  static CAN_data data = {0};
  if (millis - autoupshifting_status_can_send_timer >=
      AUTOUPSHIFTING_STATUS_CAN_SEND_INTERVAL) {
    data.halfword0 = eng_rpm;
    data.halfword1 = get_target_upshift_rpm();
    data.byte4 = (uint8_t)wheel_speed;
    data.byte5 = (uint8_t)get_target_upshift_speed();

    CAN_send_message(AUTOUPSHIFTING_STATUS_CAN_ID, 8, data);

    autoupshifting_status_can_send_timer = millis;
  }
}

//=========================== LOGIC FUNCTIONS ==================================

/**
 * Process a (post-debounce) paddle press and increment the corresponding queue
 * counter.
 * @param direction the direction of the shift paddle that was pressed
 */
void process_paddle_press(shift_direction_t direction) {
  // If we're still within the lockout interval, do nothing
  if (millis - paddle_lockout_timer < PADDLE_LOCKOUT_DURATION) {
    // TODO: send a CAN message
    return;
  }

  if (direction == SHIFT_UP) {
    // If downshifts were queued, cancel everything
    if (queue_dn > 0) {
      queue_up = 0;
      queue_dn = 0;
      queue_nt = 0;
    }
    // Otherwise increment the queue counter
    else {
      queue_up++;
      paddle_lockout_timer = millis;
    }
  } else {
    // If upshifts were queued, cancel everything
    if (queue_up > 0) {
      queue_up = 0;
      queue_dn = 0;
      queue_nt = 0;
    }
    // Otherwise increment the queue counter
    else {
      queue_dn++;
      paddle_lockout_timer = millis;
    }
  }
}

/**
 * Checks various conditions and determines if the requested shift is allowable.
 *
 * @param direction - The direction of shift being requested
 * @return 0 if restricted, 1 if allowable
 */
uint8_t check_shift_conditions(shift_direction_t direction) {
  return 1;

  /*
  // Prevent shifting past 1st gear
  if (gear == 1 && direction == SHIFT_DOWN)
  {
      return 0;
  }

  // Prevent shifting past 6th gear
  if (gear == 6 && direction == SHIFT_UP)
  {
      return 0;
  }

  // Only perform these checks if CAN data isn't stale
  uint8_t stale_CAN = (millis - CAN_receive_timer >= CAN_STATE_WAIT);
  if (!stale_CAN)
  {
      // Prevent shifting while the kill switch is pressed
      if (kill_sw)
      {
          send_errno_CAN_msg(GCM_ID, ERR_GCM_KILLSW);
          return 0;
      }

      // Prevent shifting from neutral into gear at high rpm
      if (gear == GEAR_NEUT && eng_rpm >= RPM_NEUT_LOCK)
      {
          send_errno_CAN_msg(GCM_ID, ERR_GCM_NEUTLOCK);
          return 0;
      }

      // Check for over/under rev
      if (0)
      {
          double output_speed = eng_rpm / gear_ratio[gear];

          if (shift_enum == SHIFT_ENUM_UP && gear != 6)
          {
              double new_rpm = output_speed * gear_ratio[gear + 1];
              if (new_rpm <= RPM_UNDERREV)
              {
                  send_errno_CAN_msg(GCM_ID, ERR_GCM_UNDERREV);
                  return 0;
              }
          }
          else if (shift_enum == SHIFT_ENUM_DN && gear != 1)
          {
              double new_rpm = output_speed * gear_ratio[gear - 1];
              if (new_rpm >= RPM_OVERREV)
              {
                  send_errno_CAN_msg(GCM_ID, ERR_GCM_OVERREV);
                  return 0;
              }
          }
      }
  }
  else if (!COMP)
  {
      // Prevent shifting if CAN state variables aren't updated
      send_errno_CAN_msg(GCM_ID, ERR_GCM_NOCAN);
      return 0;
  }

  // Only perform these checks if we aren't at competition and have good CAN
  // data
  if (!COMP && !stale_CAN)
  {
      // Prevent shifting up past neutral on low voltage
      if (gear == 1 && shift_enum == SHIFT_ENUM_UP)
      {
          if (bat_volt != 0.0 && bat_volt < LOW_VOLT)
          {
              send_errno_CAN_msg(GCM_ID, ERR_GCM_LOWVOLT);
              return 0;
          }
      }

      // Prevent shifting down past neutral on low voltage
      if (gear == 2 && shift_enum == SHIFT_ENUM_DN)
      {
          if (bat_volt != 0 && bat_volt < LOW_VOLT)
          {
              send_errno_CAN_msg(GCM_ID, ERR_GCM_LOWVOLT);
              return 0;
          }
      }

      // Prevent any shifting on very low voltage
      if (bat_volt != 0 && bat_volt < LOWER_VOLT)
      {
          send_errno_CAN_msg(GCM_ID, ERR_GCM_LOWERVOLT);
          return 0;
      }
  }

  return 1;
  */
}

/**
 * Gets the target up shifting RPM for autoupshifting
 * @param gear current vehicle gear
 * @return target shift RPM for gear, adjusted for delay
 */
int16_t get_target_upshift_rpm(void) {
  if (gear == GEAR_FAIL) {
    return 20000; // If invalid gear, make sure we never shift
  }

  return upshift_target_rpms[gear == GEAR_NEUT ? gear : gear - 1] -
         eng_rpm_delay_adjustment;
}

/**
 * Gets the target up shifting speed for autoupshifting
 * @param gear current vehicle gear
 * @return target speed for gear, adjusted for delay
 */
double get_target_upshift_speed(void) {
  if (gear == GEAR_FAIL) {
    return 200.0; // If invalid gear, make sure we never shift
  }

  return upshift_target_speeds[gear == GEAR_NEUT ? gear : gear - 1] -
         wheel_speed_delay_adjustment;
}

void process_auto_upshift(void) {
  // Exit if auto-upshifting mode is not enabled
  if (mode != AUTO_UPSHIFTING)
    return;

  // Check conditions and queue upshift if they pass
  if (eng_rpm >= get_target_upshift_rpm() &&       // RPM threshold
      wheel_speed >= get_target_upshift_speed() && // Speed threshold
      queue_up == 0 &&                             // Not already shifting
      gear < 6 &&                                  // Not trying to get past 6th
      (millis - autoupshifting_lockout_timer) >=
          AUTOUPSHIFTING_LOCKOUT_DURATION // Didn't just auto upshift

  ) {
    queue_dn = 0;
    queue_up = 1;
    autoupshifting_lockout_timer = millis;
  }
}

/**
 * Performs the shift sequence.
 * @param direction the direction to shift in
 */
void do_shift(shift_direction_t direction) {
  /* Figure out what our target gear is */
  uint8_t gear_target;

  // Gear position is invalid
  if (gear == GEAR_FAIL) {
    gear_target = GEAR_FAIL;
  }
  // Up shift
  else if (direction == SHIFT_UP) {
    gear_target = (gear == GEAR_NEUT) ? 2 : gear + 1;
  }
  // Down shift
  else if (direction == SHIFT_DOWN) {
    gear_target = (gear == GEAR_NEUT) ? 1 : gear - 1;
  }

  /* Check shift conditions */
  // If we can't shift then reset queue and increment the disallowed shift count
  if (!check_shift_conditions(direction)) {
    if (direction == SHIFT_UP) {
      queue_up = 0;
    } else {
      queue_dn = 0;
    }
    // TODO implement disallowed shift count

    return;
  }

  /* Fire Actuator */
  if (direction == SHIFT_UP) {
    ACT_UP_LAT = ACTUATOR_ON;
  } else {
    ACT_DN_LAT = ACTUATOR_ON;
  }

  // Set actuator timer to current time
  actuator_fire_timer = millis;

  // Transmit ignition cut indicator to ECU
  if (direction == SHIFT_UP) {
    ignition_cut_upshift = IGNITION_CUT_ENABLE;
  } else if (SHIFT_DOWN) {
    ignition_cut_downshift = IGNITION_CUT_ENABLE;
  }
  send_ignition_cut_status_can(OVERRIDE);

  /* Shift Loop */
  uint8_t shift_loop_done = 0;
  while (!shift_loop_done) {
    // Sample gear position
    sample_gear_position();

    // Check if we've reached our target gear (assuming no error state), OR
    // we've reached the maximum shift duration
    if ((gear_target != GEAR_FAIL && gear == gear_target) ||
        (millis - actuator_fire_timer >= MAX_SHIFT_DURATION)) {
      shift_loop_done = 1;
    }

    // Also do main loop misc functions while waiting
    main_loop_misc();
  }

  /* Exit Shift Loop */

  // Relax actuator
  if (direction == SHIFT_UP) {
    ACT_UP_LAT = ACTUATOR_OFF;
  } else {
    ACT_DN_LAT = ACTUATOR_OFF;
  }

  // Decrement queued shifts value
  if (direction == SHIFT_UP) {
    queue_up--;
  } else {
    queue_dn--;
  }

  // Relax ignition cut
  ignition_cut_upshift = IGNITION_CUT_DISABLE;
  ignition_cut_downshift = IGNITION_CUT_DISABLE;
  send_ignition_cut_status_can(OVERRIDE);

  // Send new state on CAN
  send_state_can(OVERRIDE);

  // While we wait for the actuator to return back, run misc functions
  relax_wait();
}

//========================== UTILITY FUNCTIONS =================================

/**
 * Update the GCM's mode (normal or auto-upshifting)
 * Makes sure the mode defaults to normal if the CAN bus is stale.
 */
void update_gcm_mode(void) {
  if ((millis - CAN_mode_message_receive_timer) < CAN_MODE_MSG_STALE_INTERVAL) {
    mode = autoupshift_switch ? AUTO_UPSHIFTING : NORMAL;
  } else {
    mode = NORMAL;
  }
}

/**
 * void relax_wait(void)
 *
 * Waits for the actuator to return to the relaxed position
 */
void relax_wait(void) {
  // Wait for RELAX_WAIT_DURATION millis and do main loop functions in the
  // process
  uint32_t start = millis;
  while (millis - start < RELAX_WAIT_DURATION) {
    main_loop_misc();
  }
}

/**
 * void main_loop_misc(void)
 *
 * Do non-shift logic main-loop functions. Typically run
 * while waiting in a blocking section of the shifting logic.
 */
void main_loop_misc(void) {
  debounce_shift_inputs();

  // Sample sensors
  sample_temp();
  sample_gear_position();

  // Send CAN messages
  send_diag_can();
  send_state_can(NO_OVERRIDE);
  send_ignition_cut_status_can(NO_OVERRIDE);
}

/**
 * Keeps track of switch state in order to debounce switch inputs. The inputs
 * are debounced collectively, rather than debouncing each individual switch
 * input separately.
 */
void debounce_shift_inputs(void) {
  // Held shift value
  static volatile uint8_t shift_prev = 0;

  // Packed 8-bit value of all shift switch states
  uint8_t shift_raw = 0x0 | (!SHIFT_UP_PORT) << 2 | (!SHIFT_DN_PORT) << 1 |
                      (!SHIFT_NT_PORT) << 0;

  // Check if the inputs have changed since the last sample
  if (shift_raw != shift_prev) {
    paddle_debounce_timer = millis;
  }
  // If the debounce time has expired, update the debounced switch value
  else if (millis - paddle_debounce_timer >= SHIFT_DEBOUNCE_INTERVAL) {
    shift_debounced_packed = shift_raw;
  }

  // Update held switch value with new sample
  shift_prev = shift_raw;
}
