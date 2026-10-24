# VESC motor controller telemetry.
# Field set follows the bldc firmware (commands.c, COMM_GET_VALUES) and the
# f1tenth/vesc ROS 1 package; vesc_nexus fills the subset available over CAN
# status packets (see README, "CAN protocol").

# Fault codes (bldc mc_fault_code)
int32 FAULT_CODE_NONE=0
int32 FAULT_CODE_OVER_VOLTAGE=1
int32 FAULT_CODE_UNDER_VOLTAGE=2
int32 FAULT_CODE_DRV8302=3
int32 FAULT_CODE_ABS_OVER_CURRENT=4
int32 FAULT_CODE_OVER_TEMP_FET=5
int32 FAULT_CODE_OVER_TEMP_MOTOR=6

float64 temp_fet             # MOSFET temperature, deg C
float64 temp_motor           # motor temperature, deg C             (CAN STATUS_4)
float64 current_motor        # motor current, A                     (CAN STATUS)
float64 current_input        # input (battery) current, A           (CAN STATUS_4)
float64 avg_id
float64 avg_iq
float64 duty_cycle           # duty cycle, -1..1                    (CAN STATUS)
float64 speed                # legacy: electrical RPM (use erpm)

float64 voltage_input        # input voltage, V                     (CAN STATUS_5)
float64 charge_drawn         # charge drawn from input, Ah          (CAN STATUS_2)
float64 charge_regen         # charge regenerated to input, Ah      (CAN STATUS_2)
float64 energy_drawn         # energy drawn from input, Wh
float64 energy_regen         # energy regenerated to input, Wh
int32   displacement         # net tachometer, counts
int32   distance_traveled    # absolute tachometer, counts
int32   fault_code

float64 pid_pos_now          # PID position, deg                    (CAN STATUS_4)
int32 controller_id

float64 ntc_temp_mos1
float64 ntc_temp_mos2
float64 ntc_temp_mos3
float64 avg_vd
float64 avg_vq

# vesc_nexus additions
string label                 # joint name
float64 speed_rpm            # mechanical motor shaft RPM = erpm / pole_pairs
float64 erpm                 # electrical RPM                       (CAN STATUS)
float64 power_w
float64 temp_controller      # MOSFET temperature, deg C            (CAN STATUS_4)
uint32 tachometer            # electrical revolutions               (CAN STATUS_5)
uint32 tachometer_abs
bool alive                   # at least one status packet received
