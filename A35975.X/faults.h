#ifndef __FAULTS_H
#define __FAULTS_H

#include "A35975.h"

typedef struct {
  unsigned int spi1_bus_error;
  unsigned int spi2_bus_error;
  unsigned int external_adc_false_trigger;
  unsigned int LTC2656_write_error;
  unsigned int setpoint_not_valid;
  unsigned int scale16bit_saturation;
  unsigned int reversescale16bit_saturation;
} TYPE_DEBUG_COUNTER;

EXTERN TYPE_DEBUG_COUNTER global_debug_counter;


// Simplified Fault List

/*

Magnetron Heater Faults
* External Analog Comparator Over voltage with Latch
* ADC Voltage reading over hard limit
* ADC Voltage reading over programmed value by X percent
* ADC Voltage reading under programmed value by X percent
* ADC Current reading to high for programmed voltage - A short circuit could cause this
* ADC Current reading to low for porgrammed voltage - An open circuit could cause this

Magnetron Magnet 
* External Analog Comparator Current out of Range Latch (note this will of course be set durring start up)
* ADC Current reading over hard limit
* ADC Current reading over programmed value by X percent
* ADC Current reading under programmed value by X percent
* ADC Voltage reading to high for programmed current - An open circuit could cause this
* ADC Voltage reading to low for porgrammed current - A short circuit could cause this

Arc Faults
* Too many consecutive alts
* Too many arcs over the last 300 pulses (1 second at full rep rate)
* Too many arcs over the last 18000 Pulses (1 minute at full rep rate)


Lambda Faults
* Lambda Sum Fault
* Lambda Over Temp
* Lambda Interlock Open
* Lambda Load Fault (usually a short on the lambda output)
* Lambda Phase Loss
* Lambda EOC Timeout (the lambda did not reach EOC durring the required time)
* ADC Lambda vpeak reading is too high
* ADC Lambda vpeak reading is too low (this needs a delay because the lamabda of course starts at zero Volts so it takes a while for Vpeak to charge up)

Thyratron Faults
* ADC Cathode Heater Voltage over hard limit
* ADC Cathode Heater Voltage over programmed value by X%
* ADC Cathode Heater Voltage under programmed value by X%
* Cathode Heater 4-20mA driver fault (the same line is used to monitor both the cathode heater and reservoir heater driver fault)
* Cathode Heater Control Saturation - The PID loop has saturated indicating that the cathode voltage is no longer under control

* ADC Reservoir Heater Voltage over hard limit
* ADC Reservoir Heater Voltage over programmed value by X%
* ADC Reservoir Heater Voltage under programmed value by X%
* Reservoir Heater 4-20mA driver fault (the same line is used to monitor both the cathode heater and reservoir heater driver fault)
* Reservoir Heater Control Saturation - The PID loop has saturated indicating that the reservoir voltage is no longer under control


Other Faults on control board
* The High Voltage Lambda is Not Powered
* Control Board Interlock #1 is open (this is where the water flow is monitored on the test unit)
* Control Board Interlock #2 is open
* Control Board Interlock #3 is open
* Control Board Interlock #4 is open


*/

// Debug Fault Register

/*
  This register is not handeled like the other fault registers.  The "faults" or "status" in this register can only be latched.  
  This is register is reset at power on.  It is not saved to EEPROM or normally monitored except in the lab.

  EEprom error - the test of the eeprom failed
  IO Expanded Error - the test of the external I/O expander Failed
  BOR Reset - The processor restarted from a Brown out Reset
  TRAPR Reset - The processor restarted from a Trap Reset
  WDT Reset - The processor restarted from a Watch Dog Timer Reset
  IOPUWR Reset - The processor restarted from a Illegal Condition Reset
  
  Math Saturation Errors - Overflow in the 16 bit math - this would be a result of a bug in the code or bad config paramenters
  
*/


// ------------------- FAULT REGISTER SET UP --------------------- //

EXTERN unsigned int debug_status_register;

#define STATUS_BOR_RESET                          0x0001 // The processor restarted from a Brown Out Reset 
#define STATUS_TRAPR_RESET                        0x0002 // The processor restarted from a Trap Reset
#define STATUS_WDT_RESET                          0x0004 // The processor restarted from a Watch Dog Time Out 
#define STATUS_IOPUWR_RESET                       0x0008 // The processor restarted from a Illegal Operation Reset
#define STATUS_POR_RESET                          0x0010 // The processor restarted from a Power On Reset
#define STATUS_EXTERNAL_RESET                     0x0020 // The processor restarted from an External Reset
#define STATUS_SOFTARE_RESET                      0x0040 // The processor restarted from a Software Reset
#define STATUS_DIGITAL_IO_EXP_ERROR               0x0080 // The read/write/read test of one of the IO Expanders failed

#define STATUS_UNUSED_3                           0x0100 
#define STATUS_UNUSED_4                           0x0200 
#define STATUS_UNUSED_5                           0x0400 
#define STATUS_UNUSED_6                           0x0800 
#define STATUS_UNUSED_7                           0x1000
#define STATUS_UNUSED_8                           0x2000
#define STATUS_UNUSED_9                           0x4000
#define STATUS_UNUSED_10                          0x8000


// CONTROL FAULT REGISTER, 3 main types
EXTERN unsigned int faults_reg_system_control;
EXTERN unsigned int faults_reg_software;	    
EXTERN unsigned int faults_reg_digi_from_gd_fpgaid;

#define FPGAID_FAULTS_MASK                        0x0FD6

// main fault type definitions
#define FAULTS_TYPE_SYSTEM_CONTROL                1
#define FAULTS_TYPE_SOFTWARE                      2
#define FAULTS_TYPE_DIGI_FROM_FPGAID              3

// system fault bits
#define FAULTS_SYS_FPGAID                         0x0001
#define FAULTS_SYS_CAN_TIMEOUT                    0x0002
#define FAULTS_SYS_FPGA_WATCHDOG_ERR              0x0004

#define FAULTS_SYS_LOGIC_STATE                    0x0010
#define FAULTS_SYS_ILLEGAL_INTERRUPT			  0x0020


#define FAULTS_SW_EFOV_IFOC  					  0x0001  // Ef > (120% or .2V) ref, or If > Ifmax
#define FAULTS_SW_EFUV       					  0x0002  // Ef < (85% or .2V) ref
#define FAULTS_SW_ECUV      					  0x0004  // abs(Ec) < 120V
#define FAULTS_SW_EKOV_EKUV  					  0x0008  // Ek 10% higher or lower

#define FAULTS_SW_EGOV      					  0x0010  // Eg > (ref + 10)
#define FAULTS_SW_24V        					  0x0020  // out of 10% 24V range





#define NO_FAULTS  0x0000
#define ALL_FAULTS 0x1111    



void DoFaultRecord(unsigned int fault_type, unsigned int fault_bit);
void DoFaultClear(unsigned int fault_type, unsigned int fault_bit);

extern void CheckAnalogLimits(unsigned index);
extern void DoFaultAction(unsigned char type, unsigned char disable_htr_auto_reset);



void UpdateFaults(void);
/*
  This function updates all faults that are checked on a periodic basis.
  This is all faults EXCEPT for the pulse faults (these are checked after each pulse)
  It is called by Do10msTicToc() once every 10 ms
  What this function does
  * Loads the fault/warning masks for the current state
  * Checks all the Magnetron Faults Inputs and sets Status/Warning/Fault registers
  * Checks all the HV Lambda Fault Inputs and sets Status/Warning/Fault registers
  * Checks all the Thyratron Fault Inputs and sets Status/Warning/Fault registers
  * Checks all the Control Board Faults and sets Status/Warning/Fault registers

*/


void UpdatePulseData(unsigned char mode);
/*
  This function updates all the pulse faults and is called after each pulse.
  * Looks for an arc. 
*/

void ResetHWLatches(void);

void ResetAllFaults();
// DPARKER need to write function

unsigned int CheckStartupFailed(void);

unsigned int CheckFaultActive(void);

unsigned int CheckColdFaultActive(void);





void RecordThisMagnetronFault(unsigned int fault_bit);
void RecordThisHighVoltageFault(unsigned int fault_bit);
void RecordThisThyratronFault(unsigned int fault_bit);
void RecordThisControlBoardFault(unsigned int fault_bit);


void ResetPulseLatches(void);


#endif



