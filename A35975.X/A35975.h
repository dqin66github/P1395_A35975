/*
  -------------------------------------------
  This file contains configuration data specific to the A35975-000
  
  Dan Parker
  2012-06-09

  --------------------------------------------
*/

#ifndef __A35975_H
#define __A35975_H


#include <p30F6014a.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include "P1395_CAN_SLAVE.h"
#include "ETM_ANALOG.h"
#include "P1395_MODULE_CONFIG.h"

#include "A35975_SETTINGS.h"



#ifndef ISMAIN 
  #define EXTERN extern
#else
  #define EXTERN /*main*/
#endif

// --------- Compile Time Options -----------------

//#define DEMO   /* simulator for CAN protocol */

#define TEST_MODE_BYP_FIBER_OFF        1    /* don't turn off hv or trig because fiber off, for test only */

#define USE_ENGINEERING_UNIT_ON_GUN_DRIVER    1   /* use engineering units for all parameters on CAN */

//#define TEST_BYP_FPGA_FAULTS       1

//#define TEST_STATE_LOG             1

//#define TEST_SIMULATOR    1


//#define LOOPBACK_TEST    1

//#define ENABLE_STANDARD_CANOPEN     1   // enable heartbeat, standard command set


// --------- Resource Summary  -----------------
/*
  Hardware Module Resource Usage

  CAN2   - Used/Configured by ETM CAN (optical CAN)
  Timer2 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer3 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by Gun Driver
  I2C    - Used/Configured by EEPROM Module


  Timer1 - Used to time to Lambda Charge Time and the Lambda Inhibit Time
  Timer5 - Used for 10msTicToc

  ADC Module - Not used

*/


// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// ----------------- DIGITAL INPUT PINS --------------- //
/*
	RA15 - optical HV enable1
	RA14 - optical HV enable2
	RA13 - test pulse good2
	RA12 - test pulse good1
  
	RB15 - CS DAC Enable 

	RB11 - optical Trig enable
	RB9  - optical test pulse enable


*/

//   ------------------  Digital Output Pins ---------------
/*

  
*/

//					         fedcba9876543210
#define A35975_TRISA_VALUE 0b1111000000000000 
#define A35975_TRISB_VALUE 0b1000101000000000 
#define A35975_TRISC_VALUE 0b0000000000000000 
#define A35975_TRISD_VALUE 0b0000000000000000 
#define A35975_TRISF_VALUE 0b0000000010000101 
#define A35975_TRISG_VALUE 0b0000000000000001

//------------------- GUN Driver Interface I/O ------------------------- //
// FPGA Output Pins
#define PIN_CS_DAC_ENABLE                    _LATD13		   
#define OLL_CS_DAC_ENABLE                    1

#define PIN_CS_ADC_ENABLE                    _LATD14		   
#define OLL_CS_ADC_ENABLE                    1

#define PIN_CS_AUX_ENABLE                    _LATD15		   
#define OLL_CS_AUX_ENABLE                    1

#define PIN_CS_DAC_ENABLE_BIT                0x2000
#define PIN_CS_ADC_ENABLE_BIT                0x4000
#define PIN_CS_AUX_ENABLE_BIT                0x8000
#define PIN_CS_ALL_ENABLE_BITS               0xe000


// LOGIC Output Pins
#define PIN_OPT_GD_READY                     _LATA10	   
#define OLL_OPT_GD_READY                     1

#define PIN_CAN_HV_ENABLE                    _LATB12	   
#define OLL_CAN_HV_ENABLE                    1

#define PIN_CAN_PULSETOP_ENABLE              _LATB10	   
#define OLL_CAN_PULSETOP_ENABLE              1

#define PIN_CAN_TRIGGER_ENABLE               _LATB8	   
#define OLL_CAN_TRIGGER_ENABLE               1


//#define PIN_OPT_CAN_XMIT_OUT                 _LATG1		   
//#define OLL_OPT_CAN_XMIT_ON                  1


// LOGIC Input Pins
//#define PIN_CS_DAC_ENABLE_INPUT            _RB15		   // was designed for DAC output 


#ifndef DEMO
	#define PIN_OPT_HV_ENABLE1_INPUT             _RA15	   
	#define ILL_OPT_HV_ENABLE                    1

	#define PIN_OPT_HV_ENABLE2_INPUT             _RA14	   // A14 and A15 inputs are tied together

	#define PIN_OPT_TRIG_ENABLE_INPUT            _RB11
	#define ILL_OPT_TRIG_ENABLE                  1

	#define PIN_OPT_TEST_PULSE_ENABLE_INPUT       _RB9	   
	#define ILL_OPT_TEST_PULSE_ENABLE             1

#else
	#define PIN_OPT_HV_ENABLE1_INPUT             1	   
	#define ILL_OPT_HV_ENABLE                    1

	#define PIN_OPT_HV_ENABLE2_INPUT             _RA14	   // A14 and A15 inputs are tied together

	#define PIN_OPT_TRIG_ENABLE_INPUT              1
	#define ILL_OPT_TRIG_ENABLE                    1

	#define PIN_OPT_TEST_PULSE_ENABLE_INPUT        1	   
	#define ILL_OPT_TEST_PULSE_ENABLE              1
#endif

#define PIN_TEST_PULSE_GOOD1_INPUT           _RA12
#define ILL_TEST_PULSE_GOOD                  1

#define PIN_TEST_PULSE_GOOD2_INPUT           _RA13	   // A12 and A13 inputs are tied together



// MCP4822 DAC Output Pins
#define PIN_CS_MCP4822_ENABLE                _LATB14	   
#define OLL_CS_MCP4822_ENABLE                0

#define PIN_LDAC_MCP4822_ENABLE              _LATB13	   
#define OLL_LDAC_MCP4822_ENABLE              0



// UART TX enable
#define PIN_RS422_DE                         _LATF4
#define OLL_RS422_DE_ENABLE_RS422_DRIVER     1




// LED Indicator Output Pins
#define OLL_LED_ON                            0

#define PIN_LED_24DC_OK                      _LATG14	   

#define PIN_LED_LAST_PULSE_GOOD              _LATG12	   

#define PIN_LED_GD_READY                     _LATG13	   

#define PIN_LED_HV_ENABLE                    _LATG15   

#define PIN_LED_AC_ON                        _LATC1   

#define PIN_LED_LAST_PULSE_FAIL              _LATC2  
//#define TRIS_PIN_LED_LAST_PULSE_FAIL         _TRISC2		   
//#define OLL_LED_LAST_PULSE_FAIL              0

#define PIN_LED_WARMUP                       _LATC3   

#define PIN_LED_SUM_FAULT                    _LATC4   


// -----------------------  END IO PIN CONFIGURATION ------------------------ //


/* ------------------------------ CLOCK AND TIMING CONFIGURATION ------------------------- */
//#define FCY_CLK                    10000000      // 29.495 MHz   defined in ETM CAN
//#define FCY_CLK_MHZ                10.000        // 29.495 MHz   defined in ETM CAN

#define UART1_BAUDRATE             124000        // U1 Baud Rate
#define I2C_CLK                    100000        // Target I2C Clock frequency of 100KHz



// -------------------------------------------- INTERNAL MODULE CONFIGURATION --------------------------------------------------//


/*
  --- SPI1 Port --- 
  This SPI port is used to connect with the gun driver
  The prescales of 16:1 and 1:1 will generate a clock = Fcy/16.  In this case 1.843MHz
  This must be slower to compensate for the 2x delay across the optocoupler 200ns with filtering in one direction, 80ns (without filtering) in the other direction
  Minimum clock period is therefore 280ns + holdtime + margins
*/
//#define A35975_SPI2CON_VALUE  (FRAME_ENABLE_OFF & DISABLE_SDO_PIN & SPI_MODE16_ON & SPI_SMP_ON & SPI_CKE_ON & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
//#define A35975_SPI2STAT_VALUE (SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   
#define A35975_SPI1CON_VALUE  (FRAME_ENABLE_OFF & ENABLE_SDO_PIN & SPI_MODE16_OFF & SPI_SMP_OFF & SPI_CKE_OFF & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
#define A35975_SPI1STAT_VALUE (SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   


/* 
   --- SPI2 Port ---
   This SPI port is used to connect to the octal DAC
*/
//#define A35975_SPI1CON_VALUE  (FRAME_ENABLE_OFF & ENABLE_SDO_PIN & SPI_MODE16_ON & SPI_SMP_OFF & SPI_CKE_ON & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
//#define A35975_SPI1STAT_VALUE (SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   
#define A35975_SPI2CON_VALUE  (FRAME_ENABLE_OFF & ENABLE_SDO_PIN & SPI_MODE16_ON & SPI_SMP_OFF & SPI_CKE_ON & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
#define A35975_SPI2STAT_VALUE (SPI_DISABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   



#if FCY_CLK > 20000000
// FCY between 20 and 30 MHZ
// SPI1 Clock (divide by 4) 5->7.7MHz
// SPI2 Clock (divide by 16) 1.25MHz - 1.875MHz
#define A35975_SPI1CON_CLOCK (SEC_PRESCAL_1_1 & PRI_PRESCAL_4_1)
#define A35975_SPI2CON_CLOCK (SEC_PRESCAL_1_1 & PRI_PRESCAL_16_1)

#elif FCY_CLK > 15000000
// FCY between 15 and 20 MHZ
// SPI1 Clock (divide by 3) 5->6.6MHz
// SPI2 Clock (divide by 12) 1.25MHz - 1.66MHz
#define A35975_SPI1CON_CLOCK (SEC_PRESCAL_3_1 & PRI_PRESCAL_1_1)
#define A35975_SPI2CON_CLOCK (SEC_PRESCAL_3_1 & PRI_PRESCAL_4_1)

#elif FCY_CLK > 8000000
// FCY Between 8 and 15 MHZ
// SPI1 Clock (divide by 2) 4->7.5MHz
// SPI2 Clock (divide by 8) 1 -> 1.875MHz
#define A35975_SPI1CON_CLOCK (SEC_PRESCAL_4_1 & PRI_PRESCAL_4_1)
#define A35975_SPI2CON_CLOCK (SEC_PRESCAL_2_1 & PRI_PRESCAL_1_1)

#else
// FCY Less than 8 MHz.
// SPI1 Clock (divide by 1) 0 -> 8MHz
// SPI2 Clock (divide by 5) 0 -> 1.6 MHz 
#define A35975_SPI1CON_CLOCK (SEC_PRESCAL_1_1 & PRI_PRESCAL_1_1)
#define A35975_SPI2CON_CLOCK (SEC_PRESCAL_5_1 & PRI_PRESCAL_1_1)

#endif





/* 
   --- I2C setup ---
  See i2c.h and Microchip documentation for more information about the condfiguration
*/
#define I2CCON_SETUP   (I2C_ON & I2C_IDLE_STOP &  I2C_CLK_REL & I2C_IPMI_DIS & I2C_7BIT_ADD & I2C_SLW_EN & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS & I2C_ACK & I2C_ACK_DIS)
#define I2C_BAUD_RATE_GENERATOR   ((FCY_CLK/I2C_CLK) - (FCY_CLK/4000000) - 1) // This equation assumes PGD of 250nS.  This is not listed in the data sheet.  The supplementat data sheet uses
                                                                              // 250ns in the table and 900ns in the precalcuated values . . . which is it???  does it matter?  no!!


/* 
   --- UART 1 setup ---
   See uart.h and Microchip documentation for more information about the condfiguration

*/
#define A35975_U1MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A35975_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A35975_U1BRG_VALUE         (((FCY_CLK/UART1_BAUDRATE)/16)-1)


/* 
   --- CAN 1 setup ---
   See can.h and Microchip documentation for more information about the condfiguration

*
#define A35975_C1MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A35975_C1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A35975_C1BRG_VALUE         (((FCY_CLK/UART1_BAUDRATE)/16)-1)

*/

/*
  --- Timer1 Setup ---
  x64 multiplier will yield max period of 142mS, 2.17uS per tick
  Period of 100mS
*/
#define A35975_T1CON_VALUE  (T1_OFF & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_64 & T1_SOURCE_INT)
#define A35975_PR1_ROLL_US  100000      // 100mS
#define A35975_PR1_VALUE    FCY_CLK_MHZ*A35975_PR1_ROLL_US/64
//#define A35975_PR2_VALUE    46080


/*
  --- Timer4 Setup ---
  x1 multiplier will yield max period of 2.22mS
*/
#define A35975_T4CON_VALUE  (T4_OFF & T4_IDLE_CON & T4_GATE_OFF & T4_PS_1_1 & T4_32BIT_MODE_OFF & T4_SOURCE_INT)


/*
  --- Timer5 Setup ---
  x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
  Period of 10mS
*/
#define A35975_T5CON_VALUE     (T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT)
#define A35975_PR5_VALUE_US    10000   // 10mS
#define A35975_PR5_VALUE       (FCY_CLK_MHZ*A35975_PR5_VALUE_US/8)
//#define A35975_PR5_VALUE       36869



/* 
   --- 12-BIT ADC Configuration ---
   Goal, when setup, the system will scan through the selected Analog channels and sample
     
   ADC_CONV_CLK_7Tcy2 makes the ADC cloack 350ns at 10 MHz Clock

   AN3 -  PFN Rev Current           - Only sampled after a pulse
   
   AN4 - pac_#1                     - 2.56s tau - Analog Input Bandwidth = 200 Hz
   AN5 - pac_#2                     - 2.56s tau - Analog Input Bandwidth = 200 Hz
   
   AN6 - Thyratron Cathode Heater   - 160mS tau - Analog Input Bandwidth = 10 Hz
   AN7 - Thyratron Reservoir Heater - 160mS tau - Analog Input Bandwidth = 10 Hz
   
   AN8  - magnet_current            - 160mS tau - Analog Input Bandwidth = 200 Hz
   AN9  - magnet_voltage            - 160mS tau - Analog Input Bandwidth = 200 Hz
   
   AN10 - filament_voltage          - 160mS tau - Analog Input Bandwidth = 200 Hz    
   AN11 - filament_current          - 160mS tau - Analog Input Bandwidth = 200 Hz    
   
   AN12 - lambda_vpeak              - 640mS tau - Analog Input Bandwidth = 200 Hz
   AN13 - lambda_vmon               - Only Sampled at EOC
*/



//#define A35975_ADCON1_VALUE (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
//#define A35975_ADCON2_VALUE (ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_11 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF)
//#define A35975_ADCON3_VALUE (ADC_SAMPLE_TIME_3 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_7Tcy2)
//#define A35975_ADCHS_VALUE  (ADC_CH0_POS_SAMPLEA_AN3 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN3 & ADC_CH0_NEG_SAMPLEB_VREFN)
//#define A35975_ADPCFG_VALUE (ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA & ENABLE_AN8_ANA & ENABLE_AN9_ANA & ENABLE_AN10_ANA & ENABLE_AN11_ANA & ENABLE_AN12_ANA & ENABLE_AN13_ANA)
//#define A35975_ADCSSL_VALUE (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN14 & SKIP_SCAN_AN15)

// With these settings, the effective sample rate is around 150K samples/sec




 
// ---- Hard Coded Delays ---- //
#define DELAY_TCY_10US                          100       // 10us us at 10MHz Clock

#define DELAY_PULSE_CABLE_SELECT_PROP_DELAY_US  1        // 1uS
                                                          // This delay must be longer than the propogation delay on the Isolated DAC Cable Select Line
#define DELAY_PULSE_CABLE_SELECT_PROP_DELAY     (FCY_CLK_MHZ*DELAY_PULSE_CABLE_SELECT_PROP_DELAY_US)



/*
#define DELAY_TCY_TRIGGER_TO_ADC_SAMPLE_US      2.5       // 2uS
                                                          // This is the delay between when a trigger pulse is detected and when the isolated DACs are sampled
#define DELAY_TCY_TRIGGER_TO_ADC_SAMPLE         (FCY_CLK_MHZ*DELAY_TCY_TRIGGER_TO_ADC_SAMPLE_US)


#define DELAY_TCY_ISR_CALL_MODIFIER             5         // 
                                                          // This compensates for the tirgger ISR call delay and instructions before the delay call is made
                                                          // This should be equal to 4 + the number of instructions cycles to reach 
                                                          //   __delay32(DELAY_TCY_TRIGGER_TO_ADC_SAMPLE - DELAY_TCY_ISR_CALL_MODIFIER);
#define DELAY_TCY_TRIGGER_TO_CURRENT_PULSE_NS   100      
*/


// ----------- Data Structures ------------ //

#define ANALOG_SET_SIZE   8
#define ANALOG_READ_SIZE  10


// analog set pointers
enum {
   ANA_SET_EK = 0,
   ANA_SET_EG,
   ANA_SET_EF,
   ANA_SET_HV_ON,
   ANA_SET_HTR_ON,
   ANA_SET_PULSETOP_ON,
   ANA_SET_TRIG_ON,
   ANA_SET_WDOG,
};


  EXTERN struct {
	    unsigned int channel;       /* DAC channel 							   */
		unsigned int ip_set;        /* current setting (0-ffff)		           */
		unsigned int ip_set_alt;    /* static alt setting if flag              */
		unsigned int ip_set_flag;   /* static alt setting if varptr goes true  */
				  
		unsigned int ip_set_max;    /* factory maximum setting (0-ffff) 	   */
		unsigned int ip_set_min;    /* factory minimum setting (0-ffff) 	   */

		unsigned int need_update; 

	} analog_sets[ANALOG_SET_SIZE]
#ifdef ISMAIN
	= {	  {0,  0, 0, 0,   0xffff, 0,   0},
		  {1,  0, 0, 0,   0xffff, 0,   0},
		  {2,  0, 0, 0,   0xffff, 0,   0},
		  {3,  0, 0, 0,   0xffff, 0,   0},
		  {4,  0, 0, 0,   0xffff, 0,   0},
		  {5,  0, 0, 0,   0xffff, 0,   0},
		  {6,  0, 0, 0,   0xffff, 0,   0},
		  {7,  0, 0, 0,   0xffff, 0,   0},
		  
	}
#endif
	;

extern void FaultEk(unsigned state);
extern void FaultEf(unsigned state);
extern void FaultIf(unsigned state);
extern void FaultEg(unsigned state);
extern void FaultEc(unsigned state);
extern void Fault24v(unsigned state);

extern void SetEfLimits(void);
extern void SetEkLimits(void);
extern void SetEgLimits(void);

extern void SetEf(unsigned set_value);
extern void SetEk(unsigned set_value);
extern void SetEg(unsigned set_value);

extern unsigned char AreAnyReferenceNotConfigured(void);

// analog read pointers
enum {
   ANA_RD_EK = 0,
   ANA_RD_IKA,
   ANA_RD_IKP,
   ANA_RD_EF,
   ANA_RD_IF,
   ANA_RD_EG, // grid V
   ANA_RD_EC, // bias V
   ANA_RD_24V, // 24DC
   ANA_RD_TEMP, // temperature
   
   ANA_RD_HTR_WARMUP,
   ANA_RD_WATCHDOG,
   ANA_RD_ARC,
   ANA_RD_OT,
   ANA_RD_PW_DUTY,
   ANA_RD_BIASFLT,
   ANA_RD_DA_FDBK,
};

 EXTERN	struct {
	    unsigned int channel;	/* ADC channel 							       */
		unsigned int read_cur;  /* current reading                             */

		unsigned int read_cnt;  /* how many readings taken                     */
		unsigned int read_err;  /* how many errors                             */

		unsigned int read_f_lo; /* low end fault level                         */
		unsigned int read_f_hi; /* high end fault level                        */
		unsigned int read_m_lo; /* lo end mask   0 off   1 on   2 trip         */
		unsigned int read_m_hi; /* hi end mask   0 off   1 on   2 trip         */
		unsigned int events;
		void (*fault_vect)(unsigned state);

	} analog_reads[ANALOG_READ_SIZE]
#ifdef ISMAIN			
	= {	 {0,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultEk  }, 
		 {1,  0, 0, 0,    0, 0xfff, 0,  0, 0, 0        }, 
		 {2,  0, 0, 0,    0, 0xfff, 0,  0, 0, 0        }, 
		 {3,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultEf  }, 
		 {4,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultIf  }, 
		 {5,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultEg  }, 
		 {6,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultEc  }, 
		 {7,  0, 0, 0,    0, 0xfff, 0,  0, 0, Fault24v }, 
		 {8,  0, 0, 0,    0, 0xfff, 0,  0, 0, 0        }, 

	}
#endif
	;


#ifdef USE_ENGINEERING_UNIT_ON_GUN_DRIVER 

#define CAN_SCALE_TABLE_SIZE  13
// analog read pointers
enum {
   CAN_RD_EK = 0,
//   CAN_RD_IKA,
   CAN_RD_IKP,
   CAN_RD_EF,
   CAN_RD_IF,
   CAN_RD_EG, // grid V
   CAN_RD_EC, // bias V
 //  CAN_RD_24V, // 24DC
   CAN_RD_TEMP, // temperature
   
   CAN_RD_EKSET,
   CAN_RD_EFSET,
   CAN_RD_EGSET,
   CAN_SET_EKSET,
   CAN_SET_EFSET,
   CAN_SET_EGSET,
   
};
/*
Name 	      |   cal factor | offset|	CAN Interface |	CAN Unit/bit |	CAN Range|	CAN Scaling| ScaleFactor |Scale Offset
--------------------------------------------------------------------------------------------------------------------------
EK_RD		  |	  0.005555	 | 0	 |	1 V/bit	      |  0.001		 |	65.535	 |	5.555	   | 22753 		 | 0
IKA_RD		  |	  0.001667	 | 0	 |	1 mA/bit	  |  0.001		 |	65.535	 |	1.667	   | 54624 		 | 0
IKP_RD		  |	  0.277		 | 0	 |	100 mA/bit    |    0.1		 |	6553.5	 |	2.77	   | 11345 		 | 0
EF_RD		  |	  0.00222	 | 0	 |	1 mV/bit	  |  0.001		 |	65.535	 |	2.22	   | 9093  		 | 0
IF_RD		  |	  0.001667	 | 0	 |	10 mA/bit	  |  0.001		 |	65.535	 |	1.667	   | 54624 		 | 0
EG_RD		  |	  0.1111	 | 80	 |   100 mV/bit	  |    0.1		 |	6553.5	 |	1.111	   | 36405 		 | 0
EC_RD		  |	  0.05555	 | 0	 |	100 mV/bit    |    0.1		 |	6553.5	 |	0.5555	   | 18202 		 | 0
TEMP_RD		  |	  0.0133	 | 0	 |	0.01 C/bit    |   0.01		 |	655.35	 |	1.33	   | 43581 		 | 0
			  |				 |		 |				  |				 |		  	 |			   |	   		 |	
Ekset bits-val|	  0.0003333	 | 0	 |	1 V/bit	      |  0.001		 |	65.535	 |	0.3333	   | 10921 		 | 0
Ekset val-bits|				 |		 |				  |				 |		  	 |	3.00030003 | 12289 		 | 0
Efset bits-val|	  0.000133	 | 0	 |	10 mV/bit	  |  0.001		 |	65.535	 |	0.133	   | 4358  		 | 0
Efset val-bits|				 |		 |				  |				 |		  	 |	7.518796992| 30796 		 | 0
Egset bits-val|	  0.00666	 | 80	 |   100 mV/bit	  |    0.1		 |	6553.5	 |	0.0666	   | 2182  		 | 0
Egset val-bits|				 |		 |				  |				 |			 |	15.01501502| 61501 		 | 0  


Note:  Scale offset for EG read/set is handled by GUI.
			  
*/			  

#define CAL_EK_RD    0.005555
#define CAL_IKA_RD   0.001667
#define CAL_IKP_RD   0.277
#define CAL_EF_RD    0.00222
#define CAL_IF_RD    0.001667
#define CAL_EG_RD    0.1111
#define CAL_EC_RD	 0.05555
#define CAL_TEMP_RD	 0.0133

#define CAL_EKSET    0.0003333
#define CAL_EFSET    0.000133
#define CAL_EGSET    0.00666

#define CAN_EK_SCALE     0.001
#define CAN_IKA_SCALE 	 0.001	
#define CAN_IKP_SCALE 	 0.1	
#define CAN_EF_SCALE 	 0.001	
#define CAN_IF_SCALE 	 0.001	
#define CAN_EG_SCALE 	 0.1	
#define CAN_EC_SCALE 	 0.1	
#define CAN_TEMP_SCALE 	 0.01	
								
#define CAN_EKSET_SCALE  0.001
#define CAN_EFSET_SCALE  0.001
#define CAN_EGSET_SCALE  0.1
					 
					 


 EXTERN	struct {
	  // -------- These are used to calibrate and scale the ADC Reading to Engineering Units ---------
	  unsigned int fixed_scale;
	  signed int   fixed_offset;

	} CAN_scale_table[CAN_SCALE_TABLE_SIZE]
#ifdef ISMAIN			
	= {	 {0, 0}, 
		 {0, 0}, 
		 {0, 0}, 
		 {0, 0}, 
		 {0, 0}, 
		 {0, 0}, 
		 {0, 0},
		  
		 {0, 0}, 
		 {0, 0}, 
		 {0, 0}, 
		 {0, 0}, 
		 {0, 0}, 
		 {0, 0}, 

	}
#endif
	;

#endif /* USE_ENGINEERING_UNIT_ON_GUN_DRIVER */




#define FAULT_SIZE  22  /* 6 ADC + 16 FPGA_ID */

//#define DIGI_ADC_HTR_FLT_WARMUP    0
#define DIGI_ADC_FPGA_WATCHDOG     1
#define DIGI_ADC_ARC      		   2
//#define DIGI_ADC_TEMP			   3
//#define DIGI_ADC_PW_DUTY  		   4
//#define DIGI_ADC_BIAS_TOP          5

#define DIGI_ID_ARC_COUNT          6
//#define DIGI_ID_ARC_HV_INHIBIT     7
//#define DIGI_ID_EF_LESS_4p5V       8
//#define DIGI_ID_TEMP_65C		   9
#define DIGI_ID_TEMP_75C		   10
#define DIGI_ID_PW_LIMITING        11
#define DIGI_ID_PRF			       12
#define DIGI_ID_CURR_PW		       13

#define DIGI_ID_GRID_HW		  	   14
#define DIGI_ID_GRID_OV		       15
#define DIGI_ID_GRID_UV		       16
#define DIGI_ID_BIAS_V			   17
//#define DIGI_ID_HV_REGULATION      18
#define DIGI_ID_DIP_SW             19
#define DIGI_ID_TEST_MODE		   20
#define DIGI_ID_LOCAL_MODE         21
 




 EXTERN  struct {
	    unsigned int state;                 /* fault input state                */
		unsigned int mask;                  /* 0 = disabled  -  1 = enabled     */

	    unsigned int from_adc;              /* 1: from ADC, 0: from FPGA ID     */
	    unsigned int bits;                   /* bit position */
	    unsigned int fre;                   /* how many events have to happen   */
	                                        /* 0 = do next. 1 = one free        */
	    unsigned int left;                  /* how many events left.            */

	    unsigned int fault_latched;         /* fault was latched, need send RESET to FPGA board  */
		unsigned int action_code;           /* 0: no action, update the bit, 1: htr off, 2: hv off, 3: pulsetop off, 4: trig off, 99: ignore the bit */


	} digi_reads[FAULT_SIZE]
	#ifdef ISMAIN                       /* if main initialise the table */
	= {	
		{0, 0,   1, 0,  1,   0,    0, 99},	// htr/warmup fault
		{0, 0,   1, 1,  1,   0,    1, 1},	// fpga watchdog fault
		{0, 0,   1, 2,  1,   0,    0, 2},	// arc fault
		{0, 0,   1, 3,  1,   0,    0, 99},	// overtemp fault
		{0, 0,   1, 4,  1,   0,    0, 99},	// pw/duty fault
		{0, 0,   1, 5,  1,   0,    0, 99},	// bias or top fault
														   
		{0, 0,   0, 0,  1,   0,    0, 99},  // arc count > 0 			   
		{0, 0,   0, 1,  1,   0,    0, 99}, 	// arc HV inh active			   
		{0, 0,   0, 2,  1,   0,    0, 99},  // heater volt < 4.5V
		{0, 0,   0, 3,  1,   0,    0, 99},	// max temp > 65c
		{0, 0,   0, 4,  1,   0,    1, 1},	// max temp > 75c
		{0, 0,   0, 5,  1,   0,    0, 99},	// pulse width limiting
		{0, 0,   0, 6,  1,   0,    0, 2},	// prf fault
		{0, 0,   0, 7,  1,   0,    0, 2},	// current pw fault

		{0, 0,   0, 8,  1,   0,    1, 1},	// grid module hw fault
		{0, 0,   0, 9,  1,   0,    1, 1},	// grid module o/v fault
		{0, 0,   0, 10, 1,   0,    1, 1},	// grid module u/v fault
		{0, 0,   0, 11, 1,   0,    1, 1},	// grid module biasV fault
		{0, 0,   0, 12, 1,   0,    0, 99},	// hv regulation fault
		{0, 0,   0, 13, 1,   0,    0, 99},	// dip sw1 on
		{0, 0,   0, 14, 1,   0,    0, 99},	// test mode switch on
		{0, 0,   0, 15, 1,   0,    0, 99},	// local mode switch on


	}
	#endif
	;

/*
  --- LOGIC  STATE DEFINITIONS ---
  See flow diagram for more information
  DPARKER add flow diagram doc number
*/
#define STATE_START_UP                       0x06
#define STATE_START_UP2                      0x08

#define STATE_SYSTEM_HTR_OFF          		 0x10
#define STATE_READY_FOR_HEATER 				 0x12
#define STATE_HEATER_STARTUP   				 0x14
#define STATE_WARM_UP                        0x16

#define STATE_SYSTEM_HV_OFF                  0x20
#define STATE_READY_FOR_HV		     		 0x22
#define STATE_HV_STARTUP                     0x24
#define STATE_HV_ON                          0x26

#define STATE_SYSTEM_PULSETOP_OFF            0x30
#define STATE_READY_FOR_PULSETOP    		 0x32
#define STATE_PULSETOP_STARTUP               0x34
#define STATE_PULSETOP_ON		             0x36

#define STATE_SYSTEM_TRIG_OFF                0x40
#define STATE_READY_FOR_TRIG		         0x42
#define STATE_TRIG_STARTUP	                 0x44
#define STATE_TRIG_ON	                     0x46


#define STATE_FAULT_COLD_FAULT               0x80 
#define STATE_FAULT_HOT_FAULT                0xA0 // 0x80 + 0x20


/* 
  --- SYSTEM STATE BYTE DEFINITIONS --- 
*/
#define SYS_BYTE_HTR_ON						 0x01
#define SYS_BYTE_LOGIC_READY  				 0x02
#define SYS_BYTE_HV_ON						 0x04
#define SYS_BYTE_PULSETOP_ON    		     0x08

#define SYS_BYTE_TRIG_ON					 0x10
#define SYS_BYTE_FAULT_ACTIVE                0x20
#define SYS_BYTE_HTR_WARMUP				     0x40  /* htr off or warmup */
#define SYS_BYTE_HV_DRIVEUP				     0x80

/*
  --- Public Functions ---
*/

extern void DoStateMachine(void);

extern void LogicHeaterControl(unsigned char turnon);
extern void LogHvControl(unsigned char turnon);
extern void LogPulsetopControl(unsigned char turnon);
extern void LogTrigControl(unsigned char turnon);

extern void SendHeaterRef(unsigned int bits);
extern void SendHvRef(unsigned int bits);
extern void SendPulsetopRef(unsigned int bits);
extern void SendWatchdogRef(unsigned int bits);

extern void Do10msTicToc(void);
extern void ResetFPGA(void);


#define SYSTEM_WARM_UP_TIME      3000 /* 100ms Units  //DPARKER this is way to short */
#define EF_READ_MAX              2838 /*  -6.3/-0.00222 */
#define IF_READ_MAX              1051 /*  1.75/0.001666 */
//#define IF_READ_MAX_95P          1824 // 95% of IF_MAX
//#define IF_READ_MAX_85P          1633 // 85% of IF_MAX

#define IF_READ_MAX_90P           945 /* 90% of IF_MAX */

#define EF_SET_MAX              47369 /*  -6.3/-0.000133  */
#define EG_SET_MAX              33033 /* 140V, 0.00666    */
#define EK_SET_MAX              60060 /* -20kV/-0.000333  */
 


#define _ISRFASTNOPSV __attribute__((interrupt, shadow, no_auto_psv)) 
#define _ISRNOPSV __attribute__((interrupt, no_auto_psv)) 


/*
  --- Gobal Variales ---
*/
#ifdef ISMAIN
	unsigned char control_state;
	unsigned char last_control_state = STATE_START_UP;	
    unsigned char system_byte;

	unsigned int led_pulse_count;
	unsigned int htd_timer_in_100ms;
	unsigned int software_skip_warmup = 0;

	unsigned int fpga_ASDR;
	unsigned int faults_from_ADC;

 	unsigned long read_cycles_in_2s = 0;	// how fast is the updating speed

    unsigned int ekuv_timeout_10ms = 0;

    unsigned int ek_ref_changed_timer_10ms = 0;   // mask Ek faults when ref is changed
	unsigned int ef_ref_changed_timer_10ms = 0;   // mask Ef, If faults when ref is changed
	unsigned int eg_ref_changed_timer_10ms = 0;   // mask Eg fault when ref is changed

	unsigned char htr_OVOC_count = 0;   // for auto-reset htr OVOC feature
	unsigned int  htr_OVOC_rest_delay_timer_10ms = 0;	  // after OVOC fault, rest for a few seconds before turning htr on
	unsigned char htr_OVOC_auto_reset_disable = 0;        // if other system fault happens, disable htr OVOC auto-reset

//	BUFFER64BYTE uart1_input_buffer;
//	BUFFER64BYTE uart1_output_buffer;

	volatile unsigned int lvdinterrupt_counter = 0;

	volatile unsigned int _PERSISTENT last_known_action;
	volatile unsigned int _PERSISTENT last_osccon;

	unsigned int _PERSISTENT processor_crash_count;
	unsigned int previous_last_action;

    unsigned long hw_version_data;
#else
	extern unsigned char control_state;
	extern unsigned char last_control_state;
    extern unsigned char system_byte;

	extern unsigned int led_pulse_count;
	extern unsigned int htd_timer_in_100ms;
	extern unsigned int software_skip_warmup;	

	extern unsigned int fpga_ASDR;
	extern unsigned int faults_from_ADC;

	extern unsigned long read_cycles_in_2s;
 
    extern unsigned int ekuv_timeout_10ms;

    extern unsigned int ek_ref_changed_timer_10ms;   // mask Ek faults when ref is changed
	extern unsigned int ef_ref_changed_timer_10ms;   // mask Ef, If faults when ref is changed
	extern unsigned int eg_ref_changed_timer_10ms;   // mask Eg fault when ref is changed

	extern unsigned char htr_OVOC_count;                  // for auto-reset htr OVOC feature
	extern unsigned int  htr_OVOC_rest_delay_timer_10ms;  // after OVOC fault, rest for a few seconds before turning htr on
	extern unsigned char htr_OVOC_auto_reset_disable;     // if other system fault happens, disable htr OVOC auto-reset

//	extern BUFFER64BYTE uart1_input_buffer;
//	extern BUFFER64BYTE uart1_output_buffer;

	extern volatile unsigned int lvdinterrupt_counter;

	extern volatile unsigned int _PERSISTENT last_known_action;
	extern volatile unsigned int _PERSISTENT last_osccon;

	extern unsigned int _PERSISTENT processor_crash_count;
	extern unsigned int previous_last_action;

	extern  unsigned long hw_version_data;

#endif

extern signed int ps_magnet_config_ram_copy[16];
extern unsigned long EE_address_ps_magnet_config_in_EEPROM;



#define LAST_ACTION_DEFAULT_INT                        0xFABC
#define LAST_ACTION_CLEAR_LAST_ACTION                  0x0000
//#define LAST_ACTION_ADC_INTERRUPT                      0x0001
#define LAST_ACTION_LVD_INT                            0x0002
//#define LAST_ACTION_T1_INT                             0x0003
//#define LAST_ACTION_INT1_INT                           0x0004
//#define LAST_ACTION_UPDATE_IO_EXPANDER                 0x0005
#define LAST_ACTION_FILTER_ADC                         0x0006
#define LAST_ACTION_READ_ISOLATED_ADC                  0x0007
//#define LAST_ACTION_DO_THYRATRON_PID                   0x0008
#define LAST_ACTION_DO_10MS                            0x0009
#define LAST_ACTION_UPDATE_DAC_ALL                     0x000A
//#define LAST_ACTION_POST_PULSE_PROC                    0x000B
#define LAST_ACTION_HV_ON_LOOP                         0x000C
#define LAST_ACTION_OSC_FAIL                           0x000D
#define LAST_ACTION_ADDRESS_ERROR                      0x000E
#define LAST_ACTION_STACK_ERROR                        0x000F
#define LAST_ACTION_MATH_ERROR                         0x0010


// CAN bus related variables

EXTERN unsigned char sdo_reset_cmd_active;	  // logic resumes only when reset isn't active
EXTERN unsigned char sdo_logic_reset;        // a separate cmd to reset fault
EXTERN unsigned char sdo_htd_timer_reset; 

EXTERN unsigned char sdo_htr_enable;
EXTERN unsigned char sdo_hv_bypass;
EXTERN unsigned char sdo_hv_enable;
EXTERN unsigned char sdo_pulsetop_enable;
EXTERN unsigned char sdo_trig_enable;  




#define _STATUS_GD_HV_DISABLE                           _STATUS_0	
#define _STATUS_GD_HTR_NOT_READY                        _STATUS_1
#define _STATUS_GD_TRIG_NOT_ENABLED                     _STATUS_2
#define _STATUS_GD_TOP_NOT_ENABLED                      _STATUS_3
#define _STATUS_GD_HV_NOT_ENABLED    				    _STATUS_4
#define _STATUS_GD_HTR_NOT_ENABLED                      _STATUS_5	

//#define _STATUS_GD_FPGA_DIP_SWITCH                      _STATUS_5
//#define _STATUS_GD_FPGA_WIDTH_LIMITING                  _STATUS_6
//#define _STATUS_GD_FPGA_ARC_WARNING                     _STATUS_7

#define _FAULT_GD_SUM_FAULT                             _FAULT_0
#define _FAULT_GD_FPGA_COMM_LOST                        _FAULT_1
#define _FAULT_GD_SW_HTR_OVOC                           _FAULT_2
#define _FAULT_GD_SW_BIAS_UV                            _FAULT_3
#define _FAULT_GD_SW_EK_OV                              _FAULT_4
#define _FAULT_GD_SW_EK_UV                              _FAULT_5
#define _FAULT_GD_SW_GRID_OV                            _FAULT_6
#define _FAULT_GD_FPGA_TEMP_75C                         _FAULT_7

#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_8

#define _FAULT_GD_FPGA_ARC_FAULT                        _FAULT_9
#define _FAULT_GD_FPGA_PULSE_FAULT                      _FAULT_A
#define _FAULT_GD_FPGA_GRID_FAULT                       _FAULT_B

#define _FAULT_GD_SW_HTR_UV                             _FAULT_C
#define _FAULT_GD_SW_24V_FAULT                          _FAULT_D
#define _FAULT_GD_SYS_FAULTS                            _FAULT_E
//#define _FAULT_GD_SYS_FAULTS                            _FAULT_F


#endif
