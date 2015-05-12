
#define ISMAIN	 1   /* init some values at bootup */


#include "A35975.h"
#include "FIRMWARE_VERSION.h"
#include "ETM_EEPROM.h"
#include "ETM_SPI.h"
#include <spi.h>
#include "faults.h"
#include "ETM_SCALE.h"


// This is firmware for the Gun Driver Board

_FOSC(ECIO & CSW_FSCM_OFF); 
//_FWDT(WDT_OFF);  // 1 Second watchdog timer
_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


//static void SetupAdc(void);
static unsigned ReadAdcChannel(unsigned chan);
static void SetDacChannel(unsigned chan, unsigned setvalue);
static unsigned long ReadFPGAID(void);
static void DoFpgaWatchdog(void);

void InitializeA35975(void);
void DoStateMachine(void);
void DoA35975(void);



//#define FPGA_ID     				0x1840  // used to verify GD FPGA communications
#define FPGA_ID     				0x0040  // used to verify GD FPGA communications
//#define FPGA_ID     				0x0880  // used to verify GD FPGA communications

// watch dog related
#define WATCHDOG_1V_KICK        	16000
#define WATCHDOG_3V_KICK        	48000
#define WATCHDOG_1V_FEEDBACK 		1000
#define WATCHDOG_3V_FEEDBACK 		3000

#define WATCHDOG_ERR_MAX     	    5
#define WATCHDOG_FEEDBACK_MARGIN    200

#define EF_SET_MIN					7518  /* 1V */
#define EG_SET_MIN                  150   /* -79V */
#define EK_RD_MIN_FOR_GRID_ON       900	  /* min 5kv to turn grid on */			    


int main(void) {

  
  // ---- Configure the dsPIC ADC Module ------------ //
  ADPCFG = 0xffff;             // all are digital I/O

   // disable uart for now
  // Begin UART operation
  _U1TXIF = 0;	// Clear the Transmit Interrupt Flag
  _U1TXIE = 0;	// Enable Transmit Interrupts
  _U1RXIF = 0;	// Clear the Recieve Interrupt Flag
  _U1RXIE = 0;	// Enable Recieve Interrupts
  
  U1MODEbits.UARTEN = 0;	// And turn the peripheral on
//  PIN_RS422_DE = OLL_RS422_DE_ENABLE_RS422_DRIVER;  // Enable the U69-RS422 Driver output (The reciever is always enabled)

 // global_data_A35975.control_state = STATE_STARTUP;
   
  control_state = STATE_START_UP;
   
  InitializeA35975();  

  T1CONbits.TON = 1;

  analog_sets[ANA_SET_EK].ip_set = 0;
  analog_sets[ANA_SET_EF].ip_set = 0;
  analog_sets[ANA_SET_EG].ip_set = 0; 
  _CONTROL_NOT_CONFIGURED = 1;
  _CONTROL_NOT_READY = 1;
  
  _STATUS_GD_HTR_NOT_READY = 1;
  _STATUS_GD_HTR_NOT_ENABLED = 1;
  _STATUS_GD_HV_NOT_ENABLED = 1;
  _STATUS_GD_TOP_NOT_ENABLED = 1;
  _STATUS_GD_TRIG_NOT_ENABLED = 1;


  while (1) {
  
    DoStateMachine();
    Do10msTicToc();  // Execute 10mS timed functions if the 10ms Timer has rolled
    ETMCanSlaveDoCan();

  }
}



void DoStateMachine(void) {
  static unsigned int delay_counter; 
  static unsigned int ef_ref_step;
  
  #ifdef LOOPBACK_TEST
    unsigned char txData[8] = {1, 2, 3, 4, 7, 8, 5, 6};
  #endif
  

  if ((control_state & 0x7f) > STATE_SYSTEM_HV_OFF) {

  #ifndef TEST_MODE_BYP_FIBER_OFF
    if (PIN_OPT_HV_ENABLE1_INPUT != ILL_OPT_HV_ENABLE) {
        LogHvControl(0);
    }
    else if ((PIN_OPT_TRIG_ENABLE_INPUT != ILL_OPT_TRIG_ENABLE) && ((control_state & 0x7f) > STATE_SYSTEM_TRIG_OFF)) {
    	LogTrigControl(0);    	
    }  
  #endif
  }  	

    
  if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
  	  _STATUS_GD_HV_DISABLE = 1;
  	  if ((control_state & 0x7f) > STATE_SYSTEM_HV_OFF)  LogHvControl(0);

  }
  else
  	  _STATUS_GD_HV_DISABLE	= 0;

  
  switch(control_state) {
    
  case STATE_START_UP:

    system_byte = 0;
    
    analog_reads[ANA_RD_IF].read_f_hi = IF_READ_MAX;
    analog_reads[ANA_RD_EC].read_f_lo = 2150; // 120/0.05555
    analog_reads[ANA_RD_24V].read_f_hi = 3960; // 3600 + 360, 10% more 24V
    analog_reads[ANA_RD_24V].read_f_lo = 3240; // 3600 - 360, 10% less 24V     
    delay_counter = 0;    

    #ifdef LOOPBACK_TEST
   	 PutResponseToBuffer(8, &txData[0]);
   #endif
   control_state = STATE_START_UP2;
    break;

  case STATE_START_UP2:
    if (_T1IF) {
		// 100ms Timer over flow 
		_T1IF = 0;
	    delay_counter++;
    }
	if (delay_counter > 10) {// 1s
        analog_reads[ANA_RD_EC ].read_m_lo = 1;
        analog_reads[ANA_RD_24V].read_m_lo = 1;
        analog_reads[ANA_RD_24V].read_m_hi = 1;
        
        delay_counter = 0;
        control_state = STATE_SYSTEM_HTR_OFF;
    }
  	break;
    
  case STATE_SYSTEM_HTR_OFF:
  	if (_CONTROL_NOT_CONFIGURED == 0) control_state = STATE_READY_FOR_HEATER;
    break;

  case STATE_READY_FOR_HEATER:
  	system_byte |= SYS_BYTE_LOGIC_READY;
    htd_timer_in_100ms = 100; // SYSTEM_WARM_UP_TIME;
    ef_ref_step = 0;
    software_skip_warmup = 1; /* no local htd for linac gun driver */
    delay_counter = 0;
    
 	// waiting htr on command from canopen
    if (analog_sets[ANA_SET_EF].ip_set >= EF_SET_MIN && htr_OVOC_rest_delay_timer_10ms == 0) {
    	  LogicHeaterControl(1);

          htr_OVOC_auto_reset_disable = 0;
    }

    break;

  case STATE_HEATER_STARTUP:
     if (_T1IF) {	 // runs every 100ms
     	_T1IF = 0; 
		if (htd_timer_in_100ms) htd_timer_in_100ms--;
        
                    
		  	// heater ramp up
		    if (ef_ref_step == 0) {
		    	ef_ref_step = 120; // around 512 steps for max Ef, (EF_SET_MAX >> 9);
		        if (ef_ref_step < 10) ef_ref_step = 10;
		        analog_sets[ANA_SET_EF].ip_set_alt = ef_ref_step;
		        analog_sets[ANA_SET_EF].ip_set_flag = 1;
		    }
		    else if (analog_sets[ANA_SET_EF].ip_set_flag) {	                      
	        	if (analog_reads[ANA_RD_IF].read_cur <= IF_READ_MAX_90P) {            
			    	analog_sets[ANA_SET_EF].ip_set_alt += ef_ref_step;
			        if (analog_sets[ANA_SET_EF].ip_set_alt >= analog_sets[ANA_SET_EF].ip_set) {
			        	analog_sets[ANA_SET_EF].ip_set_alt = analog_sets[ANA_SET_EF].ip_set;
			         	analog_sets[ANA_SET_EF].ip_set_flag = 0;
			        }   
		    	}
                if (htd_timer_in_100ms == 0) // 10s passed, check whether htr is shorted
                {
                	if (analog_sets[ANA_SET_EF].ip_set_alt < EF_SET_MIN)
                    {  // htr is shorted
						FaultIf(2);	// call OC handler
                        break;
                    }
                }
                
                if (analog_sets[ANA_SET_EF].ip_set_alt >= EF_SET_MIN)
				{
                	if (delay_counter < 200) 
                    {
	                    delay_counter++;
	                    if (delay_counter == 100)
	                    {  // 10s after Ef > 1V without OV/OC, only set once at 10s delay time
	                    	
	                        _STATUS_GD_HTR_NOT_READY = 0;
					        system_byte |= SYS_BYTE_HTR_WARMUP;        
							PIN_LED_WARMUP = !OLL_LED_ON;
	                    
	                    }
                    }
                }
                /*
	            else if (analog_reads[ANA_RD_IF].read_cur > IF_READ_MAX_95P) {
	            	if (analog_sets[ANA_SET_EF].ip_set_alt > ef_ref_step) {
	                	analog_sets[ANA_SET_EF].ip_set_alt -= ef_ref_step; 
	                }
	            }
               */
 		    }
        
           
        if (analog_sets[ANA_SET_EF].ip_set_flag) {
            SendHeaterRef(analog_sets[ANA_SET_EF].ip_set_alt);
        }
        else {
            analog_sets[ANA_SET_EF].ip_set_flag = 0;
            SendHeaterRef(analog_sets[ANA_SET_EF].ip_set);
		    // send htr ref and htr on cmd
			 //     htd_timer_in_100ms = SYSTEM_WARM_UP_TIME;
			 // 	PIN_LED_WARMUP = OLL_LED_ON;
	         //   analog_reads[ANA_RD_EF].read_m_lo = 1;
            htd_timer_in_100ms = 100; // 10s for heater to be stable
            control_state = STATE_WARM_UP;
            
			 
        }
    }       
    break;

  case STATE_WARM_UP:
     if (_T1IF) {
		if (htd_timer_in_100ms) htd_timer_in_100ms--;
		_T1IF = 0;
      }
	
    if (!htd_timer_in_100ms) {
		control_state = STATE_SYSTEM_HV_OFF;
        
	    htr_OVOC_count = 0;        
        // Enable EfUV, IFOC
        analog_reads[ANA_RD_EF].read_m_lo = 1;
            
		software_skip_warmup = 0;
        _STATUS_GD_HTR_NOT_READY = 0;
        
        system_byte |= SYS_BYTE_HTR_WARMUP;        
		PIN_LED_WARMUP = !OLL_LED_ON;
    } 

    break;
    
  case STATE_SYSTEM_HV_OFF:
	// wait canopen HV on command
        if (sdo_hv_bypass) {
			control_state = STATE_SYSTEM_PULSETOP_OFF;
       
        }
#ifndef TEST_MODE_BYP_FIBER_OFF
        else if (PIN_OPT_HV_ENABLE1_INPUT == ILL_OPT_HV_ENABLE && _SYNC_CONTROL_RESET_ENABLE == 0) 
#else
		else if (_SYNC_CONTROL_RESET_ENABLE == 0)
#endif        
			control_state = STATE_READY_FOR_HV;
   break;
    
  case STATE_READY_FOR_HV:
		// wait canopen HV on command
#ifndef TEST_MODE_BYP_FIBER_OFF
        if (PIN_OPT_HV_ENABLE1_INPUT == ILL_OPT_HV_ENABLE && _SYNC_CONTROL_RESET_ENABLE == 0) {
#else
        if (_SYNC_CONTROL_RESET_ENABLE == 0) {
#endif        
   			system_byte |= SYS_BYTE_LOGIC_READY;
    		delay_counter = 0;
            
            if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV == 0) LogHvControl(1);  // follow ethernet board to turn hv on
        }
        else {
        	control_state = STATE_SYSTEM_HV_OFF;
            system_byte &= ~SYS_BYTE_LOGIC_READY;
        }
        	 
   break;
    
  case STATE_HV_STARTUP:

      
      if (_T1IF) {
		// 100ms Timer over flow 
		_T1IF = 0;
    	delay_counter++;
      }

	  if (delay_counter > 10) {
	  	delay_counter = 0;
		control_state = STATE_HV_ON;
	  }
    
    break;

    
  case STATE_HV_ON:
      if (_T1IF) {
		// 100ms Timer over flow 
		_T1IF = 0;
    	delay_counter++;
      }

	  if (delay_counter > 10) {
	  	delay_counter = 0;
        system_byte |= SYS_BYTE_HV_DRIVEUP;
        
        analog_reads[ANA_RD_EK].read_m_lo = 1;
        
		control_state = STATE_SYSTEM_PULSETOP_OFF;
	  }
    
     break;
 
  case STATE_SYSTEM_PULSETOP_OFF:
    control_state = STATE_READY_FOR_PULSETOP;
     break;

  case STATE_READY_FOR_PULSETOP:
  	// wait for pulse on cmd
     system_byte |= SYS_BYTE_LOGIC_READY;
     delay_counter = 0;
     
     // check Ek >= 5kv before grid on
     if (analog_reads[ANA_RD_EK].read_cur >= EK_RD_MIN_FOR_GRID_ON && analog_sets[ANA_SET_EG].ip_set >= EG_SET_MIN)  
     		LogPulsetopControl(1);

     break;

  case STATE_PULSETOP_STARTUP:
 

     if (_T1IF) {
		// 100ms Timer over flow 
		_T1IF = 0;
    	delay_counter++;
      }

	  if (delay_counter > 10) {
	  	delay_counter = 0;
		control_state = STATE_PULSETOP_ON;
	  }
    
     break;
  
  case STATE_PULSETOP_ON:
 
 	  // send out top ref and top on cmd
     if (_T1IF) {
		// 100ms Timer over flow 
		_T1IF = 0;
    	delay_counter++;
      }

	  if (delay_counter > 2) {
	  	delay_counter = 0;        
        
		control_state = STATE_SYSTEM_TRIG_OFF;
	  }
    
     break;

  case STATE_SYSTEM_TRIG_OFF:

#ifndef TEST_MODE_BYP_FIBER_OFF
  	 if (PIN_OPT_TRIG_ENABLE_INPUT == ILL_OPT_TRIG_ENABLE)
#endif     
	 	control_state = STATE_READY_FOR_TRIG;

     break;


  case STATE_READY_FOR_TRIG:
  	// wait for trig on cmd
#ifndef TEST_MODE_BYP_FIBER_OFF
  	 if (PIN_OPT_TRIG_ENABLE_INPUT == ILL_OPT_TRIG_ENABLE) 
#else
	 if (1)     
#endif
     {     
   		system_byte |= SYS_BYTE_LOGIC_READY;
    	delay_counter = 0;
        
        LogTrigControl(1);  // turn trig on automatically
     }
     else {
     	system_byte &= ~SYS_BYTE_LOGIC_READY;
	 	control_state = STATE_SYSTEM_TRIG_OFF;
     }
     break;

  case STATE_TRIG_STARTUP:
 
 	  // send out top ref and top on cmd
     if (_T1IF) {
		// 100ms Timer over flow 
		_T1IF = 0;
    	delay_counter++;
      }

	  if (delay_counter > 2) {
	  	delay_counter = 0;
		control_state = STATE_TRIG_ON;
	  }
    
     break;
  
  case STATE_TRIG_ON:
     // highest state, can be turned off by user or a fault
     PIN_LED_GD_READY = OLL_LED_ON;
     PIN_OPT_GD_READY  = OLL_OPT_GD_READY;
     PIN_CAN_TRIGGER_ENABLE = OLL_CAN_TRIGGER_ENABLE; 

	 _CONTROL_NOT_READY = 0;
     break;

        
  case STATE_FAULT_COLD_FAULT:
  	 if (htr_OVOC_auto_reset_disable == 0 && htr_OVOC_count > 0) {
     	htr_OVOC_rest_delay_timer_10ms = 500;
        sdo_logic_reset = 1;  // htr OVOC autoreset
     }
     else if (_SYNC_CONTROL_RESET_ENABLE && htr_OVOC_count == 0)
     	sdo_logic_reset = 1;
  
  	 if (sdo_logic_reset) {
     	sdo_logic_reset = 0;
        DoFaultClear(FAULTS_TYPE_SYSTEM_CONTROL,   0xffff);
       	DoFaultClear(FAULTS_TYPE_SOFTWARE,     0xffff);
        DoFaultClear(FAULTS_TYPE_DIGI_FROM_FPGAID, 0xffff);
        
        // send reset to fpga board
		ResetFPGA();
        // clear fault status on CAN
        _FAULT_REGISTER = _FAULT_REGISTER & 0x04; // _FAULT_GD_SW_HTR_OVOC; //FAULTS_SW_EFOV_IFOC;

        PIN_LED_SUM_FAULT = !OLL_LED_ON;
        
        
        control_state = STATE_START_UP;
     }
  	 break;
  case STATE_FAULT_HOT_FAULT:
    // waiting for reset cmd
	 if (_SYNC_CONTROL_RESET_ENABLE)
     	sdo_logic_reset = 1;

  	 if (sdo_logic_reset) {
     	sdo_logic_reset = 0;
        DoFaultClear(FAULTS_TYPE_SYSTEM_CONTROL,   0xffff);
        DoFaultClear(FAULTS_TYPE_SOFTWARE,         0xffff);
        DoFaultClear(FAULTS_TYPE_DIGI_FROM_FPGAID, 0xffff);
        
        // clear fault status on CAN
         _FAULT_REGISTER = 0;

        if (analog_reads[ANA_RD_EF].read_m_lo > 1) analog_reads[ANA_RD_EF].read_m_lo = 1;
        if (analog_reads[ANA_RD_EG].read_m_hi > 1) analog_reads[ANA_RD_EG].read_m_hi = 1;
        if (analog_reads[ANA_RD_EK].read_m_hi > 1) analog_reads[ANA_RD_EK].read_m_hi = 1;
        if (analog_reads[ANA_RD_EK].read_m_lo > 1) analog_reads[ANA_RD_EK].read_m_lo = 1;


        PIN_LED_SUM_FAULT = !OLL_LED_ON;        
        if (last_control_state < 0x80) control_state = last_control_state;
        else                           control_state = STATE_START_UP;
      }     
     break;
    

  default:
    // DPARKER throw an ERROR
    _FAULT_GD_SYS_FAULTS = 1;
    DoFaultRecord(FAULTS_TYPE_SYSTEM_CONTROL, FAULTS_SYS_LOGIC_STATE);
    break;
  }
  
  if (control_state < 0x80) last_control_state = control_state;
  
}

void InitializeA35975(void) {
  unsigned int n;

 
  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;

  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;

  
  // Init analog scaling for CAN bus
#ifdef USE_ENGINEERING_UNIT_ON_GUN_DRIVER
	CAN_scale_table[CAN_RD_EK  ].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAL_EK_RD/CAN_EK_SCALE);
	CAN_scale_table[CAN_RD_IKP ].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAL_IKP_RD/CAN_IKP_SCALE);
	CAN_scale_table[CAN_RD_EF  ].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAL_EF_RD/CAN_EF_SCALE);
	CAN_scale_table[CAN_RD_IF  ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_IF_RD/CAN_IF_SCALE);
	CAN_scale_table[CAN_RD_EG  ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EG_RD/CAN_EG_SCALE);
	CAN_scale_table[CAN_RD_EC  ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EC_RD/CAN_EC_SCALE);
	CAN_scale_table[CAN_RD_TEMP].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_TEMP_RD/CAN_TEMP_SCALE);

	CAN_scale_table[CAN_RD_EKSET ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EKSET/CAN_EKSET_SCALE);
	CAN_scale_table[CAN_RD_EFSET ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EFSET/CAN_EFSET_SCALE);
	CAN_scale_table[CAN_RD_EGSET ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EGSET/CAN_EGSET_SCALE);
	CAN_scale_table[CAN_SET_EKSET].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAN_EKSET_SCALE/CAL_EKSET);
	CAN_scale_table[CAN_SET_EFSET].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAN_EFSET_SCALE/CAL_EFSET);
	CAN_scale_table[CAN_SET_EGSET].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAN_EGSET_SCALE/CAL_EGSET);
    
    // all offsets are default to 0
   // CAN_scale_table[CAN_RD_EG    ].fixed_offset = ((double)(-80) / CAL_EG_RD);
   // CAN_scale_table[CAN_RD_EGSET ].fixed_offset = ((double)(-80)/ CAL_EGSET);
   // CAN_scale_table[CAN_SET_EGSET].fixed_offset = ((double)80/CAN_EGSET_SCALE);
#endif 

   // --------- BEGIN IO PIN CONFIGURATION ------------------

	  // Initialize Ouput Pin Latches BEFORE setting the pins to Output
	  PIN_CS_DAC_ENABLE = !OLL_CS_DAC_ENABLE;
	  PIN_CS_ADC_ENABLE = !OLL_CS_ADC_ENABLE;
	  PIN_CS_AUX_ENABLE = !OLL_CS_AUX_ENABLE;
	  
	  // LOGIC Output Pins
	  PIN_OPT_GD_READY  = !OLL_OPT_GD_READY;
	  PIN_CAN_HV_ENABLE = !OLL_CAN_HV_ENABLE;
	  PIN_CAN_PULSETOP_ENABLE = !OLL_CAN_PULSETOP_ENABLE;
	  
	  PIN_CAN_TRIGGER_ENABLE = !OLL_CAN_TRIGGER_ENABLE;
	  

	// MCP4822 DAC Output Pins
	  PIN_CS_MCP4822_ENABLE = !OLL_CS_MCP4822_ENABLE;
	  PIN_LDAC_MCP4822_ENABLE = !OLL_LDAC_MCP4822_ENABLE;

	  
	// UART TX enable
	  PIN_RS422_DE = !OLL_RS422_DE_ENABLE_RS422_DRIVER;


	// LED Indicator Output Pins
	  PIN_LED_24DC_OK = OLL_LED_ON;  
	  PIN_LED_LAST_PULSE_GOOD = !OLL_LED_ON;
	  PIN_LED_GD_READY = !OLL_LED_ON;  
	  
	  PIN_LED_HV_ENABLE = !OLL_LED_ON;  
	  PIN_LED_AC_ON = !OLL_LED_ON;  
	//  PIN_LED_LAST_PULSE_FAIL = !OLL_LED_ON;

	  PIN_LED_WARMUP = !OLL_LED_ON;
	  PIN_LED_SUM_FAULT = !OLL_LED_ON;

 
   // Configure T1 Inetrrupt
  _T1IP   = 5;



  // Initialize all I/O Registers
  TRISA = A35975_TRISA_VALUE;
  TRISB = A35975_TRISB_VALUE;
  TRISC = A35975_TRISC_VALUE;
  TRISD = A35975_TRISD_VALUE;
  TRISF = A35975_TRISF_VALUE;
  TRISG = A35975_TRISG_VALUE;

    



#if 0
  // Config Input Pins
  // LOGIC Input Pins
  TRIS_PIN_CS_DAC_ENABLE_INPUT    = TRIS_INPUT_MODE; // not used input
  
  
  TRIS_PIN_OPT_HV_ENABLE1_INPUT   = TRIS_INPUT_MODE;
  TRIS_PIN_OPT_HV_ENABLE2_INPUT   = TRIS_INPUT_MODE;
  
  TRIS_PIN_OPT_TRIG_ENABLE_INPUT = TRIS_INPUT_MODE;
  TRIS_PIN_TEST_PULSE_GOOD1_INPUT   = TRIS_INPUT_MODE;
  TRIS_PIN_TEST_PULSE_GOOD2_INPUT   = TRIS_INPUT_MODE;
#endif

  for (n = 0; n < 2; n++) {
    PIN_LED_24DC_OK 		 = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_LAST_PULSE_GOOD  = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_GD_READY         = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_HV_ENABLE        = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_AC_ON            = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_LAST_PULSE_FAIL  = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_WARMUP           = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_SUM_FAULT        = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

   
  }
	// LED Indicator Output Pins, keep 24DC on
	  PIN_LED_24DC_OK = OLL_LED_ON;  
      
  // config SPI1 for Gun Driver
   ConfigureSPI(1, (A35975_SPI1CON_VALUE & A35975_SPI1CON_CLOCK), 0, A35975_SPI1STAT_VALUE, 1000000, FCY_CLK);  

  
  // ----------- Configure Interupts -------------- //


  // Configure UART Interrupts
  _U1RXIE = 0;
  _U1RXIP = 3;
  
  _U1TXIE = 0;
  _U1RXIP = 3;


  


 // ---------- Configure Timers ----------------- //

  // Configure TMR1
  T1CON = A35975_T1CON_VALUE;
  PR1 = A35975_PR1_VALUE;  
  TMR1 = 0;
  _T1IF = 0;



  // Configure TMR4
  T4CON = A35975_T4CON_VALUE;


  // Initialize TMR5
  PR5   = A35975_PR5_VALUE;
  TMR5  = 0;
  _T5IF = 0;
  _T5IP = 5;
  T5CON = A35975_T5CON_VALUE;



  #if 0
  // --------------- Initialize U44 - LTC2656 ------------------------- //
  U44_LTC2656.pin_cable_select = _PIN_RD15;
  U44_LTC2656.pin_dac_clear = _PIN_RB15;
  U44_LTC2656.pin_load_dac = _PIN_RD14;
  U44_LTC2656.pin_por_select = _PIN_NOT_CONNECTED;
  U44_LTC2656.por_select_value = 0;
  U44_LTC2656.spi_port = ETM_SPI_PORT_1;

  SetupLTC2656(&U44_LTC2656);
  #endif 


  ResetAllFaults();

  
//  command_string.data_state == COMMAND_BUFFER_EMPTY;
  
    // Initialize the Can module
  ETMCanSlaveInitialize();


  
}







/////////////////////////////////////////////////////////////////////////
// LogicHeaterControl() 
// turn on/off heater 
//
void LogicHeaterControl(unsigned char turnon)
{
    
    if (turnon) {
    	if (control_state == STATE_READY_FOR_HEATER) {
         	SetDacChannel(ANA_SET_HTR_ON, 0xffff);
			control_state = STATE_HEATER_STARTUP; 
            
            analog_sets[ ANA_SET_HTR_ON ].ip_set = 0xffff;

            system_byte |= SYS_BYTE_HTR_ON;
            system_byte &= ~SYS_BYTE_LOGIC_READY; 
            
            htd_timer_in_100ms = 100;  //SYSTEM_WARM_UP_TIME;
			PIN_LED_WARMUP = OLL_LED_ON; 
            
            _STATUS_GD_HTR_NOT_ENABLED = 0;
            
	        analog_reads[ANA_RD_EF].read_m_hi = 1;
	        analog_reads[ANA_RD_IF].read_m_hi = 1;
            
        }
        
    }
    else {
    	LogHvControl(0);
        SetDacChannel(ANA_SET_HTR_ON,      0);

        analog_sets[ ANA_SET_HTR_ON      ].ip_set = 0;   
        
        SendHeaterRef(0);   
        
        PIN_LED_WARMUP = !OLL_LED_ON;
        htd_timer_in_100ms = SYSTEM_WARM_UP_TIME;
                
        sdo_hv_bypass = 0;  // reset hv bypass when htr is off
        
        analog_reads[ANA_RD_EF].read_m_hi = 0;
        analog_reads[ANA_RD_EF].read_m_lo = 0;
        analog_reads[ANA_RD_IF].read_m_hi = 0;
        analog_reads[ANA_RD_IF].read_m_lo = 0;

		last_control_state =  STATE_START_UP;

        if (control_state & 0x80)
        	control_state = STATE_FAULT_COLD_FAULT;
        else
        	control_state = STATE_START_UP;
            
        system_byte &= ~(SYS_BYTE_HTR_ON | SYS_BYTE_HTR_WARMUP);
        _STATUS_GD_HTR_NOT_ENABLED = 1;
        _STATUS_GD_HTR_NOT_READY = 1;

    }     

}
/////////////////////////////////////////////////////////////////////////
// LogHvControl() 
// turn on/off hv 
//
void LogHvControl(unsigned char turnon)
{
	if (turnon) {
    	if (control_state == STATE_READY_FOR_HV) {
            
            // send out hv ref and hv on cmd
      		SendHvRef(analog_sets[ANA_SET_EK].ip_set);

         	SetDacChannel(ANA_SET_HV_ON, 0xffff);
            PIN_CAN_HV_ENABLE = OLL_CAN_HV_ENABLE;
            PIN_LED_HV_ENABLE = OLL_LED_ON;

			control_state = STATE_HV_STARTUP;       

            analog_sets[ ANA_SET_HV_ON ].ip_set = 0xffff;   
            
            analog_reads[ANA_RD_EK].read_m_hi = 1;   

            system_byte |= SYS_BYTE_HV_ON;
            system_byte &= ~SYS_BYTE_LOGIC_READY;  
            
    		_STATUS_GD_HV_NOT_ENABLED = 0;
        }
        
    }
    else {
    	LogPulsetopControl(0);
        SetDacChannel(ANA_SET_HV_ON,       0);
        
        analog_sets[ ANA_SET_HV_ON       ].ip_set = 0;      

        system_byte &= ~(SYS_BYTE_HV_ON | SYS_BYTE_HV_DRIVEUP);
        PIN_LED_HV_ENABLE = !OLL_LED_ON;
		PIN_CAN_HV_ENABLE = !OLL_CAN_HV_ENABLE;
        
        analog_reads[ANA_RD_EK].read_m_hi = 0;   
        analog_reads[ANA_RD_EK].read_m_lo = 0;   
        
        ekuv_timeout_10ms = 0;

		if ((last_control_state > STATE_SYSTEM_HV_OFF) && (last_control_state < 0x80)) last_control_state =  STATE_SYSTEM_HV_OFF;

        if (control_state & 0x80)
        	control_state = STATE_FAULT_HOT_FAULT;
        else if (control_state > STATE_SYSTEM_HV_OFF)
        	control_state = STATE_SYSTEM_HV_OFF;
        
        _STATUS_GD_HV_NOT_ENABLED = 1;    
        
    }     

}

/////////////////////////////////////////////////////////////////////////
// LogPulsetopControl() 
// turn on/off pulsetop 
//
void LogPulsetopControl(unsigned char turnon)
{
	if (turnon) {
    	if (control_state == STATE_READY_FOR_PULSETOP) {
        
        	// send out top ref and top on cmd
     		SendPulsetopRef(analog_sets[ANA_SET_EG].ip_set);

         	SetDacChannel(ANA_SET_PULSETOP_ON, 0xffff);
            
            PIN_CAN_PULSETOP_ENABLE = OLL_CAN_PULSETOP_ENABLE;

			control_state = STATE_PULSETOP_STARTUP;       

            analog_sets[ ANA_SET_PULSETOP_ON ].ip_set = 0xffff;      

            analog_reads[ANA_RD_EG].read_m_hi = 1;   

            system_byte |= SYS_BYTE_PULSETOP_ON;
            system_byte &= ~SYS_BYTE_LOGIC_READY;  
            
            _STATUS_GD_TOP_NOT_ENABLED = 0;   
        }
        
    }
    else {
		LogTrigControl(0);
        SetDacChannel(ANA_SET_PULSETOP_ON, 0);

        analog_sets[ ANA_SET_PULSETOP_ON ].ip_set = 0;      

        system_byte &= ~SYS_BYTE_PULSETOP_ON;
        PIN_CAN_PULSETOP_ENABLE = !OLL_CAN_PULSETOP_ENABLE;

        
		if ((last_control_state > STATE_SYSTEM_PULSETOP_OFF) && (last_control_state < 0x80)) last_control_state = STATE_SYSTEM_PULSETOP_OFF;

        if (control_state & 0x80)
        	control_state = STATE_FAULT_HOT_FAULT;
        else if (control_state > STATE_SYSTEM_PULSETOP_OFF)
        	control_state = STATE_SYSTEM_PULSETOP_OFF;
        
        _STATUS_GD_TOP_NOT_ENABLED = 1;   
    }     

}

/////////////////////////////////////////////////////////////////////////
// LogTrigControl() 
// turn on/off trig 
//
void LogTrigControl(unsigned char turnon)
{
    
	if (turnon) {
    	if (control_state == STATE_READY_FOR_TRIG) {
         	SetDacChannel(ANA_SET_TRIG_ON, 0xffff);
			control_state = STATE_TRIG_STARTUP;       

            analog_sets[ ANA_SET_TRIG_ON ].ip_set = 0xffff;      

            analog_reads[ANA_RD_EG].read_m_hi = 1;  // make sure faults are enabled 
            analog_reads[ANA_RD_EK].read_m_hi = 1;   
            analog_reads[ANA_RD_EK].read_m_lo = 1;   

            system_byte |= SYS_BYTE_TRIG_ON;
            system_byte &= ~SYS_BYTE_LOGIC_READY; 
            _STATUS_GD_TRIG_NOT_ENABLED = 0;    
       }
        
    }
    else {
        SetDacChannel(ANA_SET_TRIG_ON, 0);

        analog_sets[ ANA_SET_TRIG_ON     ].ip_set = 0;      

        system_byte &= ~(SYS_BYTE_TRIG_ON | SYS_BYTE_LOGIC_READY);
		PIN_LED_GD_READY = !OLL_LED_ON;
        PIN_OPT_GD_READY = !OLL_OPT_GD_READY;        
        PIN_CAN_TRIGGER_ENABLE = !OLL_CAN_TRIGGER_ENABLE;
        
        _CONTROL_NOT_READY = 1; 

		if ((last_control_state > STATE_SYSTEM_TRIG_OFF) && (last_control_state < 0x80)) last_control_state = STATE_SYSTEM_TRIG_OFF;

        if (control_state & 0x80)
        	control_state = STATE_FAULT_HOT_FAULT;
        else if (control_state > STATE_SYSTEM_TRIG_OFF)
        	control_state = STATE_SYSTEM_TRIG_OFF;
        
        _STATUS_GD_TRIG_NOT_ENABLED = 1;    
    }     

}

/////////////////////////////////////////////////////////////////////////
// SendHeaterRef() 
//  
//
void SendHeaterRef(unsigned int bits)
{	
	// need to check max, min
    SetDacChannel(ANA_SET_EF, bits);

}

/////////////////////////////////////////////////////////////////////////
// SendHvRef() 
//  
//
void SendHvRef(unsigned int bits)
{	
	// need to check max, min
    SetDacChannel(ANA_SET_EK, bits);

}

/////////////////////////////////////////////////////////////////////////
// SendPulsetopRef() 
//  
//
void SendPulsetopRef(unsigned int bits)
{	
	// need to check max, min
    SetDacChannel(ANA_SET_EG, bits);

}
/////////////////////////////////////////////////////////////////////////
// SendWatchdogRef() 
//  
//
void SendWatchdogRef(unsigned int bits)
{	
	// need to check max, min
    SetDacChannel(ANA_SET_WDOG, bits);
	analog_sets[ ANA_SET_WDOG ].ip_set = bits;  // for debug
    
}







void Do10msTicToc(void) {

  static unsigned char gd_read_ptr;  // cycle the readbacks from gd fpga board
  static unsigned long read_cycles = 0;
  static unsigned char watch_dog_timer = 0;

  static unsigned int sec_count = 0;

  unsigned long temp;
  unsigned int i, bit_value;
  
  /*
    Certain functions need to happen at regular interval for the system to work

    Thyratron PIDs - The gain and phase of the PID control loop is a function of it's execution frequency therefor it must be updated at a regular interval
    Analog Filters - The filter response is function of the execution frequency so they must be executed at a regular interval

    DAC updates - The DAC must be regularly.  Durring HV ON this should happen AFTER a pulse so that the SPI bus is not corrupted by EMI
    If the state is not in HV_ON or the system is pulsing at a very low freqeuncy, DAC updates must be handeled by this function.
    
    Calculating the PRF

    Other timing functions like flashing LEDs
  */

  last_known_action = LAST_ACTION_DO_10MS;
  
  if (_POR) {
    debug_status_register |= STATUS_POR_RESET;
    // _POR = 0;
  }
  if (_EXTR) {
    debug_status_register |= STATUS_EXTERNAL_RESET;
    //_EXTR = 0;
  }
  if (_SWR) {
    debug_status_register |= STATUS_SOFTARE_RESET;
    //_SWR = 0;
  }
  if (_BOR) {
    debug_status_register |= STATUS_BOR_RESET;
    //_BOR = 0;
  }
  if (_TRAPR) {
    debug_status_register |= STATUS_TRAPR_RESET;
    //_TRAPR = 0;
  }
  if (_WDTO) {
    debug_status_register |= STATUS_WDT_RESET;
    // _WDTO = 0;
  }
  if (_IOPUWR) {
    debug_status_register |= STATUS_IOPUWR_RESET;
    //_IOPUWR = 0;
  }


  ClrWdt();
  
  // update one value from gd fpga board, watchdog channel is handled by watchdog kicking
  if (gd_read_ptr < 12)	{	// don't care ch9, 13, 14
   	 
  	 temp = ReadAdcChannel(gd_read_ptr);

     if (gd_read_ptr <= 8) {
     	analog_reads[ gd_read_ptr ].read_cur = temp;
    	analog_reads[ gd_read_ptr ].read_cnt++;
        CheckAnalogLimits(gd_read_ptr);
     }    
     else if (gd_read_ptr >= 10 && gd_read_ptr <= 11) {	// only care about wdog and arc faults 
			i = gd_read_ptr - 9;
            if (temp <= 2500)   faults_from_ADC |= (1 << i);
			else				faults_from_ADC &= ~(1 << i);
            
            if (digi_reads[i].action_code < 99) {
	     		digi_reads[i].state = (temp > 2500);
                
	            if (temp <= 2500) {
	                DoFaultRecord(FAULTS_TYPE_DIGI_FROM_FPGAID, (1 << i)); // mapped to fpgaID bit 1 & 2
                }
        	}
     }
     
  }
  else {
     
     temp = ReadFPGAID(); 
	 // check if the ID read is valid
     if ((temp & 0x03c0) == (FPGA_ID & 0x03c0)) {  // check FPGA major version only.  0x40 for the low byte, two bits stay 0 for the 2nd low byte, xxxxxx00 01000000
     	temp >>= 16;

        fpga_ASDR = (unsigned int)temp;
        // handle digi faults from fpgaid
        for (i = 0; i < 16; i++)
        {
        	bit_value = (temp & (1 << i)) > 0;
            if (digi_reads[ i + DIGI_ID_ARC_COUNT].action_code < 99) {
	        	digi_reads[ i + DIGI_ID_ARC_COUNT].state = bit_value;
	        	if (bit_value)
	             	DoFaultRecord(FAULTS_TYPE_DIGI_FROM_FPGAID, (1 << i));
	           	else if (digi_reads[ i + DIGI_ID_ARC_COUNT].action_code == 0)    
	            	DoFaultClear(FAULTS_TYPE_DIGI_FROM_FPGAID, (1 << i));
            }
               
        }
         
        PIN_LED_AC_ON = OLL_LED_ON;    
        
     }
     else { // declare a fault
         PIN_LED_AC_ON = !OLL_LED_ON;    
    	 DoFaultRecord(FAULTS_TYPE_SYSTEM_CONTROL, FAULTS_SYS_FPGAID);
     }
     
  }
  
  if (gd_read_ptr == 8)	{
    gd_read_ptr = 9; // bypass ch9
  }
  else if (gd_read_ptr == 11) {
    gd_read_ptr = 14; // bypass ch12, 13, 14
  } 
  
  gd_read_ptr = (gd_read_ptr + 1) & 0x000f;	 // 0 to 15 for gd_read_ptr
  
  if (!gd_read_ptr)  read_cycles++;	 
  
  if (_T5IF) {
    _T5IF = 0;
    //10ms roll has occured
    sec_count++;
    if (sec_count > 200) {
        read_cycles_in_2s = read_cycles;
        read_cycles = 0;
        sec_count = 0;
    }
        
    if ((control_state & 0x7f) > STATE_READY_FOR_HEATER) {
	    if (_CONTROL_CAN_COM_LOSS)	{
 	    	 // declare fault, turn all off
	    	 DoFaultRecord(FAULTS_TYPE_SYSTEM_CONTROL, FAULTS_SYS_CAN_TIMEOUT);
             _FAULT_CAN_COMMUNICATION_LATCHED = 1;
	    }
    }

    watch_dog_timer++;
    if (watch_dog_timer >= 3) 
    {
    	DoFpgaWatchdog();  // tickle the watchdog every 30ms
        watch_dog_timer = 0;
    }
    
 	// record debug data for CAN bus    
    local_debug_data.debug_0 = analog_reads[ANA_RD_EK].read_cur;
    local_debug_data.debug_1 = analog_reads[ANA_RD_IKA].read_cur;
    local_debug_data.debug_2 = analog_reads[ANA_RD_IKP].read_cur;
    local_debug_data.debug_3 = analog_reads[ANA_RD_EF].read_cur;
 
    local_debug_data.debug_4 = analog_reads[ANA_RD_IF].read_cur;;
    local_debug_data.debug_5 = analog_reads[ANA_RD_EG].read_cur;;
    local_debug_data.debug_6 = analog_reads[ANA_RD_EC].read_cur;;
    local_debug_data.debug_7 = analog_reads[ANA_RD_TEMP].read_cur;;
    local_debug_data.debug_8 = control_state;
    local_debug_data.debug_9 = htd_timer_in_100ms;
    

    local_debug_data.debug_D = analog_sets[ANA_SET_EK].ip_set;
    local_debug_data.debug_E = analog_sets[ANA_SET_EF].ip_set;
    local_debug_data.debug_F = analog_sets[ANA_SET_EG].ip_set;

   
    if (ekuv_timeout_10ms) {
    	if (ekuv_timeout_10ms < 1000) ekuv_timeout_10ms++;
    }
        
   
    if (ek_ref_changed_timer_10ms) ek_ref_changed_timer_10ms--;
    if (ef_ref_changed_timer_10ms) ef_ref_changed_timer_10ms--;
    if (eg_ref_changed_timer_10ms) eg_ref_changed_timer_10ms--;

	if (htr_OVOC_rest_delay_timer_10ms) htr_OVOC_rest_delay_timer_10ms--; 

    
	if ((_FAULT_REGISTER &  0x04 /*_FAULT_GD_SW_HTR_OVOC */) && (htr_OVOC_count == 0)) {
	  	if (_SYNC_CONTROL_RESET_ENABLE  && ((faults_reg_software & FAULTS_SW_EFOV_IFOC) == 0))
	  	   
	       _FAULT_REGISTER = _FAULT_REGISTER & (~0x04/* _FAULT_GD_SW_HTR_OVOC */);
	}
        
        
    led_pulse_count = ((led_pulse_count + 1) & 0b00111111);	 // 640ms
    if (led_pulse_count == 0) {
      // 10ms * 16 counter has ocurred
      // Flash the LED - NOTE "PIN_MAIN_CONTACTOR_CLOSE = !PIN_MAIN_CONTACTOR_CLOSE" was causing any changes made in Port F durring interrupt to be overwritten
      if (PIN_LED_LAST_PULSE_GOOD) {
			PIN_LED_LAST_PULSE_GOOD = 0;
      } else {
			PIN_LED_LAST_PULSE_GOOD = 1;
      }  
      
    }
    
    
  } 
}



#if 0 // copied to ReadAdcChannel function
/////////////////////////////////////////////////////////////////////////
// SetupGDadc() setup the ADC chip on the Gun Driver board
// 
//
static void SetupAdc(void)
{
	unsigned int temp;
 
    temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
    temp |= PIN_CS_ADC_ENABLE_BIT;
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

 
 //   temp = 0x78;  // clock mode 11(single conv), Ref mode 10 (internal, always on), no differential inputs 
    temp = 0x74;  // clock mode 11(single conv), Ref mode 01 (external single ended), no differential inputs 
 //   temp <<= 8; // high 8 bit out

 //  	spiSend((~mode) & 0xFF00); 
    temp = ~temp; 
    SendAndReceiveSPI(temp & 0x00ff, ETM_SPI_PORT_1);     // send the setup byte out

    temp = LATD & 0x1fff; // clear all CS bits
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

		
  	return;    
}
#endif


#ifdef DEMO
/////////////////////////////////////////////////////////////////////////
// CalculateDemoAdc() 
//  
//
static unsigned CalculateDemoAdc(unsigned chan)
{
	unsigned ret = 0;
    double temp = 0;
    static double little_change = -0.002; 
    
   
    
	switch (chan) {
	case ANA_RD_EK:
    	if ((control_state & 0x7f) >= STATE_HV_STARTUP) {
        	temp = (double)analog_sets[ANA_SET_EK].ip_set * (-0.0003333);	// ref v
            temp += temp * little_change;
            temp /= -0.005555;
        }
     break;
	case ANA_RD_IKA:
    	if ((control_state & 0x7f) >= STATE_HV_STARTUP) {
        	temp = 202.2;
			temp += temp *little_change;            
        	temp = temp/0.001667;
        }
     break;
	case ANA_RD_IKP:
    	if ((control_state & 0x7f) >= STATE_HV_STARTUP) {
        	temp = 602.5;
			temp += temp * little_change;            
        	temp = temp/0.277;
         }
     break;
	case ANA_RD_EF:
    //	if ((control_state & 0x7f) >= STATE_HEATER_STARTUP) {
        	temp = analog_sets[ANA_SET_EF].ip_set_flag? analog_sets[ANA_SET_EF].ip_set_alt : analog_sets[ANA_SET_EF].ip_set;
        	temp *= 0.000133;	// ref v
			temp += temp * little_change;            
            temp /= 0.00222;
    //    }
     break;
	case ANA_RD_IF:
    	if ((control_state & 0x7f) >= STATE_HEATER_STARTUP) {
        	temp = 2.5;
			temp += temp * little_change;
            temp /= 1.667e-3;
        }
     break;
    case ANA_RD_EG:
    	if ((control_state & 0x7f) < STATE_PULSETOP_STARTUP) {
        	temp = 80;
            temp /= 0.1111;
        }           
        else {
        	temp = (double)analog_sets[ANA_SET_EG].ip_set * 0.00666 - 80;  // -80 offset
			temp += temp * little_change;            
            temp += 80;
            temp /= 0.1111;
        }
     break;
	case ANA_RD_EC:
            temp = -150.5;
			temp += temp * little_change;
            temp /= -55.55e-3;
     break;
	case ANA_RD_24V:
    	    temp = 24;
			temp += temp * little_change;            
            temp /= 0.00666;            
     break;
    case ANA_RD_TEMP:
    	    temp = 40;
			temp += temp * little_change; 
            temp /= 0.0133;           
     break;
     
	default:
    	temp = 0xfff; // set high for no fault
     break;
    }

    if (temp > 0xfff) temp = 0xfff;
    else if (temp < 0) temp = 0;
    
    ret = (unsigned)(temp + 0.5);

    little_change += 0.001;
    if (little_change > 0.002) little_change = -0.002; // change between -0.002 to 0.002
 	return (ret);
}
#endif
/////////////////////////////////////////////////////////////////////////
// ReadAdcChannel() 
// read ADC one channel 
//
static unsigned ReadAdcChannel(unsigned chan)
{
	unsigned int temp, ret, adc_val1, adc_val2;
 
 	// enable ADC chip select
    temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
    temp |= PIN_CS_ADC_ENABLE_BIT;
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

    // always reset ADC mode first
    temp = 0x10;  // clock mode 11(single conv), Ref mode 01 (external single ended), no differential inputs 
    temp = ~temp; 
    SendAndReceiveSPI(temp & 0x00ff, ETM_SPI_PORT_1);     // send the setup byte out

    // always setup ADC mode second
    temp = 0x74;  // clock mode 11(single conv), Ref mode 01 (external single ended), no differential inputs 
    temp = ~temp; 
    SendAndReceiveSPI(temp & 0x00ff, ETM_SPI_PORT_1);     // send the setup byte out

    // send out conv command
    temp = 0x80 | (chan << 3) | 0x06; // conv enable, no scan, no temp measurement
	temp = ~temp;
    ret = SendAndReceiveSPI(temp & 0x00ff, ETM_SPI_PORT_1);     // send the setup byte out

    // get the msb
    adc_val1 = ~SendAndReceiveSPI(0xffff, ETM_SPI_PORT_1) & 0x00ff;
	adc_val2 = ~SendAndReceiveSPI(0xffff, ETM_SPI_PORT_1) & 0x00ff;
    
	// clear all CS bits
    temp = LATD & 0x1fff; 
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate


    ret = ((adc_val1 << 8) | adc_val2) & 0x0fff;

#ifdef DEMO
	// calculate the channel readback
    ret = CalculateDemoAdc(chan);
#endif    

  	return (ret); 
}


/////////////////////////////////////////////////////////////////////////
// SetDacChannel() 
// set DAC one channel, used 8 bit SPI instead of 16bit, otherwise,
// SPI would get a reset when bit config is changed.
//
static void SetDacChannel(unsigned chan, unsigned setvalue)
{
	unsigned int temp, ret;
 
 	if (chan >= 8) return;
    if (setvalue > 65535) setvalue = 65535;
    
 	// enable DAC chip select
    temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
    temp |= PIN_CS_DAC_ENABLE_BIT;
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

    temp = 0x30 | chan; // conv enable, no scan, no temp measurement
 //   temp <<= 8;  // 8 bit out
	temp = ~temp;
    ret = SendAndReceiveSPI(temp, ETM_SPI_PORT_1);     // send the setup byte out

    // set setvalue out
    temp = (setvalue >> 8) & 0x00ff;
    temp = ~temp;
    ret  = SendAndReceiveSPI(temp, ETM_SPI_PORT_1);	   // high setbyte sent out first
    temp = setvalue & 0x00ff;
    temp = ~temp;
    ret  = SendAndReceiveSPI(temp, ETM_SPI_PORT_1);
    
	// clear all CS bits
    temp = LATD & 0x1fff; 
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);   // Wait for the cable select signal to propagate

    ret = ret; // used		
  	return;    
}

/////////////////////////////////////////////////////////////////////////
// ReadFPGAID() 
// read FPGA ID, high 16 bits are fault information, low 16 bits are rev information. 
// used 8 bit SPI instead of 16bit, otherwise, SPI would get a reset when bit config is changed.
//
static unsigned long ReadFPGAID(void)
{
    unsigned i;
    unsigned int temp;
    unsigned long ret = 0;
 
 	// enable Aux chip select
    temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
    temp |= PIN_CS_AUX_ENABLE_BIT;
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

    for (i = 0; i < 4; i++)
    {
    	ret <<= 8;
    	ret |= (~SendAndReceiveSPI(0xffff, ETM_SPI_PORT_1) & 0x00ff);
	}    
	// clear all CS bits
    temp = LATD & 0x1fff; 
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

#ifdef DEMO
    ret = FPGA_ID;
#endif       

    return (ret);

}

/////////////////////////////////////////////////////////////////////////
// ResetFPGA() 
// send a reset to FPGA board 
//
void ResetFPGA(void)
{
    unsigned int temp;
 
 	// enable ADC chip select
    temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
    temp |= PIN_CS_ALL_ENABLE_BITS;
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

	// clear all CS bits
    temp = LATD & 0x1fff; 
    LATD = temp;
    __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

    return;
}

/////////////////////////////////////////////////////////////////////////
// DoFpgaWatchdog() 
// send a watchdog DAC to fpga board 
//
static void DoFpgaWatchdog(void)
{
    static unsigned char kicked_1v = 0;
    static unsigned error_count = 0;
    int temp, counts;
    
    
    counts = kicked_1v? WATCHDOG_1V_FEEDBACK : WATCHDOG_3V_FEEDBACK;
    temp = (int)ReadAdcChannel(15);
    
	#ifndef DEMO
    if ((temp < (counts - WATCHDOG_FEEDBACK_MARGIN)) || (temp > (counts + WATCHDOG_FEEDBACK_MARGIN))) {
    	error_count++;
        if (error_count >= WATCHDOG_ERR_MAX) {            	
     		DoFaultRecord(FAULTS_TYPE_SYSTEM_CONTROL, FAULTS_SYS_FPGA_WATCHDOG_ERR);
       } 
    }
    else {
    	error_count = 0;
    }
    #endif
    
    if (kicked_1v) {
    	kicked_1v = 0;
        counts = WATCHDOG_3V_KICK;    
    }
    else {
    	kicked_1v = 1;
        counts = WATCHDOG_1V_KICK;
    }
    
    SendWatchdogRef(counts);													
    	 
     
}


/////////////////////////////////////////////////////////////////////////
// SetEk() 
// Change Ek Ref  
//
void SetEk(unsigned set_value)
{
	static unsigned last_set_value = 0;
    
    if (set_value > EK_SET_MAX)  set_value = EK_SET_MAX;
     
    if (last_set_value != set_value) 
    { 
        last_set_value = set_value;
	    analog_sets[ANA_SET_EK].ip_set = set_value;
	    SetEkLimits();
	    SendHvRef(analog_sets[ANA_SET_EK].ip_set);
               
	}

}


/////////////////////////////////////////////////////////////////////////
// SetEg() 
// Change Pulse Top Ref  
//
void SetEg(unsigned set_value)
{
	static unsigned last_set_value = 0;

    if (set_value > EG_SET_MAX)  set_value = EG_SET_MAX;
                
    if (last_set_value != set_value) 
    { 
        last_set_value = set_value;
	    analog_sets[ANA_SET_EG].ip_set = set_value;
	    SetEgLimits();        
	    SendPulsetopRef(analog_sets[ANA_SET_EG].ip_set);
        if (set_value < EG_SET_MIN && (system_byte & SYS_BYTE_PULSETOP_ON) > 0)
        	 LogPulsetopControl(0);
	}
            
}


/////////////////////////////////////////////////////////////////////////
// SetEf() 
// Change Ef Ref  
//
void SetEf(unsigned set_value)
{
	static unsigned last_set_value = 0;
    if (set_value > EF_SET_MAX)  set_value = EF_SET_MAX;
                
    if (last_set_value != set_value) { 
        last_set_value = set_value;
	    analog_sets[ANA_SET_EF].ip_set = set_value;
	    SetEfLimits();
        if (set_value >= EF_SET_MIN) {
		    if ((control_state & 0x7f) >= STATE_WARM_UP) // send htr ref directly if htr dly started.
		    	SendHeaterRef(analog_sets[ANA_SET_EF].ip_set);
        }
        else if ((system_byte & SYS_BYTE_HTR_ON) > 0) 
        	LogicHeaterControl(0);  // turn off heater if ref < 1V
	}
            
}

/////////////////////////////////////////////////////////////////////////
// unsigned char AreAllReferencesConfigured() 
// Change Ef Ref  
//
unsigned char AreAnyReferenceNotConfigured(void)
{
	unsigned char ret = 0;
    
    if (analog_sets[ANA_SET_EK].ip_set == 0) ret = 1;
	else if (analog_sets[ANA_SET_EF].ip_set == 0) ret = 1;
    else if (analog_sets[ANA_SET_EG].ip_set == 0) ret = 1;
    
    return (ret);
}




