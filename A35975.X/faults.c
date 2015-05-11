#include "A35975.h"
#include "faults.h"
#include <libpic30.h>
//#include "Config.h"




void LoadFaultMaskRegisters(void);
void WriteToEventLog(unsigned char fault_register, unsigned int fault_bit);



/*
  FAULT EVALUATION
  
  10_ms_tic_toc
  Once every 10ms, the internal ADC is read and faults are updated

  After Every Pulse

  DPARKER give a big description of each fault here - How the input is measured, how the data is filtered, how the fault is generated (delay if included)

*/


/*
  In each State, Each fault input can do one of the following . . .

  1) It can be totally ignored (This is the default behavior - If the warning mask & the fault mask are NOT set)
  2) It can cause a fault  (This is set by the fault MASK)
     There are no "warm" or "cold" faults.  Any fault in state warm_ready or hv_on will go to state warm fault.
     If the fault is still active in the warm state fault then it will move to cold fault.
  3) It can cause a latched warning (This is set the warning MASK)
     NOTE: the warning register is independent of the fault mask.  To be latched as a warning, the fault input MUST BE IN THE WARNING REGISTER
  4) Special state at startup to handle board level failures . . . ????
*/

/*  
    Fault Log
    
    Each entry in the fault log contains the following information
    4 bits for fault register bit
    2 bits for fault register select
    2 bits for the calling state (HV_ON, WARM_READY, FAULT, STARTUP)
*/

/*
  For each state there are three fault masks for each fault register.
  Warning Ignore Mask - This shows which fault inputs will NOT generate a warning (this is mainly for debugging and and is used to filter out non-events, if a fault is in the warning ignore mask and in the warm or cold fault mask it will still generate a fault)
  Faul Mask - this shows which fault inputs will generate a fault in the current state


*/

/*

  Status_Register -> This register shows the current states of all the Inputs.  It IS NOT LATCHED.
  Fault_Register -> This register latches any fault input that matches the fault_mask for the current state.
  Warning_Latch_Regisiter -> Any fault input that matches the fault_mask or the warning_mask will be latched in this register.

  How faults are checked . . .
  
  Every time through a control loop, 50us->500us depending upon the state, all faults registeres are updated and checked.
  Faults like magnetron_heater_over_current - The limit is compared to the filtered data in RAM every time, even though the filtered data in ram is only update once every 10mS
  The fault status register is then compared to the masks to generate the warnings and faults.

 */

/* ----------------------FAULT MANAGEMENT ----------------------*/

/*
  There are lots of Fault Conditions, see A34335.h for a list of fault registeres and faults.
  For a given state a particular Input may
  1) No Action - But set a non latching status bit indicating the state of the fault input
  2) No Action - But set a latching "warning" bit 
  3) Generate a fault Condition - This will change the state to warm fault (or cold fault if the fault is globably defined as a cold fault)
 

  How are faults handeled . . .
  There a 3 data storage locations for each fault register.
  Status_Register -> This register shows the current states of all the Inputs.  It IS NOT LATCHED.
  Latch_Regisiter -> This is a latched version of the the status register.
  Fault_Register  -> If the Input is a Fault (it matches the warm_fault OR cold_fault mask) it is latched into the Fault Register
  
  How external data and faults are processesed . . . 

  Data is read from the external ADCs and inernal ADCs on a regular 10ms interval or after a pulse (for pulse data types).
  This inerval is scheduled to occur directly proceding a pulse if possible (pulse rate > 100 Hz).
  At PRF less than 100Hz (or in any other state) the readings will take place at opproximate 10ms intervals
  ADC values are filtered with software RC/glitch filters when they are read.
 
 
  Faults are evaluated durring the 10ms_tic_toc the occurs once every (approximatly) 10mS
  The following steps occur
  1) Status registers are reset to zero
  2) The input condition is tested, if it is a "logical fault" then (record_this_xxxxx_fault) is called which does the following
    a) Sets the appropriate bit in the status register
    b) If the bit matches the fault_mask, 
       ^ The appropriate bit in the fault register is set
       ^ The fault is added to the error log - TO BE IMPLEMENTED
    c) If the bit matches the warning_mask, the appropriate bit in the warning register is set 
  
  STEP 2 is repeated for all fault conditions.

  Some faults can not be tested durring the 10ms TicToc.
  These faults will have record_this_xxxxx_fault called when the fault is checked.
  These faults are . . .
  FAULT_HV_LAMBDA_EOC_TIMEOUT - this is evaluated in the TMR1 interrupt and set there.


  After all the faults have been tested and the fault registers updated the following action occurs
  
  + A cold shutdown occurs
  + A warm shutdown occurs
  
  + The fault is added to the Log
  + For *some* important faults, the fault counter is incremented.

  + The fault is added to the Log
  + For *some* important faults, the fault counter is incremented.
  
*/

/////////////////////////////////////////////////////////////////////////
// DoFaultAction() 
// take actions	when faulting
// 1: htr off, 2: hv off, 3: pulsetop off, 4: trig off, otherwise, no action
//
void DoFaultAction(unsigned char type, unsigned char disable_htr_auto_reset) {
	switch (type) {
	case 1:
    	LogicHeaterControl(0);
        if (disable_htr_auto_reset) {
        	htr_OVOC_auto_reset_disable = 1;
 	 //		htr_OVOC_count = 0;
        }
     //   if (control_state < 0x80) last_control_state = control_state;
        control_state = STATE_FAULT_COLD_FAULT;
    	break;
	case 2:
		LogHvControl(0);
    //    if (control_state < 0x80) last_control_state = control_state;
        control_state = STATE_FAULT_HOT_FAULT;
    	break;
	case 3:
		LogPulsetopControl(0);
    //    if (control_state < 0x80) last_control_state = control_state;
        control_state = STATE_FAULT_HOT_FAULT;
    	break;
	case 4:
		LogTrigControl(0);
     //   if (control_state < 0x80) last_control_state = control_state;
        control_state = STATE_FAULT_HOT_FAULT;
    	break;
	default:
    	break;
   }
   
}   
/////////////////////////////////////////////////////////////////////////
// DoFaultRecord() 
// record fault and take actions
//
void DoFaultRecord(unsigned int fault_type, unsigned int fault_bit) {
   
    unsigned idx;		    
																		     
	switch (fault_type) {
    												  
	case FAULTS_TYPE_SYSTEM_CONTROL:    
    	if ((faults_reg_system_control & fault_bit) == 0) {
    		faults_reg_system_control |= fault_bit;	
            // action
            DoFaultAction(1, 1);
            
           if (fault_bit & (FAULTS_SYS_FPGAID |	FAULTS_SYS_FPGA_WATCHDOG_ERR)) 
           		_FAULT_GD_FPGA_COMM_LOST = 1;
        }			   	
        break;												   
	
    case FAULTS_TYPE_SOFTWARE:
    	if ((faults_reg_software & fault_bit) == 0) {
    		faults_reg_software |= fault_bit;	
            // action
        }			   	
        break;
	
    case FAULTS_TYPE_DIGI_FROM_FPGAID:
#ifndef TEST_BYP_FPGA_FAULTS
    	if ((faults_reg_digi_from_gd_fpgaid & fault_bit) == 0) {
    		faults_reg_digi_from_gd_fpgaid |= fault_bit;
            // action
             if (fault_bit == 0x0002 || fault_bit == 0x0004) {  // wdog and arc faults from ADC
             	 if (fault_bit > 2)  _FAULT_GD_FPGA_ARC_FAULT = 1;
                 else				 _FAULT_GD_FPGA_COMM_LOST = 1;
                 
                 idx = (fault_bit >> 1);
                 DoFaultAction(digi_reads[idx].action_code, 1);
            }
            else { 
            	for (idx = 0; idx < 15; idx++) {
                	if (fault_bit == (1 << idx)) break;
                }
                if (idx < 15) {	// found the index
                    idx += DIGI_ID_ARC_COUNT;
                    switch (idx) {
                    case DIGI_ID_TEMP_75C:
                    	_FAULT_GD_FPGA_TEMP_75C = 1;
                    	break;
                    case DIGI_ID_PRF:
                    case DIGI_ID_CURR_PW:
                    	_FAULT_GD_FPGA_PULSE_FAULT = 1;
                    	break;
 
                    case DIGI_ID_GRID_HW:
                    case DIGI_ID_GRID_OV:
                    case DIGI_ID_GRID_UV:
                    case DIGI_ID_BIAS_V:
                    	_FAULT_GD_FPGA_GRID_FAULT = 1;
                    	break;

                    default:
                    	break;

                    }

                    DoFaultAction(digi_reads[idx].action_code, 1);
                }
            }
        }
#endif        			   	
        break;
	
    default:
    break;
    }        
		
    if (faults_reg_system_control || faults_reg_software || (faults_reg_digi_from_gd_fpgaid & FPGAID_FAULTS_MASK))	{
    	PIN_LED_SUM_FAULT = OLL_LED_ON; 
        system_byte |= SYS_BYTE_FAULT_ACTIVE; 
        _FAULT_GD_SUM_FAULT = 1;       
    }     
    else {
     	PIN_LED_SUM_FAULT = !OLL_LED_ON;  
        system_byte &= ~SYS_BYTE_FAULT_ACTIVE;
        _FAULT_GD_SUM_FAULT = 0;        
    }
        
}		
/////////////////////////////////////////////////////////////////////////
// DoFaultClear() 
// clear fault and let logic continue
//
void DoFaultClear(unsigned int fault_type, unsigned int fault_bit) {		    
																		     
	switch (fault_type) {
    												  
	case FAULTS_TYPE_SYSTEM_CONTROL:
   		faults_reg_system_control &= ~fault_bit;	
        break;												   
	
    case FAULTS_TYPE_SOFTWARE:
   		faults_reg_software &= ~fault_bit;	
        break;
	
    case FAULTS_TYPE_DIGI_FROM_FPGAID:
   		faults_reg_digi_from_gd_fpgaid &= ~fault_bit;
        break;
	
    default:
    break;
    } 
    
    if (faults_reg_system_control || faults_reg_software || (faults_reg_digi_from_gd_fpgaid & FPGAID_FAULTS_MASK))	{
    	PIN_LED_SUM_FAULT = OLL_LED_ON;
        system_byte |= SYS_BYTE_FAULT_ACTIVE;        
        _FAULT_GD_SUM_FAULT = 1;       
    }      
    else {
     	PIN_LED_SUM_FAULT = !OLL_LED_ON;      
        system_byte &= ~SYS_BYTE_FAULT_ACTIVE;        
        _FAULT_GD_SUM_FAULT = 0;       
	}
		
}		


/////////////////////////////////////////////////////////////////////////
// CheckAnalogLimits() check analog over/under limits
// 
//
void CheckAnalogLimits(unsigned index) {

    /* test for foo. */
    if (index < ANALOG_READ_SIZE) {
		    /* test if over the max */
		    if (analog_reads[index].read_m_hi &&
 		        (analog_reads[index].read_cur > analog_reads[index].read_f_hi))	{

		        if (analog_reads[index].read_m_lo > 1)  /*  if low was in service */
		            analog_reads[index].read_m_lo = 1;   /* reset it               */

		        /* faulted once already - trip fault this time */
		        if (analog_reads[index].read_m_hi == 3){

		            analog_reads[index].read_m_hi = 2;       /* set in service mode */

		            if (analog_reads[index].fault_vect)      /* test for vector */
		                (analog_reads[index].fault_vect)(2); /* call if so send hi signal*/

		            if (analog_reads[index].read_m_lo == 2)  /*  if low was in service */
		                analog_reads[index].read_m_lo = 1;   /* reset it               */

		        }

		        /* if not masked and not in service */
		        if (analog_reads[index].read_m_hi == 1){

		            analog_reads[index].read_m_hi = 3;       /* set one free trip */

		            return;

		        }

		    }

		    /* test if under the min */
		    else if (analog_reads[index].read_m_lo &&
		        (analog_reads[index].read_cur < analog_reads[index].read_f_lo)) {

		        if (analog_reads[index].read_m_hi > 1)  /*  if hi was in service */
		            analog_reads[index].read_m_hi = 1;   /* reset it               */

		        /* if TRIP + HAD ONE FREE ALREADY */
		        if (analog_reads[index].read_m_lo == 3){

		            analog_reads[index].read_m_lo = 2;       /* set in service mode */

		            if (analog_reads[index].fault_vect)      /* test for vector */
		                (analog_reads[index].fault_vect)(1); /* call if so send lo signal*/

		            if (analog_reads[index].read_m_hi == 2)  /*  if hi was in service */
		                analog_reads[index].read_m_hi = 1;   /* reset it               */

		        }

		        /* if not masked and not in service */
		        if (analog_reads[index].read_m_lo == 1){

		            analog_reads[index].read_m_lo = 3;       /* set ONE FREE TRIP mode */

		            return;
		        }

		    }
		    
		    /* test for clear                   */
		    /* could be 2 TRIP or 3 - one time  */
		    /* one free fault added             */
		    else if ((analog_reads[index].read_m_hi > 1) || (analog_reads[index].read_m_lo > 1)) {

		        /* log event - reset happened after only one reading */
		        if ((analog_reads[index].read_m_hi == 3) || (analog_reads[index].read_m_lo == 3)) 
		            analog_reads[index].events++;

		        if (analog_reads[index].fault_vect)      /* test for vector */
		            (analog_reads[index].fault_vect)(0); /* call if so send lo signal*/

		        if (analog_reads[index].read_m_hi)
		            analog_reads[index].read_m_hi = 1;     /* reset in-services - cleared or masked */

		        if (analog_reads[index].read_m_lo)
		            analog_reads[index].read_m_lo = 1;         

		    }

	}  // index < ANALOG_READ_SIZE


}

/////////////////////////////////////////////////////////////////////////
// FaultEk() 
//  
//
void FaultEk(unsigned state) {

	unsigned char fault_just_happened = 0;
    
	switch (state) {
    
	case 1: // under
      if (ekuv_timeout_10ms < 220) {
      	  if (!ekuv_timeout_10ms) ekuv_timeout_10ms++;
          if (analog_reads[ANA_RD_EK].read_m_lo > 1) analog_reads[ANA_RD_EK].read_m_lo = 1;
          break; // no faulting this time
      }
      if (ek_ref_changed_timer_10ms) {
          if (analog_reads[ANA_RD_EK].read_m_lo == 2) analog_reads[ANA_RD_EK].read_m_lo = 1;
          if (analog_reads[ANA_RD_EK].read_m_hi == 2) analog_reads[ANA_RD_EK].read_m_hi = 1;
          break; // no faulting this time, armed for next time     
      }
      else {
      	  _FAULT_GD_SW_EK_UV = 1;
          fault_just_happened = 1;
      }
      break;
      	  
	case 2: // over, and under pass through
      if (ek_ref_changed_timer_10ms) {
          if (analog_reads[ANA_RD_EK].read_m_lo == 2) analog_reads[ANA_RD_EK].read_m_lo = 1;
          if (analog_reads[ANA_RD_EK].read_m_hi == 2) analog_reads[ANA_RD_EK].read_m_hi = 1;
          break; // no faulting this time, armed for next time     
      }
      else {
      	  _FAULT_GD_SW_EK_OV = 1; 
          fault_just_happened = 1;
      }    
      break;
      
	default:
      ekuv_timeout_10ms = 0;
      break;
      
    }
    if (fault_just_happened) {
      DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EKOV_EKUV);
      DoFaultAction(2, 0);
    }

}
/////////////////////////////////////////////////////////////////////////
// FaultEf() 
//  
//
void FaultEf(unsigned state) {

	switch (state) {
    
	case 1: // under
#ifndef TEST_BYP_FPGA_FAULTS
      if (ef_ref_changed_timer_10ms) {
          if (analog_reads[ANA_RD_EF].read_m_lo == 2) analog_reads[ANA_RD_EF].read_m_lo = 1;
          break; // no faulting this time, armed for next time     
      }
      _FAULT_GD_SW_HTR_UV = 1; // QQ: not fault, warning only?
      DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EFUV);
      DoFaultAction(2, 0);
#endif
      break;
	case 2: // over
#ifndef TEST_BYP_FPGA_FAULTS
      if (ef_ref_changed_timer_10ms) {
          if (analog_reads[ANA_RD_EF].read_m_hi == 2) analog_reads[ANA_RD_EF].read_m_hi = 1;
          break; // no faulting this time, armed for next time     
      }
      _FAULT_GD_SW_HTR_OVOC = 1;
      htr_OVOC_count++;
      DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EFOV_IFOC);
      DoFaultAction(1, htr_OVOC_count >= 5? 1 : 0);
      
#endif
      break;
	default:
      break;
      
    }
    
}
/////////////////////////////////////////////////////////////////////////
// FaultIf() 
//  
//
void FaultIf(unsigned state) {

	switch (state) {
    
	case 1: // under
      break;
	case 2: // over
#ifndef TEST_BYP_FPGA_FAULTS
      if (ef_ref_changed_timer_10ms) {
          if (analog_reads[ANA_RD_IF].read_m_hi == 2) analog_reads[ANA_RD_IF].read_m_hi = 1;
          break; // no faulting this time, armed for next time     
      }
      _FAULT_GD_SW_HTR_OVOC = 1;
      htr_OVOC_count++;
      DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EFOV_IFOC);
      DoFaultAction(1, htr_OVOC_count >= 5? 1 : 0);
#endif
      break;
	default:
      break;
      
    }
    
}
/////////////////////////////////////////////////////////////////////////
// FaultEg() 
//  
//
void FaultEg(unsigned state) {

	switch (state) {
    
	case 1: // under
      break;
	case 2: // over
      if (eg_ref_changed_timer_10ms) {
          if (analog_reads[ANA_RD_EG].read_m_hi == 2) analog_reads[ANA_RD_EG].read_m_hi = 1;
          break; // no faulting this time, armed for next time     
      }
      _FAULT_GD_SW_GRID_OV = 1;
      DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EGOV);
      DoFaultAction(1, 1);
      break;
	default:
      break;
      
    }
    
}
/////////////////////////////////////////////////////////////////////////
// FaultEc() 
//  
//
void FaultEc(unsigned state) {

	switch (state) {
    
	case 1: // under
	  _FAULT_GD_SW_BIAS_UV = 1;
      DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_ECUV);
      DoFaultAction(1, 1);
      break;
	case 2: // over
      break;
	default:
      break;
      
    }
    
}
/////////////////////////////////////////////////////////////////////////
// Fault24v() 
//  
//
void Fault24v(unsigned state) {

	switch (state) {
    
	case 1: // under
	case 2: // over
      _FAULT_GD_SW_24V_FAULT = 1;
      DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_24V);
      DoFaultAction(2, 0);
      break;
	default:
      break;
      
    }
    
}

/////////////////////////////////////////////////////////////////////////
// SetEfLimits() 
//  
//
void SetEfLimits(void) {

	double value, temp;
    
    value = (double)analog_sets[ANA_SET_EF].ip_set * 0.000133;
    temp = value * 0.2;
    if (temp < 0.2) temp = 0.2;
    temp += value;
    temp /= 0.00222;
    if (temp > EF_READ_MAX) temp = EF_READ_MAX;
    analog_reads[ANA_RD_EF].read_f_hi = (unsigned)temp;
    

    temp = value * 0.15;
    if (temp < 0.2) temp = 0.2;
    value -= temp;
    if (value < 0) value = 0;
    value /= 0.00222;
    analog_reads[ANA_RD_EF].read_f_lo = (unsigned)value;
    
	ef_ref_changed_timer_10ms = 220;  // 2.2s

}

/////////////////////////////////////////////////////////////////////////
// SetEkLimits() 
//  
//
void SetEkLimits(void) {

	double value, offset, temp;
    
    value = (double)analog_sets[ANA_SET_EK].ip_set * 0.0003333;
    offset = value * 0.1;
    if (offset < 0.2) offset = 0.2; // min 200v
 
    temp = value + offset;
    temp /= 0.005555;
    analog_reads[ANA_RD_EK].read_f_hi = (unsigned)temp;
    
    temp = value - offset;
    if (temp < 0) temp = 0;
    temp /= 0.005555;
    analog_reads[ANA_RD_EK].read_f_lo = (unsigned)temp;
    
	ek_ref_changed_timer_10ms = 220;  // 2.2s
    
}
/////////////////////////////////////////////////////////////////////////
// SetEgLimits() 
//  
//
void SetEgLimits(void) {

	double value;
    
    value = (double)analog_sets[ANA_SET_EG].ip_set * 0.00666;
    value += 10;
    value /= 0.1111;
    analog_reads[ANA_RD_EG].read_f_hi = (unsigned)value;

	eg_ref_changed_timer_10ms = 220;  // 2.2s    

}

// DPARKER update pulse fault must be called at 10ms Interval or the sections that count "out of range" counts will be arbitrary time lengths

void UpdateFaults(void) {
  #if 0
  // See h file for documentation
  unsigned int temp_u16int;

  // The status registers are not latched so they are reset to zero each time the faults are evaluated
  faults_magnetron_status_reg = 0;  
  faults_high_voltage_status_reg = 0;
  faults_thyratron_status_reg = 0;
  faults_control_board_status_reg = 0;
  
  
  // Load the fault masks for the current state
  LoadFaultMaskRegisters();
  
  //------------------------- START MAGNETRON FAULTS ------------------------------//
  
  // Check External Magnetron Heater Over Voltage Latch
  if (PIN_FILAMENT_OV_LATCH == ILL_FILAMENT_OV_FAULT) {
    RecordThisMagnetronFault(FAULT_HW_MAGNETRON_FILAMENT_OV);
  } 
  
  // Check that the magnetron heater voltage ADC reading has exceed fixed value set Config.h 
  if (ps_filament.v_adc_reading > ps_filament.v_adc_over_abs) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_OV_HARD_LIMIT);
  }
  
  // Check that the magnetron heater voltage ADC reading is not greater than X% of its program point (set in Config.h)
  // It must remain outside this range for at least v_out_of_range_count before a fault is generated
  if (CheckOverVoltFault(&ps_filament)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_OV);
  }
  
  // Check that the magnetron heater voltage ADC reading is not less than X% of its program point (set in Config.h)
  // It must remain outside this range for at least v_out_of_range_count before a fault is generated
  if (CheckUnderVoltFault(&ps_filament)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_UV);
  }
  
  // Check that the magnetron heater current ADC reading is not greater than X% of its expected point (set in Config.h)
  // It must remain outside this range for at least v_out_of_range_count before a fault is generated
  if (CheckOverCurrentFault(&ps_filament)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_OC);
  }
  
  
  // Check that the magnetron heater current ADC reading is not less than X% of its expected point (set in Config.h)
  // It must remain outside this range for at least v_out_of_range_count before a fault is generated
  if (CheckUnderCurrentFault(&ps_filament)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_UC);
  }
  
  
  // Check External Magnetron Magnet Current Out of Range Latch
  if (PIN_MAGNET_CURRENT_OOR_LATCH == ILL_MAGNET_CURRENT_OOR_FAULT) {
    RecordThisMagnetronFault(FAULT_HW_MAGNETRON_MAGNET_COOR);
  }
  
  // Check that the magnetron magnet current ADC reading has no exceeded fixed value set Config.h 
  if (ps_magnet.i_adc_reading > ps_magnet.i_adc_over_abs) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_OC_HARD_LIMIT);
  }
  
  // Check that the magnetron magnet current ADC reading is not greater than X% of its program point (set in Config.h)
  if (CheckOverCurrentFault(&ps_magnet)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_OC);
  }
  
  // Check that the magnetron magnet current ADC reading is not less than X% of its program point (set in Config.h)
  if (CheckUnderCurrentFault(&ps_magnet)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_UC);
  }
  
  // Check that the magnetron magnet voltage ADC reading is not greater than X% of its expected value (set in Config.h)
  if (CheckOverVoltFault(&ps_magnet)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_OV);
  }
  
  // Check that the magnetron magnet voltage ADC reading is not less than X% of its expected value (set in Config.h)
  if (CheckUnderVoltFault(&ps_magnet)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_UV);
  }
  
  
  //------------------------- START HIGH VOLTAGE FAULTS ------------------------------//
  
  // THE ARC FAULTS ARE CURRENTLY SET IN UpdatePulseData()
  // UpdatesPulseData() is called after every pulse.
  
  // Check the digital fault outputs from from HV LAMBDA -
  // The sum fault line must be set for HV_LAMBDA_SUM_FAULT_COUNTER_TRIP_POINT consective readings before a trip will occur
  
  
  
  if (PIN_HV_LAMBDA_SUM_FAULT == ILL_HV_LAMBDA_SUM_FAULT_FAULTED) {
    // Record the sum fault and check the other fault lines
    RecordThisHighVoltageFault(FAULT_HV_LAMBDA_SUM_FAULT);
    
    temp_u16int = MCP23017ReadSingleByte(&U64_MCP23017, MCP23017_REGISTER_GPIOA);
    if (temp_u16int >= 0xFA00) {
      global_debug_counter.i2c_bus_error++;
    } else {
      U64_MCP23017.input_port_a_in_ram = temp_u16int & 0xFF;
      if (U64_MCP23017.input_port_a_in_ram & BIT_INPUT_HV_LAMBDA_OVER_TEMP) {
	RecordThisHighVoltageFault(FAULT_HV_LAMBDA_OVER_TEMP);
      }
      if (U64_MCP23017.input_port_a_in_ram & BIT_INPUT_HV_LAMBDA_INTERLOCK_FAULT) {
	RecordThisHighVoltageFault(FAULT_HV_LAMBDA_INTERLOCK_FAULT);
      }
      if (U64_MCP23017.input_port_a_in_ram & BIT_INPUT_HV_LAMBDA_LOAD_FLT) {
	RecordThisHighVoltageFault(FAULT_HV_LAMBDA_LOAD_FAULT);
      }
      if (U64_MCP23017.input_port_a_in_ram & BIT_INPUT_HV_LAMBDA_PHASE_LOSS) {
	RecordThisHighVoltageFault(FAULT_HV_LAMBDA_PHASE_LOSS);
      }
    }
  }
  
  /*
    FAULT_LAMBDA_EOC_TIMEOUT is checked/set by the TMR1 Interrupt
  */


  // DPARKER these vpeak readings . . . do we need them?  do they serve any purpose????
  /*

  // Check that the lambda vpeak ADC reading is not greater than X% of its set point (set in Config.h)
  // It must remain outside this range for at least HV_LAMBDA_VPEAK_MAX_OUT_OF_RANGE_COUNT before a fault is generated
  if (hv_lambda_vpeak_adc_reading > hv_lambda_vpeak_adc_over_trip_point) {
    hv_lambda_vpeak_over_voltage_count++;
  } else if (hv_lambda_vpeak_over_voltage_count >= 1) {
    hv_lambda_vpeak_over_voltage_count--;
  }
  if (hv_lambda_vpeak_over_voltage_count > HV_LAMBDA_VPEAK_MAX_OUT_OF_RANGE_COUNT) {
    RecordThisHighVoltageFault(FAULT_HV_LAMBDA_VPEAK_OVER_VOLTAGE);
  }

  // Check that the lambda vpeak ADC reading is not less than X% of its set point (set in Config.h)
  // It must remain outside this range for at least HV_LAMBDA_VPEAK_MAX_OUT_OF_RANGE_COUNT before a fault is generated
  if (hv_lambda_vpeak_adc_reading < hv_lambda_vpeak_adc_under_trip_point) {
    hv_lambda_vpeak_under_voltage_count++;
  } else if (hv_lambda_vpeak_under_voltage_count >= 1) {
    hv_lambda_vpeak_under_voltage_count--;
  }
  if (hv_lambda_vpeak_under_voltage_count > HV_LAMBDA_VPEAK_MAX_OUT_OF_RANGE_COUNT) {
    RecordThisHighVoltageFault(FAULT_HV_LAMBDA_VPEAK_UNDER_VOLTAGE);
  }
  */
  
  //------------------------- START THYRATRON FAULTS ------------------------------//
 
  // Check that the thyratron cathode heater voltage ADC reading has exceed fixed value set Config.h 
  if (ps_thyr_cathode_htr.v_adc_reading > ps_thyr_cathode_htr.v_adc_over_abs) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_OV_HARD_LIMIT);
  }
    
  // Check that the thyratron heater voltage ADC reading is not greater than X% of its program point (set in Config.h)
  // It must remain outside this range for at least THYRATRON_HEATER_MAX_OUT_OF_RANGE_COUNT before a fault is generated  
  if (CheckOverVoltFault(&ps_thyr_cathode_htr)) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_OV);
  }

  // Check that the thyratron heater voltage ADC reading is not less than X% of its program point (set in Config.h)
  // It must remain outside this range for at least THYRATRON_HEATER_MAX_OUT_OF_RANGE_COUNT before a fault is generated
  if (CheckUnderVoltFault(&ps_thyr_cathode_htr)) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_UV);
  }
    
  // Check if the 4-20ma Driver has reported a fault
  if (PIN_4_20_DRVR_FLT == ILL_4_20_DRIVER_FAULT) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_DRVR_FLT);
  }

  // Check to see if the control loop has saturated.
  // If it has that means that the SCR controller is no longer responding and it should be shut down
  temp_u16int = thyratron_cathode_heater_PID.controlOutput;
  if (temp_u16int & 0x8000) {
    temp_u16int = 0x0000;
  }
  temp_u16int = temp_u16int << 1;
  if (temp_u16int >= THYRATRON_DAC_SATURATED) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_CONTROL_SAT);
  }
  


  // Check if the thyratron reservoir ADC reading has exceed fixed value set Config.h 
  if (ps_thyr_reservoir_htr.v_adc_reading > ps_thyr_reservoir_htr.v_adc_over_abs) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_OV_HARD_LIMIT);
  }


  // Check that the thyratron reservoir voltage ADC reading is not greater than X% of its program point (set in Config.h)
  // It must remain outside this range for at least THYRATRON_HEATER_MAX_OUT_OF_RANGE_COUNT before a fault is generated  
  if (CheckOverVoltFault(&ps_thyr_reservoir_htr)) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_OV);
  }

  // Check that the thyratron reservoir voltage ADC reading is not less than X% of its program point (set in Config.h)
  // It must remain outside this range for at least THYRATRON_HEATER_MAX_OUT_OF_RANGE_COUNT before a fault is generated
  if (CheckUnderVoltFault(&ps_thyr_reservoir_htr)) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_UV);
  }

  // Check if the 4-20ma Driver has reported a fault
  if (PIN_4_20_DRVR_FLT == ILL_4_20_DRIVER_FAULT) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_DRVR_FLT);
  }
  
  // Check to see if the control loop has saturated.
  // If it has that means that the SCR controller is no longer responding and it should be shut down
  temp_u16int = thyratron_reservoir_heater_PID.controlOutput;
  if (temp_u16int & 0x8000) {
    temp_u16int = 0x0000;
  }
  temp_u16int = temp_u16int << 1;
  if (temp_u16int >= THYRATRON_DAC_SATURATED) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_CONTROL_SAT);
  }


  //------------------------- START CONTROL BOARD FAULTS ------------------------------//


  // Check that the lambda supply powered up
  if (PIN_HV_LAMBDA_POWER_UP == ILL_PIN_HV_LAMBDA_DID_NOT_POWER_UP) {
    RecordThisControlBoardFault(FAULT_LAMBDA_OFF);
  }
  
  // Check to see if digital interlock 1 is open  
  if (PIN_INTERLOCK_1 == ILL_INTERLOCK_OPEN) {
    RecordThisControlBoardFault(FAULT_DIGITAL_INTERLOCK_1);
  }
  
  // Check to see if digital interlock 2 is open  
  if (PIN_INTERLOCK_2 == ILL_INTERLOCK_OPEN) {
    RecordThisControlBoardFault(FAULT_DIGITAL_INTERLOCK_2);
  }
  
  // Check to see if digital interlock 3 is open  
  if (PIN_INTERLOCK_3 == ILL_INTERLOCK_OPEN) {
    RecordThisControlBoardFault(FAULT_DIGITAL_INTERLOCK_3);
  }
  
  // Check to see if digital interlock 4 is open  
  if (PIN_INTERLOCK_4 == ILL_INTERLOCK_OPEN) {
    RecordThisControlBoardFault(FAULT_DIGITAL_INTERLOCK_4);
  }
  #endif
}

void ResetPulseLatches(void) {
  #if 0
  PIN_PULSE_LATCH_RESET = OLL_PULSE_LATCH_RESET;
  __delay32(DELAY_PULSE_LATCH_RESET);
  PIN_PULSE_LATCH_RESET = !OLL_PULSE_LATCH_RESET;
  #endif
}


void ResetHWLatches(void) {

 #if 0
  // Reset the latches
  PIN_LATCH_RESET = OLL_RESET_LATCH;
  __delay32(DELAY_LATCH_RESET);
  PIN_LATCH_RESET = !OLL_RESET_LATCH;
 #endif
}

void ResetAllFaults(void) {

  ResetFPGA();
  
  faults_reg_system_control = 0;
  faults_reg_software = 0;	 
  faults_reg_digi_from_gd_fpgaid = 0;
  
  control_state = STATE_START_UP;
 
 
}







void LoadFaultMaskRegisters(void) {
}



void RecordThisMagnetronFault(unsigned int fault_bit) {  
}





void WriteToEventLog(unsigned char fault_register, unsigned int fault_bit) {
  // DPARKER this function should write to the event log in ram and update the fault counter in RAM
  // These values are moved to EEPROM later on  . . . like programmed later on
}



unsigned int CheckFaultActive(void) {
 // return (faults_control_board_fault_reg | faults_thyratron_fault_reg | faults_magnetron_fault_reg | faults_high_voltage_fault_reg);
    return (0);
}


unsigned int CheckColdFaultActive(void) {
  unsigned int temp = 0;
  return temp;
}




