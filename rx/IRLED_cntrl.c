#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "IRLED_cntrl.h"

uint32_t WCn_valo() //aka function 1-7
{
	nrf_gpio_cfg_output(IRLED_PIN);
	nrf_gpio_pin_clear(IRLED_PIN);
	nrf_delay_us(300); //Make sure the output has been off long(t=?) enough before starting
	for (int i=0;i<766;i++)
	{
		nrf_gpio_pin_set(IRLED_PIN);
		nrf_delay_us(65);
		nrf_gpio_pin_clear(IRLED_PIN);
		nrf_delay_us(129); 
		nrf_delay_us(1);//Replace with NOPs if necessary to compensate for processor time used for the loop
	}
	nrf_gpio_cfg_input(IRLED_PIN,NRF_GPIO_PIN_NOPULL);
	return NRF_SUCCESS;
}


//This is a horrible mess, but shouldn't be cleaned because it might mess with the timings.
uint32_t GEWA_IR_cntrl(uint16_t IR_pattern) 
{
	nrf_gpio_cfg_output(IRLED_PIN);
	nrf_gpio_pin_clear(IRLED_PIN);
	nrf_delay_ms(30); //Make sure the output has been off long(t=?) enough before starting
	
	//Send preamble
	GEWA_pulsate(1);
	nrf_delay_us(1865); //2220us is a direct measurement result.
	
	//Send command
	for (int i = 15; i >= 0; i--) //Send the 14 "bits" - rest are fillers ... or are they?
	{
		GEWA_pulsate((IR_pattern >> i) & 0x0001);
	}
	//uint32_t total_delay_us = 41107;//49500 - (17* GEWA_HALF_PERIOD*32 + 1865)
	nrf_delay_ms(41);
	nrf_delay_us(107);
	GEWA_pulsate(1);
	nrf_delay_us(1865);
	for (int i = 15; i >= 0; i--) //Send the 14 "bits" - rest are fillers ... or are they?
	{
		GEWA_pulsate((IR_pattern >> i) & 0x0001);
	}
	nrf_delay_ms(41);
	nrf_delay_us(107);
	GEWA_pulsate(1);
	nrf_delay_us(1865);
	for (int i = 15; i >= 0; i--) //Send the 14 "bits" - rest are fillers ... or are they?
	{
		GEWA_pulsate((IR_pattern >> i) & 0x0001);
	}
	nrf_delay_ms(91);
	nrf_delay_us(107);
	GEWA_pulsate(1);
	nrf_delay_us(1865);
	for (int i = 15; i >= 0; i--) //Send the 14 "bits" - rest are fillers ... or are they?
	{
		GEWA_pulsate((IR_pattern >> i) & 0x0001);
	}
	nrf_delay_ms(91);
	nrf_delay_us(107);
	
	GEWA_pulsate(1);
	nrf_delay_us(1865);
	for (int i = 15; i >= 0; i--) //Send the 14 "bits" - rest are fillers ... or are they?
	{
		GEWA_pulsate((GEWA_ENDINGS >> i) & 0x0001);
	}
	//uint32_t total_delay_us = 41107;//49500 - (17* GEWA_HALF_PERIOD*32 + 1865)
	nrf_delay_ms(41);
	nrf_delay_us(107);
	GEWA_pulsate(1);
	nrf_delay_us(1865);
	for (int i = 15; i >= 0; i--) //Send the 14 "bits" - rest are fillers ... or are they?
	{
		GEWA_pulsate((GEWA_ENDINGS >> i) & 0x0001);
	}
	nrf_delay_ms(41);
	nrf_delay_us(107);
	GEWA_pulsate(1);
	nrf_delay_us(1865);
	for (int i = 15; i >= 0; i--) //Send the 14 "bits" - rest are fillers ... or are they?
	{
		GEWA_pulsate((GEWA_ENDINGS >> i) & 0x0001);
	}
	
	
	
	
	nrf_gpio_cfg_input(IRLED_PIN,NRF_GPIO_PIN_NOPULL);
	return NRF_SUCCESS;
}

//Necessary to fine tune delays with NOPs?
uint32_t GEWA_pulsate(uint8_t active)
{
	//Assumed that GPIO_cfg done and output == clear
	if (active)
	{
		for (int i=0; i<16;i++) //Replace loop with 4*16 lines of code?
		{
			nrf_gpio_pin_set(IRLED_PIN);
			nrf_delay_us(GEWA_HALF_PERIOD);
			nrf_gpio_pin_clear(IRLED_PIN);
			nrf_delay_us(GEWA_HALF_PERIOD);
		}
	}
	else
		nrf_delay_us(16*2*GEWA_HALF_PERIOD);
	return NRF_SUCCESS;
}

uint32_t Samsung_control(uint32_t cmd)
{
	nrf_gpio_cfg_output(IRLED_PIN);
	nrf_gpio_pin_clear(IRLED_PIN);
	nrf_delay_ms(1); //Make sure the output has been off long(t=?) enough before starting
	
	/*Preamble 
	*/
	
	/* //Inverted NEC style
	Samsung_pulsate(171);
	nrf_delay_ms(4);
	nrf_delay_us(480); */
	
	//NEC style
	//Samsung_pulsate(342); //342*26.3 = 8995us. resulted in 8921us
	/*Samsung_pulsate(344);
	nrf_delay_ms(4);
	nrf_delay_us(480); //Measured total 4509us*/
	
	//Measured style ??? THIS WORKS!
	Samsung_pulsate(171);
	nrf_delay_ms(4);
	nrf_delay_us(480);
	 
	 int n_pulses_sb = 22; //NEC target is 560us. Measurement (remote) was 560us. This is 558us
	 int sp_us = 550; //NEC target is 560us. Measurement was ~568.5us. This is 576us
	 int lp_us = 1675; //NEC target is 1690us. Measurement was ~1683us

	//32bits of data twice
	for (int i = 31; i >= 0; i--)
	{
		if((cmd >> i) & 0x00000001)
		{
			Samsung_pulsate(n_pulses_sb); //NEC target is 560us. Measurement (remote) was 560us
			nrf_delay_us(lp_us); 
		}
		else
		{
			Samsung_pulsate(n_pulses_sb);
			nrf_delay_us(sp_us); 
		}
	}
	Samsung_pulsate(n_pulses_sb); //Message end burst. NEC target is 560us.
	
	//Pause for ~46.8 ms (measured)
	nrf_delay_ms(46);
	nrf_delay_us(660);
	
	//NO PREAMBLE HERE
	
	//Repeat previous
	for (int i = 31; i >= 0; i--)
	{
		if((cmd >> i) & 0x00000001)
		{
			Samsung_pulsate(n_pulses_sb);
			nrf_delay_us(lp_us);
		}
		else
		{
			Samsung_pulsate(n_pulses_sb);
			nrf_delay_us(sp_us); 
		}
	}
	Samsung_pulsate(n_pulses_sb); //Message end burst. NEC target is 560us.
	
	//Additional (Button pressed down) commands after 205ms (measured)
	
	return NRF_SUCCESS;
}

uint32_t Samsung_pulsate(uint32_t n_cycles)
{
	for (int i=0; i<n_cycles;i++)
	{
		nrf_gpio_pin_set(IRLED_PIN);
		nrf_delay_us(SAMSUNG_ON_PERIOD);
		nrf_gpio_pin_clear(IRLED_PIN);
		nrf_delay_us(SAMSUNG_OFF_PERIOD);
	}
	return NRF_SUCCESS;
}

/* (Almost) Generic JVC IR remote emulation
*/
uint32_t JVC_control(uint16_t cmd)
{
	nrf_gpio_cfg_output(IRLED_PIN);
	nrf_gpio_pin_clear(IRLED_PIN);
	nrf_delay_ms(1); //Make sure the output has been off long(t=?) enough before starting
	
	//Preamble
	JVC_pulsate(320); //323 would be closer to the desired 8.4ms. Measurement was 8.25 ms
	nrf_delay_ms(4);
	nrf_delay_us(200); //Target is 4200us. Measurement was ~4.35ms
	
	
	for (int j = (JVC_REPEAT?0:4); j< 5; j++) //Do once or repeat 5 times
	{
		//8+8bits of data
		for (int i = 15; i >= 0; i--)
		{
			if((cmd >> i) & 0x0001)
			{
				JVC_pulsate(20); //Target is 526us. Measurement was 417us with 16 cycles. Tweak accordingly
				nrf_delay_us(1578); //Target is 1578us. Measurement was ~1750us. Tweak accordingly
			}
			else
			{
				JVC_pulsate(20);
				nrf_delay_us(526);
			}
		}
		JVC_pulsate(20); //Message end burst. Target is 526us. Measurement was 417us with 16 cycles. Tweak accordingly.
		
		if(JVC_REPEAT)
			nrf_delay_ms((j?29:20)); //Target is 50-60ms total msg length. Measured pause was 20.585ms (1st)& message time 30(repeats)-38ms(first). Tweak accordingly.
	}
	
	//nrf_gpio_cfg_input(IRLED_PIN,NRF_GPIO_PIN_NOPULL);
	return NRF_SUCCESS;
}

uint32_t JVC_pulsate(uint32_t n_cycles)
{
	for (int i=0; i<n_cycles;i++)
	{
		nrf_gpio_pin_set(IRLED_PIN);
		nrf_delay_us(JVC_ON_PERIOD);
		nrf_gpio_pin_clear(IRLED_PIN);
		nrf_delay_us(JVC_OFF_PERIOD);
	}
	return NRF_SUCCESS;
}

uint32_t Finlux_control(uint32_t cmd)
{
	nrf_gpio_cfg_output(IRLED_PIN);
	nrf_gpio_pin_clear(IRLED_PIN);
	nrf_delay_ms(1); //Make sure the output has been off long(t=?) enough before starting
	
	/* Now this will get ugly... 
	cmd format is '1' == long burst / pause, '0' == short burst / pause. 
	Bursts and pauses alternate of course starting with a burst.
	Measured times (avg): SB = 800 us, SP = 1000 us, LB=1678us, LP = 1899us
	Command is 20 bits + stop (short burst) as far as i can tell <-- NOPE! 
	Command is 20 / 22 "bits" + stop (short burst). Message length is stored in the 5 LSBs
	*/
	
	/*
	SP = 912
	SB = 879
	LB = 1772,5 
	LP = 1808
	*/
	int stop_pos = 32-(cmd & 0x0000001f); //Get message length
	
	//int n_pulses_sb =
	//int n_pulses_lb
	
	//for (int i = 31; i<=11; i--)
	for (int i = 31; i>=stop_pos; i--)
	{
		if (i%2) //burst
		{
			if((cmd >> i) & 0x00000001) //long
				Finlux_pulsate(66); //1771us
			else					//short
				Finlux_pulsate(33); //877us
		}
		else //pause
		{
			if((cmd >> i) & 0x00000001) //long
				nrf_delay_us(1780);
			else						//short
				nrf_delay_us(886); //890 = 917
		}
	}
	
	return NRF_SUCCESS;
}

uint32_t Finlux_pulsate(uint32_t n_cycles)
{
	for (int i=0; i<n_cycles;i++)
	{
		nrf_gpio_pin_set(IRLED_PIN);
		nrf_delay_us(FINLUX_ON_PERIOD);
		nrf_gpio_pin_clear(IRLED_PIN);
		nrf_delay_us(FINLUX_OFF_PERIOD);
	}
	return NRF_SUCCESS;
}

//Might as well use this generic one everywhere, but...
uint32_t IR_pulsate(uint32_t n_cycles, uint32_t on_time, uint32_t off_time)
{
	for (int i=0; i<n_cycles;i++)
	{
		nrf_gpio_pin_set(IRLED_PIN);
		nrf_delay_us(on_time);
		nrf_gpio_pin_clear(IRLED_PIN);
		nrf_delay_us(off_time);
	}
	return NRF_SUCCESS;
}

uint32_t Sony_control(uint16_t cmd)
{
	nrf_gpio_cfg_output(IRLED_PIN);
	nrf_gpio_pin_clear(IRLED_PIN);
	
	int compensated_off_period = SONY_OFF_PERIOD+3;
	int compensated_on_period = SONY_ON_PERIOD+2;
	//Auto-adjust in case you want to change on/off timings
	/* int n_pulses_pa = (2370 + SONY_OFF_PERIOD) / (SONY_ON_PERIOD+SONY_OFF_PERIOD); //Target == 2370us
	int n_pulses_sb = (582 + SONY_OFF_PERIOD) / (SONY_ON_PERIOD+ SONY_OFF_PERIOD); //Target == 582 us
	int n_pulses_lb = (1177 + SONY_OFF_PERIOD) / (SONY_ON_PERIOD+ SONY_OFF_PERIOD); //Target == 1177 us */
	
	int n_pulses_pa = (2370 + compensated_off_period) / (compensated_on_period + compensated_off_period); //Target == 2370us
	int n_pulses_sb = (582 + compensated_off_period) / (compensated_on_period + compensated_off_period); //Target == 582 us
	int n_pulses_lb = (1177 + compensated_off_period) / (compensated_on_period + compensated_off_period); //Target == 1177 us
	//6 & 13 periods => 1173 LB, 567 SB, 631 SP, 2388 pa 
	nrf_delay_ms(10); //Make sure the output has been off long(t=?) enough before starting
	
	for (int j=0;j<3;j++)
	{
		//Send preamble
		IR_pulsate(n_pulses_pa,SONY_ON_PERIOD,SONY_OFF_PERIOD);
		//nrf_delay_us(612); //612 results to 631 us
		nrf_delay_us(595);
		uint32_t n_lbs = 0;
		
		for (int i=15; i>=5; i--)
		{
			if((cmd >> i) & 0x0001)
				{
					IR_pulsate(n_pulses_lb,SONY_ON_PERIOD,SONY_OFF_PERIOD);
					nrf_delay_us(612);
					n_lbs++; //count for the pause
				}
				else
				{
					IR_pulsate(n_pulses_sb,SONY_ON_PERIOD,SONY_OFF_PERIOD);
					nrf_delay_us(612);
				}
		}
		//End burst
		IR_pulsate(n_pulses_sb,SONY_ON_PERIOD,SONY_OFF_PERIOD);
		
		//uint32_t delay_time = 45000-n_pulses_pa-n_lbs*1177-(12-n_lbs)*582-12*612;
		//nrf_delay_ms(delay_time/1000);
		uint32_t delay_time = 45000-n_pulses_pa-n_lbs*1173-(12-n_lbs)*567-12*612 -2550; //All cmds give about 2570 us too long pauses (consistent but wrong)
		nrf_delay_ms(delay_time/1000);
		nrf_delay_us(delay_time%1000);
	}
	
	return NRF_SUCCESS;
}
