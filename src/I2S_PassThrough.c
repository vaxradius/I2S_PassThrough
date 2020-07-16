#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define I2S_BCLK 19
#define I2S_BCLK_CONFIG AM_HAL_PIN_19_I2SBCLK
#define I2S_WCLK 28 // 0
#define I2S_WCLK_CONFIG AM_HAL_PIN_28_I2SWCLK
#define I2S_DATA 45 // 4
#define I2S_DATA_CONFIG AM_HAL_PIN_45_I2SDAT

#define MCLK_PIN	18
#define MCLK_TIMER	5
#define MCLK_TIMER_SEG AM_HAL_CTIMER_TIMERB

#define BCLK_PIN_INV	13
#define BCLK_TIMER_INV	0
#define BCLK_TIMER_SEG_INV AM_HAL_CTIMER_TIMERB
#define BCLK_TIMER_CLOCK_INV   0x1B//TMRB0CLK is B5

#define REF_TIMER	2
#define REF_TIMER_SEG AM_HAL_CTIMER_TIMERA
#define REF_TIMER_CLOCK   0x1B//TMRA2CLK is B5

#define LRCLK_PIN	22
#define LRCLK_TIMER	3
#define LRCLK_TIMER_SEG AM_HAL_CTIMER_TIMERA
#define LRCLK_TIMER_INT AM_HAL_CTIMER_INT_TIMERA3
#define LRCLK_TIMER_CLOCK   0x15//TMRA3CLK is A2

#define SDATA_PIN   27
#define SDATA_TIMER	1
#define SDATA_TIMER_CLOCK   0x17//TMRA1CLK is A2

#define BUF_SIZE		256

//*****************************************************************************
//
// PDM configuration information.
//
//*****************************************************************************
void *PDMHandle;

am_hal_pdm_config_t g_sPdmConfig =
{
	.eClkDivider = AM_HAL_PDM_MCLKDIV_1,
	//.eClkDivider = AM_HAL_PDM_MCLKDIV_2,
	.eLeftGain = AM_HAL_PDM_GAIN_P210DB,
	.eRightGain = AM_HAL_PDM_GAIN_P210DB,
	//.ui32DecimationRate = 0xC,
	.ui32DecimationRate = 0x18,
	//.ui32DecimationRate = 0x30,
	.bHighPassEnable = 0,
	.ui32HighPassCutoff = 0xB,
	.ePDMClkSpeed = AM_HAL_PDM_CLK_750KHZ,
	//.ePDMClkSpeed = AM_HAL_PDM_CLK_1_5MHZ,
	//.ePDMClkSpeed = AM_HAL_PDM_CLK_3MHZ,
	//.ePDMClkSpeed = AM_HAL_PDM_CLK_6MHZ,
	.bInvertI2SBCLK = 0,
	.ePDMClkSource = AM_HAL_PDM_I2S_CLK,
	.bI2SEnable = 1,
	.bPDMSampleDelay = 0,
	.bDataPacking = 1,
	.ePCMChannels = AM_HAL_PDM_CHANNEL_STEREO,
	.bLRSwap = 0,
};

void pwm_out(void)
{


	//
	// Configure the output pin.
	//
	am_hal_ctimer_output_config(MCLK_TIMER,
	                            MCLK_TIMER_SEG,
	                            MCLK_PIN,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA);

	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(MCLK_TIMER,               // ui32TimerNumber
	                            MCLK_TIMER_SEG,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             AM_HAL_CTIMER_HFRC_12MHZ|
	                             AM_HAL_CTIMER_INT_ENABLE) );

	//
	// Set up initial timer periods.
	//
	am_hal_ctimer_period_set(MCLK_TIMER,
	                         MCLK_TIMER_SEG, 2, 1); //MCKL 4MHz
	am_hal_ctimer_aux_period_set(MCLK_TIMER,
	                         MCLK_TIMER_SEG, 2, 1);
//////////////////////////////////////////////////////////////////////////////////

//No need BCLK_TIMER output
#if 1
	//
	// Configure the output pin.
	//
	am_hal_ctimer_output_config(REF_TIMER,
	                            REF_TIMER_SEG,
	                            46,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);
#endif
	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(REF_TIMER,               // ui32TimerNumber
	                            REF_TIMER_SEG,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             AM_HAL_CTIMER_HFRC_12MHZ|
	                             AM_HAL_CTIMER_INT_ENABLE));
						//AM_HAL_CTIMER_INT_ENABLE | AM_HAL_CTIMER_PIN_INVERT) );

	//
	// Set up initial timer periods.
	//
	am_hal_ctimer_period_set(REF_TIMER,
	                         REF_TIMER_SEG, 11, 6);
	am_hal_ctimer_aux_period_set(REF_TIMER,
	                         REF_TIMER_SEG, 11, 6);

	//////////////////////////////////////////////////////////////////////////////////

	//
	// Configure the output pin.
	//	
	am_hal_ctimer_output_config(BCLK_TIMER_INV,
	                            BCLK_TIMER_SEG_INV,
	                            BCLK_PIN_INV,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);

	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(BCLK_TIMER_INV,               // ui32TimerNumber
	                            BCLK_TIMER_SEG_INV,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             AM_HAL_CTIMER_HFRC_12MHZ|
	                             AM_HAL_CTIMER_INT_ENABLE) );

	//
	// Set up initial timer periods.
	//
	am_hal_ctimer_period_set(BCLK_TIMER_INV,
	                         BCLK_TIMER_SEG_INV, 11, 6);//BCLK INV 1MHz
	am_hal_ctimer_aux_period_set(BCLK_TIMER_INV,
	                         BCLK_TIMER_SEG_INV, 11, 6);//BCLK INV 


	//////////////////////////////////////////////////////////////////////////////////


	//
	// Start the timer.
	//
	am_hal_ctimer_start(MCLK_TIMER, MCLK_TIMER_SEG);
	am_hal_ctimer_start(REF_TIMER, REF_TIMER_SEG);

	am_hal_ctimer_start(BCLK_TIMER_INV, BCLK_TIMER_SEG_INV);


}


void
initialize_pattern128_counter(uint32_t ui32TimerNumber,
                           uint64_t ui64Pattern0,
                           uint64_t ui64Pattern1,
                           uint32_t ui32PatternLen,
                           uint32_t ui32Trigger,
                           uint32_t ui32OutputPin,
                           uint32_t ui32PatternClock)
{
    //
    // Set up timer.
    //
    am_hal_ctimer_clear(ui32TimerNumber, AM_HAL_CTIMER_BOTH);

    am_hal_ctimer_config_single(ui32TimerNumber, AM_HAL_CTIMER_BOTH,
                              (AM_HAL_CTIMER_FN_PTN_REPEAT    |
                               ui32PatternClock) );

    //
    // Set the pattern in the CMPR registers.
    //
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 0, 
                            (uint32_t)(ui64Pattern0 & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 1, 
                            (uint32_t)((ui64Pattern0 >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 0, 
                            (uint32_t)((ui64Pattern0 >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 1, 
                            (uint32_t)((ui64Pattern0 >> 48) & 0xFFFF));

    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 0, 
                            (uint32_t)(ui64Pattern1 & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 1, 
                            (uint32_t)((ui64Pattern1 >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 0, 
                            (uint32_t)((ui64Pattern1 >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 1, 
                            (uint32_t)((ui64Pattern1 >> 48) & 0xFFFF));
    //
    // Set the timer trigger and pattern length.
    //
    am_hal_ctimer_config_trigger(ui32TimerNumber, AM_HAL_CTIMER_BOTH,
                               ( (ui32PatternLen << CTIMER_AUX0_TMRA0LMT_Pos) |
                                 ( ui32Trigger << CTIMER_AUX0_TMRA0TRIG_Pos) ) );

    //
    // Configure timer output pin.
    //
    am_hal_ctimer_output_config(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, ui32OutputPin, 
                              AM_HAL_CTIMER_OUTPUT_NORMAL, 
                              AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);
    //
    // Start the timer.
    //
    am_hal_ctimer_start(ui32TimerNumber, AM_HAL_CTIMER_BOTH);
}

void
initialize_pattern64_counter(uint32_t ui32TimerNumber,
                           uint32_t ui32TimerSegment,
                           uint64_t ui64Pattern,
                           uint32_t ui32PatternLen,
                           uint32_t ui32Trigger,
                           uint32_t ui32OutputPin,
                           uint32_t ui32PatternClock)
{
    //
    // Set up timer.
    //
    am_hal_ctimer_clear(ui32TimerNumber, ui32TimerSegment);

    am_hal_ctimer_config_single(ui32TimerNumber, ui32TimerSegment,
                              (AM_HAL_CTIMER_FN_PTN_REPEAT    |AM_HAL_CTIMER_INT_ENABLE |
                               ui32PatternClock) );

    //
    // Set the pattern in the CMPR registers.
    //
    am_hal_ctimer_compare_set(ui32TimerNumber, ui32TimerSegment, 0, 
                            (uint32_t)(ui64Pattern & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, ui32TimerSegment, 1, 
                            (uint32_t)((ui64Pattern >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, ui32TimerSegment, 0, 
                            (uint32_t)((ui64Pattern >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, ui32TimerSegment, 1, 
                            (uint32_t)((ui64Pattern >> 48) & 0xFFFF));
    //
    // Set the timer trigger and pattern length.
    //
    am_hal_ctimer_config_trigger(ui32TimerNumber, ui32TimerSegment,
                               ( (ui32PatternLen << CTIMER_AUX0_TMRA0LMT_Pos) |
                                 ( ui32Trigger << CTIMER_AUX0_TMRA0TRIG_Pos) ) );

    //
    // Configure timer output pin.
    //
    am_hal_ctimer_output_config(ui32TimerNumber, ui32TimerSegment, ui32OutputPin, 
                              AM_HAL_CTIMER_OUTPUT_NORMAL, 
                              AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);

    //
    // Start the timer.
    //
    am_hal_ctimer_start(ui32TimerNumber, ui32TimerSegment);
}

void
global_disable(void)
{
    CTIMER->GLOBEN = 0x0;
}

void
global_enable(void)
{
    CTIMER->GLOBEN = 0xffff;
}

void I2S_init(void)
{
	am_hal_gpio_pincfg_t gpio_config = {0};

	//
	// Disable all the counters.
	//
	global_disable();


	initialize_pattern128_counter(SDATA_TIMER, 0x0000000000000000, 0x0000000000000000, 
	               127, CTIMER_AUX0_TMRB0TRIG_DIS, SDATA_PIN,
	               _VAL2FLD(CTIMER_CTRL0_TMRA0CLK, SDATA_TIMER_CLOCK));//Timer1 Clock source is CTIMERA2 OUT
	initialize_pattern64_counter(LRCLK_TIMER, LRCLK_TIMER_SEG, 0xFFFFFFFF00000000, 
	               63, CTIMER_AUX0_TMRB0TRIG_DIS, LRCLK_PIN,
	               _VAL2FLD(CTIMER_CTRL0_TMRA0CLK, LRCLK_TIMER_CLOCK));//Timer3 Clock source is CTIMERA2 OUT

	pwm_out();	
	global_enable();//I2S starts

	gpio_config.uFuncSel = I2S_BCLK_CONFIG;
	gpio_config.eGPInput = 1;
	am_hal_gpio_pinconfig(I2S_BCLK, gpio_config);
	
	gpio_config.uFuncSel = I2S_WCLK_CONFIG;
	gpio_config.eGPInput = 1;
	am_hal_gpio_pinconfig(I2S_WCLK, gpio_config);
	
	gpio_config.uFuncSel = I2S_DATA_CONFIG;
	gpio_config.eGPInput = 0;
	am_hal_gpio_pinconfig(I2S_DATA, gpio_config);
	
	
}


//*****************************************************************************
//
// PDM initialization.
//
//*****************************************************************************
void
pdm_init(void)
{
	//
	// Initialize, power-up, and configure the PDM.
	//
	am_hal_pdm_initialize(0, &PDMHandle);
	am_hal_pdm_power_control(PDMHandle, AM_HAL_PDM_POWER_ON, false);
	am_hal_pdm_configure(PDMHandle, &g_sPdmConfig);
	am_hal_pdm_enable(PDMHandle);

	//
	// Configure the necessary pins.
	//
	am_hal_gpio_pincfg_t sPinCfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	sPinCfg.uFuncSel = AM_HAL_PIN_12_PDMCLK;
	sPinCfg.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA;
	am_hal_gpio_pinconfig(12, sPinCfg);

	sPinCfg.uFuncSel = AM_HAL_PIN_11_PDMDATA;
	am_hal_gpio_pinconfig(11, sPinCfg);

}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
	//
	// Perform the standard initialzation for clocks, cache settings, and
	// board-level low-power operation.
	//
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();
	am_bsp_low_power_init();


	am_hal_interrupt_master_disable();

	I2S_init();
	

	pdm_init();


	//
	// Loop forever while sleeping.
	//
	while (1)
	{

		//
		// Go to Deep Sleep.
		//
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
	}
}

