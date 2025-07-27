/*
 * bsp_laser.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Admin
 */
#include "bsp_laser.h"
#include "app_signals.h"
#include "adc_monitor.h"

#ifdef HW_V120
	#define LASER_SPI				SPI4
	#define LASER_DMA				DMA2
	#define LASER_INT_DMA_STREAM	LL_DMA_STREAM_2
	#define LASER_EXT_DMA_STREAM	LL_DMA_STREAM_1
#endif

#define HW_V122
#ifdef HW_V122
	#define LASER_SPI				SPI2
	#define LASER_DMA				DMA2
	#define LASER_INT_DMA_STREAM	LL_DMA_STREAM_2
	#define LASER_EXT_DMA_STREAM	LL_DMA_STREAM_1
#endif

//static uint16_t ld_int_adc_value = 0;
//static uint16_t ld_int_adc_value_average = 0;
//static uint16_t ld_ext_adc_value = 0;
//static uint16_t ld_ext_adc_value_average = 0;
//static uint32_t sample_count = 0;

static uint16_t ld_adc_value[2] = {0};
static uint16_t ld_adc_value_average[2] = {0};
static uint32_t sample_count = 0;


MCP4902_Device_t DAC_device;
ADG1414_Device_t laser_int;
ADG1414_Device_t laser_ext;


extern monitor_task_t monitor_task_inst ;
static monitor_evt_t const ld_adc_evt = {.super = {.sig = EVT_MONITOR_LD_ADC_COMPLETED} };


static uint16_t current_calculate(uint16_t adc_val)	// return mA
{
	float temp = (adc_val* ADC_VREF * 10)/ADC_MAX;	//mV x 10 times
	temp /= ADC_RES_SHUNT;
	return (uint16_t)(temp);
}

void bsp_laser_set_spi_prescaler(uint32_t Prescaler)
{
    LL_SPI_Disable(LASER_SPI);
    LL_SPI_SetBaudRatePrescaler(LASER_SPI, Prescaler);
    LL_SPI_Enable(LASER_SPI);
}

void bsp_laser_set_spi_mode(spi_mode_t spi_mode)
{
	LL_SPI_Disable(LASER_SPI);
	switch(spi_mode)
	{
		case SPI_MODE_0:
			LL_SPI_SetClockPolarity(LASER_SPI, LL_SPI_POLARITY_LOW);
			LL_SPI_SetClockPhase(LASER_SPI, LL_SPI_PHASE_1EDGE);
		break;

		case SPI_MODE_1:
			LL_SPI_SetClockPolarity(LASER_SPI, LL_SPI_POLARITY_LOW);
			LL_SPI_SetClockPhase(LASER_SPI, LL_SPI_PHASE_2EDGE);
		break;

		case SPI_MODE_2:
			LL_SPI_SetClockPolarity(LASER_SPI, LL_SPI_POLARITY_HIGH);
			LL_SPI_SetClockPhase(LASER_SPI, LL_SPI_PHASE_1EDGE);
		break;

		case SPI_MODE_3:
			LL_SPI_SetClockPolarity(LASER_SPI, LL_SPI_POLARITY_HIGH);
			LL_SPI_SetClockPhase(LASER_SPI, LL_SPI_PHASE_2EDGE);
		break;
	}
	LL_SPI_Enable(LASER_SPI);
}

void bsp_laser_init(void)
{
	bsp_laser_set_spi_mode(SPI_MODE_0);
	MCP4902_Device_Init(&DAC_device, LASER_SPI, LASER_DAC_CS_GPIO_Port, LASER_DAC_CS_Pin, LASER_DAC_LATCH_GPIO_Port, LASER_DAC_LATCH_Pin);

	bsp_laser_set_spi_mode(SPI_MODE_1);
	ADG1414_Chain_Init(&laser_int, LASER_SPI, LASER_INT_SW_CS_GPIO_Port, LASER_INT_SW_CS_Pin, INTERNAL_CHAIN_SWITCH_NUM);
	ADG1414_Chain_Init(&laser_ext, LASER_SPI, LASER_EXT_SW_CS_GPIO_Port, LASER_EXT_SW_CS_Pin, EXTERNAL_CHAIN_SWITCH_NUM);

}

void bsp_laser_int_switch_on(uint32_t channel_idx)
{
	ADG1414_Chain_SwitchOn(&laser_int, channel_idx);
}

void bsp_laser_int_switch_off_all(void){
	ADG1414_Chain_SwitchAllOff(&laser_int);
}

void bsp_laser_ext_switch_on(uint32_t channel_idx)
{
	ADG1414_Chain_SwitchOn(&laser_ext, channel_idx);
}

void bsp_laser_ext_switch_off_all(void){
	ADG1414_Chain_SwitchAllOff(&laser_ext);
}
/*
 * current source has 250 ohm shunt
 * with maximum voltage of 3V, we calculate the voltage for ADC and send to ADC
 */

void bsp_laser_int_set_current(uint32_t percent)
{
	if (percent > 100) percent = 100;
	MCP4902_Set_DAC(&DAC_device, 0, 255*percent/100);
}

void bsp_laser_ext_set_current(uint32_t percent)
{
	if (percent > 100) percent = 100;
	MCP4902_Set_DAC(&DAC_device, 0, 255*percent/100);
}

void bsp_laser_set_current(uint32_t id, uint32_t percent)
{
	if (id == 0)  bsp_laser_int_set_current(percent);
	else bsp_laser_ext_set_current(percent);

}

uint16_t bsp_laser_get_ext_current(void)
{
	return current_calculate(ld_adc_value_average[1]);
}

uint16_t bsp_laser_get_int_current(void)
{
	return current_calculate(ld_adc_value_average[0]);
}

uint16_t bsp_laser_get_sample_int_current(void)
{
	return current_calculate(ld_adc_value[0]);
}

void bsp_laser_adc_init(void)
{
//	//Setting for laser ext adc
//    LL_DMA_SetPeriphIncMode(LASER_DMA, LASER_EXT_DMA_STREAM, LL_DMA_PERIPH_NOINCREMENT);
//    LL_DMA_SetMemoryIncMode(LASER_DMA, LASER_EXT_DMA_STREAM, LL_DMA_MEMORY_INCREMENT);
//    LL_DMA_SetPeriphSize(LASER_DMA, LASER_EXT_DMA_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
//    LL_DMA_SetMemorySize(LASER_DMA, LASER_EXT_DMA_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
//    LL_DMA_SetDataLength(LASER_DMA, LASER_EXT_DMA_STREAM, 1); // 1 channel for ext_laser
//    LL_DMA_SetPeriphAddress(LASER_DMA, LASER_EXT_DMA_STREAM, (uint32_t)&ADC3->DR);
//    LL_DMA_SetMemoryAddress(LASER_DMA, LASER_EXT_DMA_STREAM, (uint32_t)&ld_ext_adc_value);
//    LL_DMA_SetMode(LASER_DMA, LASER_EXT_DMA_STREAM, LL_DMA_MODE_CIRCULAR);
//
//    //Setting for laser int adc
//    LL_DMA_SetPeriphIncMode(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_PERIPH_NOINCREMENT);
//    LL_DMA_SetMemoryIncMode(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_MEMORY_INCREMENT);
//    LL_DMA_SetPeriphSize(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
//    LL_DMA_SetMemorySize(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
//    LL_DMA_SetDataLength(LASER_DMA, LASER_INT_DMA_STREAM, 1); // 1 channel for int_laser
//    LL_DMA_SetPeriphAddress(LASER_DMA, LASER_INT_DMA_STREAM, (uint32_t)&ADC2->DR);
//    LL_DMA_SetMemoryAddress(LASER_DMA, LASER_INT_DMA_STREAM, (uint32_t)&ld_int_adc_value);
//    LL_DMA_SetMode(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_MODE_CIRCULAR);
//
//
//    LL_ADC_Enable(ADC3);
//    LL_DMA_EnableStream(LASER_DMA, LASER_EXT_DMA_STREAM);
//    LL_DMA_EnableIT_TC(LASER_DMA, LASER_EXT_DMA_STREAM);
//
//    LL_ADC_Enable(ADC2);
//	LL_DMA_EnableStream(LASER_DMA, LASER_INT_DMA_STREAM);
//	LL_DMA_EnableIT_TC(LASER_DMA, LASER_INT_DMA_STREAM);






	// Disable stream DMA before configure
	LL_DMA_DisableStream(LASER_DMA, LASER_INT_DMA_STREAM);
	while (LL_DMA_IsEnabledStream(LASER_DMA, LASER_INT_DMA_STREAM));

	// Configure DMA
    LL_DMA_SetPeriphIncMode(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_SetDataLength(LASER_DMA, LASER_INT_DMA_STREAM, 2); // 2 kênh
    LL_DMA_SetPeriphAddress(LASER_DMA, LASER_INT_DMA_STREAM, (uint32_t)&ADC2->DR);
    LL_DMA_SetMemoryAddress(LASER_DMA, LASER_INT_DMA_STREAM, (uint32_t)ld_adc_value);
    LL_DMA_SetMode(LASER_DMA, LASER_INT_DMA_STREAM, LL_DMA_MODE_CIRCULAR);

    // Enable half-transfer và transfer-complete interrupt
    LL_DMA_EnableIT_TC(LASER_DMA, LASER_INT_DMA_STREAM);

    // Enable ADC
    LL_ADC_Enable(ADC2);

    // Enable DMA stream
    LL_DMA_EnableStream(LASER_DMA, LASER_INT_DMA_STREAM);
}

//void bsp_laser_ext_trigger_adc(void)
//{
//	LL_ADC_REG_StartConversionSWStart(ADC3);
//}
//
//void bsp_laser_int_trigger_adc(void)
//{
//	LL_ADC_REG_StartConversionSWStart(ADC2);
//}

void bsp_laser_trigger_adc(void)
{
	LL_ADC_REG_StartConversionSWStart(ADC2);
}

//void bsp_laser_dma_adc_current_int_laser_irq(void)
//{
//	if (LASER_DMA->LISR & DMA_LISR_TCIF2)
//	{
//		LASER_DMA->LIFCR = DMA_LIFCR_CTCIF2;
//	}
//	else
//	{
//		LASER_DMA->LIFCR = DMA_LIFCR_CHTIF2;		// Clear alf-transfer flag
//		LASER_DMA->LIFCR = DMA_LIFCR_CTEIF2;		// Clear Transfer-error flag
//		LASER_DMA->LIFCR = DMA_LIFCR_CDMEIF2;		// Clear Direct-mode-error flag
//		LASER_DMA->LIFCR = DMA_LIFCR_CFEIF2;		// Clear FIFO-error flag
//	}
//}

//void bsp_laser_dma_adc_current_ext_laser_irq(void)
//{
//	if (LASER_DMA->LISR & DMA_LISR_TCIF1)
//	{
//		LASER_DMA->LIFCR = DMA_LIFCR_CTCIF1;
//
//		if (sample_count == 0)
//		{
//			ld_ext_adc_value_average = ld_ext_adc_value;
//			sample_count++;
//		}
//		else if(sample_count < 10)
//		{
//			ld_ext_adc_value_average = (ld_ext_adc_value + ld_ext_adc_value_average)/2;
//			sample_count++;
//		}
//		else
//		{
//			sample_count = 0;
//			SST_Task_post(&monitor_task_inst.super, (SST_Evt *)&ld_adc_evt); //post to temperature monitor task
//		}
//	}
//	else
//	{
//		LASER_DMA->LIFCR = DMA_LIFCR_CHTIF1;		// Clear alf-transfer flag
//		LASER_DMA->LIFCR = DMA_LIFCR_CTEIF1;		// Clear Transfer-error flag
//		LASER_DMA->LIFCR = DMA_LIFCR_CDMEIF1;		// Clear Direct-mode-error flag
//		LASER_DMA->LIFCR = DMA_LIFCR_CFEIF1;		// Clear FIFO-error flag
//	}
//}

void bsp_laser_dma_adc_current_irq(void)
{
	if (LASER_DMA->LISR & DMA_LISR_TCIF2)
	{
		LASER_DMA->LIFCR = DMA_LIFCR_CTCIF2;
		if (sample_count == 0)
		{
			ld_adc_value_average[0] = ld_adc_value[0];
			ld_adc_value_average[1] = ld_adc_value[1];
			sample_count++;
		}
		else if(sample_count < 10)
		{
			ld_adc_value_average[0] = (ld_adc_value[0] + ld_adc_value_average[0])/2;
			ld_adc_value_average[1] = (ld_adc_value[1] + ld_adc_value_average[1])/2;
			sample_count++;
		}
		else
		{
			sample_count = 0;
			SST_Task_post(&monitor_task_inst.super, (SST_Evt *)&ld_adc_evt); //post to temperature monitor task
		}
	}
	else
	{
		LASER_DMA->LIFCR = DMA_LIFCR_CHTIF2;		// Clear alf-transfer flag
		LASER_DMA->LIFCR = DMA_LIFCR_CTEIF2;		// Clear Transfer-error flag
		LASER_DMA->LIFCR = DMA_LIFCR_CDMEIF2;		// Clear Direct-mode-error flag
		LASER_DMA->LIFCR = DMA_LIFCR_CFEIF2;		// Clear FIFO-error flag
	}
}


