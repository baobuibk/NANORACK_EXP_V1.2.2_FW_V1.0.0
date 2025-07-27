/*
 * bsp_ntc.c
 *
 *  Created on: Jun 22, 2025
 *      Author: Admin
 */


#include "bsp_ntc.h"
#include "ntc.h"
#include "stm32f7xx_ll_dma.h"
#include "app_signals.h"

#include "adc_monitor.h"
extern monitor_task_t monitor_task_inst;

#ifdef	HW_V120
#define NTC_DMA				DMA2
#define NTC_DMA_STREAM		LL_DMA_STREAM_0
#endif

#define HW_V122
#ifdef	HW_V122
#define NTC_DMA				DMA2
#define NTC_DMA_STREAM		LL_DMA_STREAM_4
#endif

static ntc_confi_t ntc_config = {
		.dma = NTC_DMA,
		.dma_stream = NTC_DMA_STREAM
};

static uint16_t ntc_ADC_value[8] = {0,0,0,0,0,0,0,0};
static uint16_t ntc_ADC_value_average[8] = {0,0,0,0,0,0,0,0};
static uint32_t sample_count = 0;

static monitor_evt_t const ntc_adc_evt = {.super = {.sig = EVT_MONITOR_NTC_ADC_COMPLETED} };

void bsp_ntc_adc_init(void)
{
	ntc_adc_dma_init(&ntc_config, ntc_ADC_value, 8);
//	ntc_adc_dma_init(ntc_ADC_value, 8);
}

void bsp_ntc_trigger_adc(void)
{
	ntc_adc_trigger();
}

int16_t bsp_ntc_get_temperature(uint32_t ntc_channel)
{
	return ntc_convert(ntc_ADC_value_average[ntc_channel]);
}


void bsp_ntc_dma_adc_irq(void)
{
#ifdef HW_V120
	// Transfer-complete (stream 0)
	if (NTC_DMA->LISR & DMA_LISR_TCIF0)
	{
		NTC_DMA->LIFCR = DMA_LIFCR_CTCIF0;	// Clear Transfer-complete flag
		if (sample_count == 0)
		{
			for (uint32_t i = 0; i < 8; i++ ) ntc_ADC_value_average[i] = ntc_ADC_value[i];
			sample_count++;
		}
		else if(sample_count < 10)
		{
			for (uint32_t i = 0; i< 8; i++ ) ntc_ADC_value_average[i] = (ntc_ADC_value[i] + ntc_ADC_value_average[i])/2;
			sample_count++;
		}
		else
		{
			sample_count = 0;
			SST_Task_post(&monitor_task_inst.super, (SST_Evt *)&ntc_adc_evt); //post to temperature monitor task
		}
	}

	// Clear all unexpected interrupt flag (stream 0)
	else
	{
		NTC_DMA->LIFCR = DMA_LIFCR_CHTIF0;		// Clear alf-transfer flag
		NTC_DMA->LIFCR = DMA_LIFCR_CTEIF0;		// Clear Transfer-error flag
		NTC_DMA->LIFCR = DMA_LIFCR_CDMEIF0;		// Clear Direct-mode-error flag
		NTC_DMA->LIFCR = DMA_LIFCR_CFEIF0;		// Clear FIFO-error flag
	}
#endif

#ifdef HW_V122
	// Transfer-complete (stream 4)
	if (NTC_DMA->HISR & DMA_HISR_TCIF4)
	{
		NTC_DMA->HIFCR = DMA_HIFCR_CTCIF4;	// Clear Transfer-complete flag
		if (sample_count == 0)
		{
			for (uint32_t i = 0; i < 8; i++ ) ntc_ADC_value_average[i] = ntc_ADC_value[i];
			sample_count++;
		}
		else if(sample_count < 10)
		{
			for (uint32_t i = 0; i< 8; i++ ) ntc_ADC_value_average[i] = (ntc_ADC_value[i] + ntc_ADC_value_average[i])/2;
			sample_count++;
		}
		else
		{
			sample_count = 0;
			SST_Task_post(&monitor_task_inst.super, (SST_Evt *)&ntc_adc_evt); //post to temperature monitor task
		}
	}

	// Clear all unexpected interrupt flag (stream 4)
	else
	{
		NTC_DMA->HIFCR = DMA_HIFCR_CHTIF4;		// Clear alf-transfer flag
		NTC_DMA->HIFCR = DMA_HIFCR_CTEIF4;		// Clear Transfer-error flag
		NTC_DMA->HIFCR = DMA_HIFCR_CDMEIF4;		// Clear Direct-mode-error flag
		NTC_DMA->HIFCR = DMA_HIFCR_CFEIF4;		// Clear FIFO-error flag
	}
#endif
}
