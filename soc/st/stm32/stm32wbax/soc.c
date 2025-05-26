/*
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for STM32WBA processor
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_icache.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include "soc.h"
#include <cmsis_core.h>
#include <otp.h>
#include "utilities_common.h"

#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_REGISTER(soc);

#if (CFG_LPM_LEVEL != 0)
bool system_startup_done = FALSE;
#endif /* ( CFG_LPM_LEVEL != 0) */

extern void UTIL_LPM_Init(void);
extern void Error_Handler(void);

RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

static void Config_HSE(void)
{
	OTP_Data_s *otp_ptr = NULL;

	/* Read HSE_Tuning from OTP */
	if (OTP_Read(DEFAULT_OTP_IDX, &otp_ptr) != HAL_OK) {
		/* OTP no present in flash, apply default gain */
		HAL_RCCEx_HSESetTrimming(0x0C);
	} else {
		HAL_RCCEx_HSESetTrimming(otp_ptr->hsetune);
	}
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 */
void stm32wba_init(void)
{
#ifdef CONFIG_STM32_FLASH_PREFETCH
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif
	/** ALMA: Is this setting managed by Zephyr */
	/* HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); */
	/* Enable instruction cache in 1-way (direct mapped cache) */

	/** ALMA: In Cube this code is called later in the initialization  */
	// LL_ICACHE_SetMode(LL_ICACHE_1WAY);
	// LL_ICACHE_Enable();

	/* Update CMSIS SystemCoreClock variable (HCLK) */
	/* At reset, system core clock is set to 16 MHz from HSI */
	/* ALMA: In cube we calculate this value using SystemCoreClockUpdate */
	SystemCoreClock = 16000000;

	/* Enable PWR */
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_PWR);

	Config_HSE();

#if defined(CONFIG_POWER_SUPPLY_DIRECT_SMPS)
	LL_PWR_SetRegulatorSupply(LL_PWR_SMPS_SUPPLY);
#elif defined(CONFIG_POWER_SUPPLY_LDO)
	LL_PWR_SetRegulatorSupply(LL_PWR_LDO_SUPPLY);
#endif
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Configure LSE Drive Capability */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);

	__HAL_RCC_RADIOSLPTIM_CONFIG(RCC_RADIOSTCLKSOURCE_LSE);
#if 0
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType =
		RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
#endif
#if 0
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
				      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
				      RCC_CLOCKTYPE_PCLK7 | RCC_CLOCKTYPE_HCLK5;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB7CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHB5_PLL1_CLKDivider = RCC_SYSCLK_PLL1_DIV1;
	RCC_ClkInitStruct.AHB5_HSEHSI_CLKDivider = RCC_SYSCLK_HSEHSI_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
#endif
	/** ALMA: devices initializations not done as managed by Zephyr */
	// MX_GPIO_Init();
	// MX_GPDMA1_Init();
	// MX_RAMCFG_Init();
	// MX_RTC_Init();
	// MX_RNG_Init();
	// MX_RAMCFG_Init();

	/* Really required ? */
	__HAL_RCC_RAMCFG_CLK_ENABLE();
	/** ALMA: MX_ICACHE_Init functions */
	/** Full retention for ICACHE in stop mode
	 */
	LL_PWR_SetICacheRAMStopRetention(LL_PWR_ICACHERAM_STOP_FULL_RETENTION);

	/** Enable instruction cache in 1-way (direct mapped cache)
	 */
	LL_ICACHE_SetMode(LL_ICACHE_1WAY);
	LL_ICACHE_Enable();

	/** ALMA: System_Init  */
	LL_RCC_ClearResetFlags();

	/** ALMA: System_Init  */
	/* Enable wakeup out of standby from RTC ( UTIL_TIMER ) */
	/** This code is done always in cube so we have removed this part in power.c  */
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_3);

	/** ALMA: System_Init  */
	/** ALMA:  This piece of code wiil be necessary if we use the Cube LPM */
#if (CFG_LPM_LEVEL != 0)
	system_startup_done = TRUE;
	UNUSED(system_startup_done);
#endif /* ( CFG_LPM_LEVEL != 0) */
/** ALMA: scm module in cube is always initialized and not only we have the PM in place */
#if (CFG_SCM_SUPPORTED == 1)
#ifdef CONFIG_BT_STM32WBA
	scm_init();
#endif
#endif

#if (CFG_LPM_LEVEL != 0)
	/* Initialize Cube low Power Manager. By default enabled */
	/** ALMA: LPM is not used  in zephyr */
	UTIL_LPM_Init();

#if (CFG_LPM_STDBY_SUPPORTED > 0)
	/* Enable SRAM1, SRAM2 and RADIO retention*/
	LL_PWR_SetSRAM1SBRetention(LL_PWR_SRAM1_SB_FULL_RETENTION);
	LL_PWR_SetSRAM2SBRetention(LL_PWR_SRAM2_SB_FULL_RETENTION);
	LL_PWR_SetRadioSBRetention(LL_PWR_RADIO_SB_FULL_RETENTION); /* Retain sleep timer configuration */
#else                                            /* (CFG_LPM_STDBY_SUPPORTED > 0) */
	UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_DISABLE);
#endif                                           /* (CFG_LPM_STDBY_SUPPORTED > 0) */
#endif                                           /* (CFG_LPM_LEVEL != 0)  */
}

void soc_early_init_hook(void)
{
	stm32wba_init();
#if CONFIG_PM
	stm32_power_init();
#endif
}
