/*
 * Copyright (c) 2022 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/pm/pm.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/arch/common/pm_s2ram.h>
#include <zephyr/drivers/timer/system_timer.h>

#include <stm32wbaxx_ll_bus.h>
#include <stm32wbaxx_ll_cortex.h>
#include <stm32wbaxx_ll_pwr.h>
#include <stm32wbaxx_ll_rcc.h>
#include <stm32wbaxx_ll_system.h>
#include <clock_control/clock_stm32_ll_common.h>

#ifdef CONFIG_BT_STM32WBA
#include "app_conf.h"
#if (CFG_SCM_SUPPORTED == 1)
#include "scm.h"
#endif
#endif

#include <zephyr/logging/log.h>

#ifdef CONFIG_BT_STM32WBA
#include "linklayer_plat.h"
#endif

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

#if (CFG_SCM_SUPPORTED == 0)
/* As SCM is not supported, SRAM handles for waitsate configurations are defined here */
static RAMCFG_HandleTypeDef sram1_ns = {
	RAMCFG_SRAM1,           /* Instance */
	HAL_RAMCFG_STATE_READY, /* RAMCFG State */
	0U,                     /* RAMCFG Error Code */
};

static RAMCFG_HandleTypeDef sram2_ns = {
	RAMCFG_SRAM2,           /* Instance */
	HAL_RAMCFG_STATE_READY, /* RAMCFG State */
	0U,                     /* RAMCFG Error Code */
};

__attribute__((optimize("Ofast"))) static void Clock_Switching(void)
{
  /* Activate HSE clock */
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() == 0);

  /* Apply PWR VOS1 power level */
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() == 0);

  /* Switch HSE frequency from HSE16 to HSE32 */
  LL_RCC_HSE_DisablePrescaler();

  /* Switch CPU system clock to HSE */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE);

  /* Apply HSE32 compatible waitstates */
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
  while(__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_0);
  HAL_RAMCFG_ConfigWaitState(&sram1_ns, RAMCFG_WAITSTATE_0);
  HAL_RAMCFG_ConfigWaitState(&sram2_ns, RAMCFG_WAITSTATE_0);

  /* Set HDIV 5 */
  LL_RCC_SetAHB5Divider(LL_RCC_AHB5_DIVIDER_1); /* divided by 1 */

  /* Ensure time base clock coherency */
  SystemCoreClockUpdate();
}
#endif /* CFG_SCM_SUPPORTED */

void stm32_power_init(void);

static void set_mode_stop(uint8_t substate_id)
{

	LL_PWR_ClearFlag_STOP();
	LL_RCC_ClearResetFlags();

	/* Erratum 2.2.15:
	 * Disabling ICACHE is required before entering stop mode
	 */
	sys_cache_instr_disable();

#ifdef CONFIG_BT_STM32WBA
	LINKLAYER_PLAT_NotifyWFIEnter();
#if (CFG_SCM_SUPPORTED == 1)
	scm_setwaitstates(LP);
#else
	/* SCM module is not supported, apply low power compatible waitstates */
	__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_3);
	while (__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_3)
		;
	HAL_RAMCFG_ConfigWaitState(&sram1_ns, RAMCFG_WAITSTATE_1);
	HAL_RAMCFG_ConfigWaitState(&sram2_ns, RAMCFG_WAITSTATE_1);
#endif
#endif
	/* Set SLEEPDEEP bit of Cortex System Control Register */
	LL_LPM_EnableDeepSleep();

	while (LL_PWR_IsActiveFlag_ACTVOS() == 0) {
	}

	switch (substate_id) {
	case 1: /* enter STOP0 mode */
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP0);
		break;
	case 2: /* enter STOP1 mode */
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
		break;
	default:
		LOG_DBG("Unsupported power state substate-id %u", substate_id);
		break;
	}
}

#if defined(CONFIG_PM_S2RAM)
static int suspend_to_ram(void)
{
	LL_LPM_EnableDeepSleep();

	while (LL_PWR_IsActiveFlag_ACTVOS() == 0) {
	}

	/* Select mode entry : WFE or WFI and enter the CPU selected mode */
	k_cpu_idle();

	return 0;
}

static void set_mode_suspend_to_ram(void)
{
	/* Enable SRAM full retention */
	LL_PWR_SetSRAM1SBRetention(LL_PWR_SRAM1_SB_FULL_RETENTION);
	LL_PWR_SetSRAM2SBRetention(LL_PWR_SRAM2_SB_FULL_RETENTION);

	/* Enable RTC wakeup
	 * This configures an internal pin that generates an event to wakeup the system
	 */
	LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN7);
	LL_PWR_SetWakeUpPinSignal3Selection(LL_PWR_WAKEUP_PIN7);

	/* Clear flags */
	LL_PWR_ClearFlag_SB();
	LL_PWR_ClearFlag_WU();
	LL_RCC_ClearResetFlags();

	sys_cache_instr_disable();

	/* Select standby mode */
	LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);

	/* Save context and enter Standby mode */
	arch_pm_s2ram_suspend(suspend_to_ram);

	/* Execution is restored at this point after wake up */
	/* Restore system clock as soon as we exit standby mode */
	stm32_clock_control_standby_exit();
}
#endif

/* Invoke Low Power/System Off specific Tasks */
void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		set_mode_stop(substate_id);

		/* Select mode entry : WFE or WFI and enter the CPU selected mode */
		k_cpu_idle();

		break;
#if defined(CONFIG_PM_S2RAM)
	case PM_STATE_SUSPEND_TO_RAM:
		set_mode_suspend_to_ram();
		break;
#endif
	default:
		LOG_DBG("Unsupported power state %u", state);
		return;
	}
}

/* Handle SOC specific activity after Low Power Mode Exit */
void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
#ifdef CONFIG_BT_STM32WBA
	LINKLAYER_PLAT_NotifyWFIExit();
	if (LL_PWR_IsActiveFlag_STOP() == 1U) {
#if (CFG_SCM_SUPPORTED == 1)
		scm_setup();
#else
		Clock_Switching();
#endif
	} else {
#if (CFG_SCM_SUPPORTED == 1)
		scm_setwaitstates(RUN);
#else
		/* Apply waitsates for HSE32 configuration */
		__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
		while (__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_0)
			;
		HAL_RAMCFG_ConfigWaitState(&sram1_ns, RAMCFG_WAITSTATE_0);
		HAL_RAMCFG_ConfigWaitState(&sram2_ns, RAMCFG_WAITSTATE_0);
#endif
	}
#endif /* CONFIG_BT_STM32WBA */

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		if (substate_id <= 2) {
			/* Erratum 2.2.15:
			 * Enable ICACHE when exiting stop mode
			 */
			sys_cache_instr_enable();

			LL_LPM_DisableSleepOnExit();
			LL_LPM_EnableSleep();
		} else {
			LOG_DBG("Unsupported power substate-id %u",
							substate_id);
		}
		break;
	case PM_STATE_SUSPEND_TO_RAM:
#if defined(CONFIG_PM_S2RAM)
		stm32wba_init();
		stm32_power_init();

		LL_LPM_DisableSleepOnExit();
		LL_LPM_EnableSleep();
#else
		LOG_DBG("Suspend to RAM needs CONFIG_PM_S2RAM to be enabled");
#endif
		break;
	case PM_STATE_STANDBY:
		__fallthrough;
	case PM_STATE_SUSPEND_TO_DISK:
		__fallthrough;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	/* When BLE is enabled, clock restoration is performed by Clock_Switching  */
#if !defined(CONFIG_BT_STM32WBA)
	stm32_clock_control_init(NULL);
#endif

	/*
	 * System is now in active mode.
	 * Reenable interrupts which were disabled
	 * when OS started idling code.
	 */
	irq_unlock(0);
}

/* Initialize STM32 Power */
void stm32_power_init(void)
{

#ifdef CONFIG_BT_STM32WBA
#if (CFG_SCM_SUPPORTED == 1)
	scm_init();
#endif
#endif

	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_PWR);

#ifdef CONFIG_DEBUG
	LL_DBGMCU_EnableDBGStandbyMode();
	LL_DBGMCU_APB7_GRP1_FreezePeriph(LL_DBGMCU_APB7_GRP1_RTC_STOP);
	LL_DBGMCU_APB7_GRP1_FreezePeriph(LL_DBGMCU_APB7_GRP1_LPTIM1_STOP);
#else
	LL_DBGMCU_DisableDBGStandbyMode();
#endif

#ifdef CONFIG_BT_STM32WBA
#if (CFG_SCM_SUPPORTED == 0)
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (FLASH_LATENCY_0));
#endif
#endif
	/* Enabling  Ultra Low power mode */
	LL_PWR_EnableUltraLowPowerMode();

	LL_FLASH_EnableSleepPowerDown();
}
