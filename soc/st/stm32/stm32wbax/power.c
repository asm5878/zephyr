/*
 * Copyright (c) 2022 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/arch/common/pm_s2ram.h>
#include <zephyr/drivers/timer/system_timer.h>

#include <stm32wbaxx_ll_bus.h>
#include <stm32wbaxx_ll_cortex.h>
#include <stm32wbaxx_ll_pwr.h>
#include <stm32wbaxx_ll_icache.h>
#include <stm32wbaxx_ll_rcc.h>
#include <stm32wbaxx_ll_system.h>
#include <clock_control/clock_stm32_ll_common.h>

#include "stm32_lpm.h"
#ifdef CONFIG_BT_STM32WBA
#include "scm.h"
#endif

#include <zephyr/logging/log.h>
extern bool system_startup_done;
static uint32_t boot_after_standby;

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

void stm32_power_init(void);

static void disable_cache(void)
{
	/* Disabling ICACHE */
	LL_ICACHE_Disable();
	while (LL_ICACHE_IsEnabled() == 1U) {
	}
#if 0 /** ALMA: This code is only related to cut 1.0 */
	/* Wait until ICACHE_SR.BUSYF is cleared */
	while (LL_ICACHE_IsActiveFlag_BUSY() == 1U) {
	}

	/* Wait until ICACHE_SR.BSYENDF is set */
	while (LL_ICACHE_IsActiveFlag_BSYEND() == 0U) {
	}
#endif
}

static void PWR_ExitOffMode_std(void)
{
#define RADIO_INTR_PRIO_HIGH_Z_REDEF (RADIO_INTR_PRIO_HIGH + _IRQ_PRIO_OFFSET)
	/** Cube Code: PWR_ExitOffMode */
	boot_after_standby = 0;
	/** ALMA: radio interrupts to be reenabled */
	HAL_NVIC_SetPriority(RADIO_INTR_NUM, RADIO_INTR_PRIO_HIGH_Z_REDEF, 0);
	HAL_NVIC_EnableIRQ(RADIO_INTR_NUM);
	HAL_NVIC_SetPriority(RADIO_SW_LOW_INTR_NUM, RADIO_SW_LOW_INTR_PRIO, 0);
	HAL_NVIC_EnableIRQ(RADIO_SW_LOW_INTR_NUM);

	/* Enable AHB5ENR peripheral clock (bus CLK) */
	__HAL_RCC_RADIO_CLK_ENABLE();

	/* Notify the Link Layer platform layer the system exited WFI
	 * and AHB5 clock may be resynchronized as is may have been
	 * turned of during low power mode entry.
	 */
	LINKLAYER_PLAT_NotifyWFIExit();

	/* Apply Prefetch configuration is enabled */
#if (PREFETCH_ENABLE != 0U)
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */

	/** In Zephyr Peripheral reinitialization is the the border between
	 * pm_state_set and pm_state_exit_post_ops
	 * In theory all is done in cube after MX_StandbyExit_PeripheralInit
	 */
	/** ALMA: done by Zephyr LP framework */
	// MX_StandbyExit_PeripheralInit();

	/** ALMA: Should be managed byt Zephyr? Enable RTC peripheral clock */
	LL_PWR_EnableBkUpAccess();
	__HAL_RCC_RTCAPB_CLK_ENABLE();

	/* Restore system clock configuration */
#if (CFG_SCM_SUPPORTED == 1)
	scm_standbyexit();
#else
	Clock_Switching();
#endif /* CFG_SCM_SUPPORTED */

	/** Is this Cube code really required  ? */

	/* Enable RTC peripheral clock */
	LL_PWR_EnableBkUpAccess();
	__HAL_RCC_RTCAPB_CLK_ENABLE();
}
static void Exit_Stop_Standby_Mode(void)
{
	LL_ICACHE_Enable();
	while (LL_ICACHE_IsEnabled() == 0U)
		;
#if (CFG_SCM_SUPPORTED == 1)
	if (LL_PWR_IsActiveFlag_STOP() == 1U) {
#if 0
		/** This code is not relevant if SCM HSE stabilization mechanism is not in place */
		/* SCM HSE BEGIN */
		/* Clear SW_HSERDY, if needed */
		if (isRadioActive() == SCM_RADIO_NOT_ACTIVE) {
			SCM_HSE_Clear_SW_HSERDY();
		}
		/* SCM HSE END */
#endif
		scm_setup();
	} else {
		scm_setwaitstates(RUN);
	}
#else
	if (LL_PWR_IsActiveFlag_STOP() == 1U) {
		Clock_Switching();
	} else {
		/* Apply waitsates for HSE32 configuration */
		__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
		while (__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_0)
			;
		HAL_RAMCFG_ConfigWaitState(&sram1_ns, RAMCFG_WAITSTATE_0);
		HAL_RAMCFG_ConfigWaitState(&sram2_ns, RAMCFG_WAITSTATE_0);
	}
#endif /* CFG_SCM_SUPPORTED */
}
static void set_mode_stop(uint8_t substate_id)
{

	LL_PWR_ClearFlag_STOP();
	LL_RCC_ClearResetFlags();

	/* Erratum 2.2.15:
	 * Disabling ICACHE is required before entering stop mode
	 */
	disable_cache();

#ifdef CONFIG_BT_STM32WBA
	scm_setwaitstates(LP);
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

	/* Set low power mode to standby */
	LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);

	/** ALMA: At this point  we should be ready to save all the CPU context */

	/* Select mode entry : WFE or WFI and enter the CPU selected mode */
	/** CONFIG_ARCH_HAS_CUSTOM_CPU_IDLE allows to have a custom idel if required */
	k_cpu_idle();

	return 0;
}

static void set_mode_suspend_to_ram(void)
{
	/* Enable SRAM full retention */
	// LL_PWR_SetSRAM1SBRetention(LL_PWR_SRAM1_SB_FULL_RETENTION);
	// LL_PWR_SetSRAM2SBRetention(LL_PWR_SRAM2_SB_FULL_RETENTION);

	/* Enable RTC wakeup
	 * This configures an internal pin that generates an event to wakeup the system
	 */
#if 1
	LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN7);
	LL_PWR_SetWakeUpPinSignal3Selection(LL_PWR_WAKEUP_PIN7);
#endif
	/* Clear flags */
	/** ALMA: not done in Cube  */
#if 0
	LL_PWR_ClearFlag_SB();
	LL_PWR_ClearFlag_WU();
#endif
	LL_PWR_ClearFlag_STOP();

	if ((system_startup_done != FALSE) && (UTIL_LPM_GetMode() == UTIL_LPM_OFFMODE)) {
		APP_SYS_BLE_EnterDeepSleep();
	}
	LL_RCC_ClearResetFlags();

	/** ALMA: In cube FW here we should disable the systick interrupt */
	/** HAL_SuspendTick() ;*/

	/** ALMA: UTIL_LPM_EnterLowPower  */
	LINKLAYER_PLAT_NotifyWFIEnter();

	// disable_cache();

	/** ALMA: Enter_Stop_Standby_Mode */
	LL_ICACHE_Disable();

#if defined(STM32WBAXX_SI_CUT1_0)
	/* Wait until ICACHE_SR.BUSYF is cleared */
	while (LL_ICACHE_IsActiveFlag_BUSY() == 1U)
		;

	/* Wait until ICACHE_SR.BSYENDF is set */
	while (LL_ICACHE_IsActiveFlag_BSYEND() == 0U)
		;
#endif /* STM32WBAXX_SI_CUT1_0 */

#if (CFG_SCM_SUPPORTED == 1)
	scm_setwaitstates(LP);
#else
	/* SCM module is not supported, apply low power compatible waitstates */
	__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_3);
	while (__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_3)
		;
	HAL_RAMCFG_ConfigWaitState(&sram1_ns, RAMCFG_WAITSTATE_1);
	HAL_RAMCFG_ConfigWaitState(&sram2_ns, RAMCFG_WAITSTATE_1);
#endif /* CFG_SCM_SUPPORTED */

	/* Select standby mode */
	// LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);

	/* Save context and enter Standby mode */
	/**  ALMA: suspend_to_ram performs the latest soc operations
	 *   as we have in
	 *   Enter_Stop_Standby_Mode:
	 *
	 *   LL_LPM_EnableDeepSleep();
	 *   while(LL_PWR_IsActiveFlag_ACTVOS( ) == 0);
	 *
	 *   and in PWR_EnterOffMode:
	 *   LL_PWR_SetPowerMode( LL_PWR_MODE_STANDBY );
	 *
	 *   in Zephyr we have an explicit call to k_cpu_idle
	 */

	/** ALMA: arch_pm_s2ram_suspend is in charge to save the context as done by CPUcontextSave
	 * in Cube however we have:
	 * if((backup_CONTROL & CONTROL_SPSEL_Msk) == CONTROL_SPSEL_Msk)
	 *    {
	 * 	__set_CONTROL( __get_CONTROL() & ~CONTROL_SPSEL_Msk ); -> switch SP to MSP
	 *    }
	 *
	 *
	 */
	/** However CPU registres are saved before calling latest soc operations (suspend_to ram) */
	arch_pm_s2ram_suspend(suspend_to_ram);

	/** A this this point reset handler will bew responsible to restore the CPU context and
	 * restart the execution from this point
	 * IMPORTANT: when we ask RF to perform STDBY we are not sure STDBY will be really honored
	 * As link layer scheduler could prevent the STDBY state if next RF activity is enough close
	 */

	/** In cube FW the Reset_Handler is in charge to check if we are booting from stdby
	 * In Zephyr a simple marker in RAM is used to understand if we are coming from STDBY or not
	 * But are we sure the STDBY requesterd by Zephyr is honored by the LL ?
	 */
	/* Execution is restored at this point after wake up */
	/* Restore system clock as soon as we exit standby mode */
	stm32_clock_control_standby_exit();

	/** Cube Code: is_boot_from_standby */

#if (CFG_DEBUGGER_LEVEL <= 1)
	LL_DBGMCU_DisableDBGStopMode();
	LL_DBGMCU_DisableDBGStandbyMode();
#endif /* CFG_DEBUGGER_LEVEL */
	__HAL_RCC_PWR_CLK_ENABLE();

	LL_PWR_EnableUltraLowPowerMode();

	__HAL_FLASH_SLEEP_POWERDOWN_ENABLE();

	/* Ensure this is a return from Standby, and not a reset */
	if ((LL_PWR_IsActiveFlag_SB() == 1UL) && (READ_REG(RCC->CSR) == 0U)) {
		/* When exit from standby, disable IRQ so that restore and PWR_ExitOffMode are in
		 * critical section */
		__disable_irq();
		// boot_after_standby = 1;
		/** Really exiting from stddby */
		PWR_ExitOffMode_std();
	} else {
		// boot_after_standby = 0;
		/** Exiting from STOP1 mode */
		Exit_Stop_Standby_Mode();
	}
	/* Put the radio in active state */
	if ((system_startup_done != FALSE) && (UTIL_LPM_GetMode() == UTIL_LPM_OFFMODE)) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_RADIO);
		ll_sys_dp_slp_exit();
	}
	UTIL_LPM_SetOffMode(1U << CFG_LPM_LL_DEEPSLEEP, UTIL_LPM_ENABLE);
}
#endif /* CONFIG_PM_S2RAM */

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
#if 0
#ifdef CONFIG_BT_STM32WBA
	if (LL_PWR_IsActiveFlag_STOP() == 1U) {
		scm_setup();
	} else {
		scm_setwaitstates(RUN);
	}
#endif
#endif
	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		if (substate_id <= 2) {
#ifdef CONFIG_BT_STM32WBA
			if (LL_PWR_IsActiveFlag_STOP() == 1U) {
				scm_setup();
			} else {
				scm_setwaitstates(RUN);
			}
#endif
			/* Erratum 2.2.15:
			 * Enable ICACHE when exiting stop mode
			 */
			LL_ICACHE_SetMode(LL_ICACHE_1WAY);
			LL_ICACHE_Enable();
			while (LL_ICACHE_IsEnabled() == 0U) {
			}

			LL_LPM_DisableSleepOnExit();
			LL_LPM_EnableSleep();
		} else {
			LOG_DBG("Unsupported power substate-id %u", substate_id);
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

	/* When BLE is enabled, clock restoration is performed by SCM */
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

	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_PWR);

#ifdef CONFIG_DEBUG
	LL_DBGMCU_EnableDBGStandbyMode();
	LL_DBGMCU_EnaableDBGStopMode();
	LL_DBGMCU_APB7_GRP1_FreezePeriph(LL_DBGMCU_APB7_GRP1_RTC_STOP);
	LL_DBGMCU_APB7_GRP1_FreezePeriph(LL_DBGMCU_APB7_GRP1_LPTIM1_STOP);
#else
	LL_DBGMCU_DisableDBGStandbyMode();
	LL_DBGMCU_DisableDBGStopMode();
#endif

#ifdef CONFIG_PM_S2RAM
#if (CFG_LPM_STDBY_SUPPORTED > 0)
	/* Enable SRAM1, SRAM2 and RADIO retention*/
	LL_PWR_SetSRAM1SBRetention(LL_PWR_SRAM1_SB_FULL_RETENTION);
	LL_PWR_SetSRAM2SBRetention(LL_PWR_SRAM2_SB_FULL_RETENTION);
	LL_PWR_SetRadioSBRetention(LL_PWR_RADIO_SB_FULL_RETENTION); /* Retain sleep timer configuration */
#endif
#endif
	/* Enabling  Ultra Low power mode */
	LL_PWR_EnableUltraLowPowerMode();
	LL_FLASH_EnableSleepPowerDown();
}
