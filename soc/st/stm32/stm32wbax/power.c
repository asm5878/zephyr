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
#include <zephyr/logging/log.h>


#ifdef CONFIG_BT_STM32WBA
#include "stm32_lpm.h"
#include "app_conf.h"
#include "app_sys.h"
#include "ll_sys.h"
#if (CFG_SCM_SUPPORTED == 1)
#include "scm.h"
#endif
#endif

#if defined(CONFIG_PM_S2RAM)
#include <zephyr/linker/linker-defs.h>
#include <zephyr/sys/barrier.h>
extern int z_arm_fault_init(void);
extern int z_arm_cpu_idle_init(void);
extern int z_arm_mpu_init(void);
extern int z_arm_configure_static_mpu_regions(void);
#endif

#ifdef CONFIG_BT_STM32WBA
extern bool system_startup_done;
static uint32_t boot_after_standby;
extern void LINKLAYER_PLAT_NotifyWFIExit(void);
extern void LINKLAYER_PLAT_NotifyWFIEnter(void);
#endif

#ifdef CONFIG_BT_STM32WBA
#if (CFG_SCM_SUPPORTED == 0)
/* If SCM is not supported, SRAM handles for waitsate configurations are defined here */
static RAMCFG_HandleTypeDef sram1_ns =
{
  RAMCFG_SRAM1,           /* Instance */
  HAL_RAMCFG_STATE_READY, /* RAMCFG State */
  0U,                     /* RAMCFG Error Code */
};

static RAMCFG_HandleTypeDef sram2_ns =
{
  RAMCFG_SRAM2,           /* Instance */
  HAL_RAMCFG_STATE_READY, /* RAMCFG State */
  0U,                     /* RAMCFG Error Code */
};
#endif /* CFG_SCM_SUPPORTED */
#endif

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

#if defined(CONFIG_PM_S2RAM)
#if defined(CONFIG_CPU_HAS_FPU)
static inline void stm32wbax_z_arm_floating_point_init(void)
{
	/*
	 * Upon reset, the Co-Processor Access Control Register is, normally,
	 * 0x00000000. However, it might be left un-cleared by firmware running
	 * before Zephyr boot.
	 */
	SCB->CPACR &= (~(CPACR_CP10_Msk | CPACR_CP11_Msk));

#if defined(CONFIG_FPU)
	/*
	 * Enable CP10 and CP11 Co-Processors to enable access to floating
	 * point registers.
	 */
#if defined(CONFIG_USERSPACE)
	/* Full access */
	SCB->CPACR |= CPACR_CP10_FULL_ACCESS | CPACR_CP11_FULL_ACCESS;
#else
	/* Privileged access only */
	SCB->CPACR |= CPACR_CP10_PRIV_ACCESS | CPACR_CP11_PRIV_ACCESS;
#endif  /* CONFIG_USERSPACE */
	/*
	 * Upon reset, the FPU Context Control Register is 0xC0000000
	 * (both Automatic and Lazy state preservation is enabled).
	 */
#if defined(CONFIG_MULTITHREADING) && !defined(CONFIG_FPU_SHARING)
	/* Unshared FP registers (multithreading) mode. We disable the
	 * automatic stacking of FP registers (automatic setting of
	 * FPCA bit in the CONTROL register), upon exception entries,
	 * as the FP registers are to be used by a single context (and
	 * the use of FP registers in ISRs is not supported). This
	 * configuration improves interrupt latency and decreases the
	 * stack memory requirement for the (single) thread that makes
	 * use of the FP co-processor.
	 */
	FPU->FPCCR &= (~(FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk));
#else
	/*
	 * FP register sharing (multithreading) mode or single-threading mode.
	 *
	 * Enable both automatic and lazy state preservation of the FP context.
	 * The FPCA bit of the CONTROL register will be automatically set, if
	 * the thread uses the floating point registers. Because of lazy state
	 * preservation the volatile FP registers will not be stacked upon
	 * exception entry, however, the required area in the stack frame will
	 * be reserved for them. This configuration improves interrupt latency.
	 * The registers will eventually be stacked when the thread is swapped
	 * out during context-switch or if an ISR attempts to execute floating
	 * point instructions.
	 */
	FPU->FPCCR = FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk;
#endif /* CONFIG_FPU_SHARING */

	/* Make the side-effects of modifying the FPCCR be realized
	 * immediately.
	 */
	barrier_dsync_fence_full();
	barrier_isync_fence_full();

	/* Initialize the Floating Point Status and Control Register. */
#if defined(CONFIG_ARMV8_1_M_MAINLINE)
	/*
	 * For ARMv8.1-M with FPU, the FPSCR[18:16] LTPSIZE field must be set
	 * to 0b100 for "Tail predication not applied" as it's reset value
	 */
	__set_FPSCR(4 << FPU_FPDSCR_LTPSIZE_Pos);
#else
	__set_FPSCR(0);
#endif

	/*
	 * Note:
	 * The use of the FP register bank is enabled, however the FP context
	 * will be activated (FPCA bit on the CONTROL register) in the presence
	 * of floating point instructions.
	 */

#endif /* CONFIG_FPU */

	/*
	 * Upon reset, the CONTROL.FPCA bit is, normally, cleared. However,
	 * it might be left un-cleared by firmware running before Zephyr boot.
	 * We must clear this bit to prevent errors in exception unstacking.
	 *
	 * Note:
	 * In Sharing FP Registers mode CONTROL.FPCA is cleared before switching
	 * to main, so it may be skipped here (saving few boot cycles).
	 *
	 * If CONFIG_INIT_ARCH_HW_AT_BOOT is set, CONTROL is cleared at reset.
	 */
#if (!defined(CONFIG_FPU) || !defined(CONFIG_FPU_SHARING)) &&                                      \
	(!defined(CONFIG_INIT_ARCH_HW_AT_BOOT))

	__set_CONTROL(__get_CONTROL() & (~(CONTROL_FPCA_Msk)));
#endif
}

#endif /* CONFIG_CPU_HAS_FPU */



/**
 * @brief Clear Fault exceptions
 *
 * Clear out exceptions for Mem, Bus, Usage and Hard Faults
 */
static ALWAYS_INLINE void stm32wbax_z_arm_clear_faults(void)
{
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	/* Reset all faults */
	SCB->CFSR = SCB_CFSR_USGFAULTSR_Msk | SCB_CFSR_MEMFAULTSR_Msk | SCB_CFSR_BUSFAULTSR_Msk;

	/* Clear all Hard Faults - HFSR is write-one-to-clear */
	SCB->HFSR = 0xffffffff;
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
}



#define VECTOR_ADDRESS ((uintptr_t)_vector_start)

/* In some Cortex-M3 implementations SCB_VTOR bit[29] is called the TBLBASE bit */
#ifdef SCB_VTOR_TBLBASE_Msk
#define VTOR_MASK (SCB_VTOR_TBLBASE_Msk | SCB_VTOR_TBLOFF_Msk)
#else
#define VTOR_MASK SCB_VTOR_TBLOFF_Msk
#endif

static inline void stm32wbax_relocate_vector_table(void)
{
	SCB->VTOR = VECTOR_ADDRESS & VTOR_MASK;
	barrier_dsync_fence_full();
	barrier_isync_fence_full();
}


static ALWAYS_INLINE void stm32wbax_z_arm_exc_setup(void)
{
	/* PendSV is set to lowest priority, regardless of it being used.
	 * This is done as the IRQ is always enabled.
	 */
	NVIC_SetPriority(PendSV_IRQn, _EXC_PENDSV_PRIO);

#ifdef CONFIG_CPU_CORTEX_M_HAS_BASEPRI
	/* Note: SVCall IRQ priority level is left to default (0)
	 * for Cortex-M variants without BASEPRI (e.g. ARMv6-M).
	 */
	NVIC_SetPriority(SVCall_IRQn, _EXC_SVC_PRIO);
#endif

#ifdef CONFIG_CPU_CORTEX_M_HAS_PROGRAMMABLE_FAULT_PRIOS
	NVIC_SetPriority(MemoryManagement_IRQn, _EXC_FAULT_PRIO);
	NVIC_SetPriority(BusFault_IRQn, _EXC_FAULT_PRIO);
	NVIC_SetPriority(UsageFault_IRQn, _EXC_FAULT_PRIO);
#if defined(CONFIG_CORTEX_M_DEBUG_MONITOR_HOOK)
	NVIC_SetPriority(DebugMonitor_IRQn, IRQ_PRIO_LOWEST);
#elif defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	NVIC_SetPriority(DebugMonitor_IRQn, _EXC_FAULT_PRIO);
#endif
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
	NVIC_SetPriority(SecureFault_IRQn, _EXC_FAULT_PRIO);
#endif /* CONFIG_ARM_SECURE_FIRMWARE */

	/* Enable Usage, Mem, & Bus Faults */
	SCB->SHCSR |=
		SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk;
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
	/* Enable Secure Fault */
	SCB->SHCSR |= SCB_SHCSR_SECUREFAULTENA_Msk;
	/* Clear BFAR before setting BusFaults to target Non-Secure state. */
	SCB->BFAR = 0;
#endif /* CONFIG_ARM_SECURE_FIRMWARE */
#endif /* CONFIG_CPU_CORTEX_M_HAS_PROGRAMMABLE_FAULT_PRIOS */

#if defined(CONFIG_ARM_SECURE_FIRMWARE) && !defined(CONFIG_ARM_SECURE_BUSFAULT_HARDFAULT_NMI)
	/* Set NMI, Hard, and Bus Faults as Non-Secure.
	 * NMI and Bus Faults targeting the Secure state will
	 * escalate to a SecureFault or SecureHardFault.
	 */
	SCB->AIRCR =
		(SCB->AIRCR & (~(SCB_AIRCR_VECTKEY_Msk))) | SCB_AIRCR_BFHFNMINS_Msk |
		((AIRCR_VECT_KEY_PERMIT_WRITE << SCB_AIRCR_VECTKEY_Pos) & SCB_AIRCR_VECTKEY_Msk);
	/* Note: Fault conditions that would generate a SecureFault
	 * in a PE with the Main Extension instead generate a
	 * SecureHardFault in a PE without the Main Extension.
	 */
#endif /* ARM_SECURE_FIRMWARE && !ARM_SECURE_BUSFAULT_HARDFAULT_NMI */

#if defined(CONFIG_CPU_CORTEX_M_HAS_SYSTICK) && !defined(CONFIG_CORTEX_M_SYSTICK)
	/* SoC implements SysTick, but the system does not use it
	 * as driver for system timing. However, the SysTick IRQ is
	 * always enabled, so we must ensure the interrupt priority
	 * is set to a level lower than the kernel interrupts (for
	 * the assert mechanism to work properly) in case the SysTick
	 * interrupt is accidentally raised.
	 */
	NVIC_SetPriority(SysTick_IRQn, _EXC_IRQ_DEFAULT_PRIO);
#endif /* CPU_CORTEX_M_HAS_SYSTICK && ! CORTEX_M_SYSTICK */
}
#endif

#ifdef CONFIG_BT_STM32WBA
#if (CFG_SCM_SUPPORTED == 0)
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
#endif /* (CFG_SCM_SUPPORTED == 0) */
#endif

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

#ifdef CONFIG_BT_STM32WBA
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
__attribute__((optimize("Ofast"))) static void Exit_Stop_Standby_Mode(void)
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
#endif

static void set_mode_stop(uint8_t substate_id)
{

	LL_PWR_ClearFlag_STOP();
	LL_RCC_ClearResetFlags();

	/* Erratum 2.2.15:
	 * Disabling ICACHE is required before entering stop mode
	 */
	disable_cache();

#ifdef CONFIG_BT_STM32WBA
#if (CFG_SCM_SUPPORTED == 1)
	scm_setwaitstates(LP);
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
	/* Set low power mode to standby */
	LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
	/** ALMA: At this point  we should be ready to save all the CPU context */

	/* Select mode entry : WFE or WFI and enter the CPU selected mode */
	/** CONFIG_ARCH_HAS_CUSTOM_CPU_IDLE allows to have a custom idle if required */
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
	if ((system_startup_done != false) && (UTIL_LPM_GetMode() == UTIL_LPM_OFFMODE)) {
		APP_SYS_BLE_EnterDeepSleep();
		//printk("\n Go2STDBY 3 \n");
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
	/** Here we should redo some of the config made in reset handler, z_prep_c and z_cstart*/
	/**
	 *
	 */
	/** z_prep_c */
	stm32wbax_relocate_vector_table();
#if defined(CONFIG_CPU_HAS_FPU)
	stm32wbax_z_arm_floating_point_init();
#endif
	stm32wbax_z_arm_exc_setup();
	z_arm_fault_init();
	z_arm_cpu_idle_init();
	stm32wbax_z_arm_clear_faults();
#if defined(CONFIG_ARM_MPU)
	z_arm_mpu_init();
	/* Configure static memory map. This will program MPU regions,
	 * to set up access permissions for fixed memory sections, such
	 * as Application Memory or No-Cacheable SRAM area.
	 *
	 * This function is invoked once, upon system initialization.
	 */
	z_arm_configure_static_mpu_regions();
#endif /* CONFIG_ARM_MPU */
#if (CONFIG_DEBUG == 1)
	LL_DBGMCU_EnableDBGStandbyMode();
	LL_DBGMCU_EnableDBGStopMode();
#endif /* CFG_DEBUGGER_LEVEL */

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
	// stm32_clock_control_standby_exit();
	stm32_clock_control_standby_exit();
	//sys_clock_idle_exit();


	/** Cube Code: is_boot_from_standby */
	__HAL_RCC_PWR_CLK_ENABLE();

	LL_PWR_EnableUltraLowPowerMode();

	__HAL_FLASH_SLEEP_POWERDOWN_ENABLE();
	/* Ensure this is a return from Standby, and not a reset */
	if ((LL_PWR_IsActiveFlag_SB() == 1UL) && (READ_REG(RCC->CSR) == 0U)) {
		/* When exit from standby, disable IRQ so that restore and PWR_ExitOffMode are in
		 * critical section */
		__disable_irq();
		/* boot_after_standby = 1 */
		/** Really exiting from stddby */
		PWR_ExitOffMode_std();
	} else {
		/* boot_after_standby = 0 */
		/** Exiting from STOP1 mode */
		Exit_Stop_Standby_Mode();
	}
	/* Put the radio in active state */
	if ((system_startup_done != false) && (UTIL_LPM_GetMode() == UTIL_LPM_OFFMODE)) {
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
#if (CFG_SCM_SUPPORTED == 1)
			if (LL_PWR_IsActiveFlag_STOP() == 1U) {
				scm_setup();
			} else {
				scm_setwaitstates(RUN);
			}
#endif
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
	LL_DBGMCU_EnableDBGStopMode();
	LL_DBGMCU_APB7_GRP1_FreezePeriph(LL_DBGMCU_APB7_GRP1_RTC_STOP);
	LL_DBGMCU_APB7_GRP1_FreezePeriph(LL_DBGMCU_APB7_GRP1_LPTIM1_STOP);
#else
	LL_DBGMCU_DisableDBGStandbyMode();
	LL_DBGMCU_DisableDBGStopMode();
#endif

#ifdef CONFIG_PM_S2RAM
	/* Enable SRAM1, SRAM2 and RADIO retention*/
	LL_PWR_SetSRAM1SBRetention(LL_PWR_SRAM1_SB_FULL_RETENTION);
	LL_PWR_SetSRAM2SBRetention(LL_PWR_SRAM2_SB_FULL_RETENTION);
#ifdef CONFIG_BT_STM32WBA
	/* Enable RADIO retention*/
	LL_PWR_SetRadioSBRetention(LL_PWR_RADIO_SB_FULL_RETENTION); /* Retain sleep timer configuration */
#endif
#endif
	/* Enabling  Ultra Low power mode */
	LL_PWR_EnableUltraLowPowerMode();
	LL_FLASH_EnableSleepPowerDown();
}
