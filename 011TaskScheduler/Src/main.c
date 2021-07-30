/*
 * This program runs several alternating user tasks while using SysTick handler as a task scheduler.
 *
 * Written for STM32F407G on Eclipse based STM32CubeIDE
 * Dev Board: STM32F407G-DISC1
 *
 */

#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "led.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

uint32_t psp_of_task[MAX_TASKS] = {T0_STACK_START, T1_STACK_START, T2_STACK_START, T3_STACK_START};
uint32_t handler_task[MAX_TASKS]; // TODO: Replace with struct

uint8_t current_task = 0; // Begin with task0

int main(void)
{
	enable_processor_faults();

	init_sched_stack(SCHED_STACK_START);

	handler_task[0] = (uint32_t)task0_handler; // Green LED  - PD12
	handler_task[1] = (uint32_t)task1_handler; // Orange LED - PD13
	handler_task[2] = (uint32_t)task2_handler; // Red LED    - PD14
	handler_task[3] = (uint32_t)task3_handler; // Blue LED   - PD12
	init_tasks_stack();

	led_init();

	init_systick(TICK_HZ);

	switch_to_psp(); // Going to process threads
	task0_handler(); // Manually start first task (Could use SVC to do this)

	/* Loop forever */
	for(;;); // Unreachable because tasks never return
}

void enable_processor_faults(void)
{
	// System Handler Control and State Register
	uint32_t *pSHCSR = SHCSR;

	// Enable configurable Exceptions
	// Set bits: [18] USGFAULTENA, [17] BUSFAULTENA, [16] MEMFAULTENA
	*pSHCSR |= (0x7 << 16); // 0b0111 << 16
}

void init_systick(uint32_t tick_hz)
{
	uint32_t *pSRVR = SYST_RVR;
	uint32_t *pSCSR = SYST_CSR;
	uint32_t count_value = (SYSTICK_CLK/tick_hz)-1;

	// Clear RVR and load count
	*pSRVR &= ~(0x00FFFFFF);
	*pSRVR |= count_value;

	// Configure SysTick
	*pSCSR |= (1 << 1); // Enable exception request
	*pSCSR |= (1 << 2); // Use processor clock

	// Enable SysTick
	*pSCSR |= (1 << 0);

}

__attribute__ ((naked)) void SysTick_Handler(void)
{
	__asm volatile ("PUSH {LR}");		// Preserve LR

	// Save context of current task SF1 + SF2 + PSP
	// Stack frame 1 is automatically stored on exception entry

	// 1. Get current PSP
	__asm volatile ("MRS R0, PSP");

	// 2. Store SF2 (R4 - R11)
	__asm volatile ("STMDB R0!,{R4-R11}");	// Push registers to task's stack, R0 is updated with PSP

	// 3. Save PSP
//	__asm volatile ("MOV %0,R0": "=r"(psp_of_task[current_task]) ::);
	__asm volatile ("BL save_psp");			// Save task's PSP from R0

	// Retrieve context of next task

	__asm volatile ("BL select_next_task");	// Advance round-robin

	// 1. Retrieve previous PSP
	__asm volatile ("BL get_psp");			// Save task's PSP to R0

	// 2. Retrieve SF2 (R4 - R11)
	__asm volatile ("LDMIA R0!,{R4-R11}");	// Pop registers to task's stack, R0 is updated with PSP

	// 3. Update PSP
	__asm volatile ("MSR PSP, R0");

	// SF1 is automatically fetched by exception exit

	__asm volatile ("POP {LR}");			// Restore LR
	__asm volatile ("BX LR");				// Return from naked function

}

__attribute__ ((naked)) void init_sched_stack(uint32_t sched_stack_top)
{
	__asm("MSR MSP, %0"::"r" (sched_stack_top));
	__asm("BX LR");
}

void init_tasks_stack(void)
{
	uint32_t *pPSP;

	for(int i = 0; i < MAX_TASKS; i++)
	{
		pPSP = (uint32_t*)psp_of_task[i];

		pPSP--; // Decrement first (Stack: full descending)
		*pPSP = INIT_XPSR;

		pPSP--;
		*pPSP = handler_task[i]; // PC

		pPSP--;
		*pPSP = 0xFFFFFFFD; // LR: return to thread with PSP

		for(int j = 0; j < 13; j++)
		{
			pPSP--;
			*pPSP = 0x00000000; // R0 - R12
		}

		psp_of_task[i] = (uint32_t)pPSP;
	}
}

uint32_t get_psp(void)
{
	return psp_of_task[current_task]; // Returned via R0 per AAPCS
}

void save_psp(uint32_t current_psp)
{
	psp_of_task[current_task] = current_psp;
}

void select_next_task(void)
{
	current_task++;
	current_task %= MAX_TASKS; // MAX -> 0
}

__attribute__ ((naked)) void switch_to_psp(void)
{
	__asm volatile ("PUSH {LR}");		// Preserve LR

	// Initialize PSP with task's stack top
	__asm volatile ("BL get_psp");		// Get current task's PSP
	__asm volatile ("MSR PSP, R0");		// Init PSP (R0 holds return value of get_psp)

	__asm volatile ("POP {LR}");		// Restore LR

	// Switch SP to PSP
	__asm volatile ("MRS R0, CONTROL");
	__asm volatile ("ORR R0, #0x2");	// SPSEL = 1 (PSP)
	__asm volatile ("MSR CONTROL, R0");

	__asm volatile ("BX LR");			// Return from naked function

}

void task0_handler(void)
{
	while(1)
	{
		led_on(LED_GREEN);
		delay(DELAY_COUNT_1S);
		led_off(LED_GREEN);
		delay(DELAY_COUNT_1S);
	}
}

void task1_handler(void)
{
	while(1)
	{
		led_on(LED_ORANGE);
		delay(DELAY_COUNT_500MS);
		led_off(LED_ORANGE);
		delay(DELAY_COUNT_500MS);
	}
}

void task2_handler(void)
{
	while(1)
	{
		led_on(LED_RED);
		delay(DELAY_COUNT_250MS);
		led_off(LED_RED);
		delay(DELAY_COUNT_250MS);
	}
}

void task3_handler(void)
{
	while(1)
	{
		led_on(LED_BLUE);
		delay(DELAY_COUNT_125MS);
		led_off(LED_BLUE);
		delay(DELAY_COUNT_125MS);
	}
}

void HardFault_Handler(void)
{
	printf("HardFault Exception\n");
	while(1);
}

void UsageFault_Handler_c(void)
{
	printf("UsageFault Exception\n");
	while(1);
}

void BusFault_Handler(void)
{
	printf("BusFault Exception\n");
	while(1);
}

void MemManage_Handler(void)
{
	printf("MemFault Exception\n");
	while(1);
}
