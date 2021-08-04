/*
 * This program runs several alternating user tasks while using SysTick and PendSV for task scheduling.
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

uint8_t current_task = 1;	// Begin with task1 as idle task only runs when all tasks are blocked.
uint32_t g_tick_count = 0;	// Global tick count for task scheduling

// Arbitrary constants to illustrate .rodata in linker map
const uint32_t const1 = 100;
const uint32_t const2 = 0x1234;

typedef struct // Task Control Block
{
	uint32_t psp;
	uint32_t block_count;
	uint8_t current_state;
	void (*task_handler)(void); // Function pointer to task handler

}TCB_t;

TCB_t user_task[MAX_TASKS];

int main(void)
{
	enable_processor_faults();

	initialise_monitor_handles(); // Initialize semihosting for

	init_sched_stack(SCHED_STACK_START);

	init_tasks();

	led_init();

	init_systick(TICK_HZ);

	printf("Init done. Going to process threads.\n");

	switch_to_psp(); // Going to process threads
	user_task[current_task].task_handler(); // Manually start first task (Could use SVC to do this)

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

void SysTick_Handler(void)
{
	g_tick_count++; // Update global tick count

	unblock_tasks(); // Unblock tasks if tick count has been reached

	schedule(); // Set PendSV
}

void schedule(void)
{
	// Set PendSV
	uint32_t *pICSR = ICSR;
	*pICSR |= (1 << 28); // PendSV exception is pending
}

__attribute__ ((naked)) void PendSV_Handler(void)
{
	__asm volatile ("PUSH {LR}");		// Preserve LR

	// Save context of current task SF1 + SF2 + PSP
	// Stack frame 1 is automatically stored on exception entry

	// 1. Get current PSP
	__asm volatile ("MRS R0, PSP");

	// 2. Store SF2 (R4 - R11)
	__asm volatile ("STMDB R0!,{R4-R11}");	// Push registers to task's stack, R0 is updated with PSP

	// 3. Save PSP
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

void init_tasks(void)
{
	// Init TCB
	user_task[0].psp = T0_STACK_START;
	user_task[0].task_handler = task0_handler; // Idle Task

	user_task[1].psp = T1_STACK_START;
	user_task[1].task_handler = task1_handler;

	user_task[2].psp = T2_STACK_START;
	user_task[2].task_handler = task2_handler;

	user_task[3].psp = T3_STACK_START;
	user_task[3].task_handler = task3_handler;

	user_task[4].psp = T4_STACK_START;
	user_task[4].task_handler = task4_handler;

	uint32_t *pPSP;

	for(int i = 0; i < MAX_TASKS; i++)
	{
		// Init TCB
		user_task[i].current_state = TASK_READY;

		// Init Task Stack
		pPSP = (uint32_t*)user_task[i].psp;

		pPSP--; // Decrement first (Stack: full descending)
		*pPSP = INIT_XPSR;

		pPSP--;
		*pPSP = (uint32_t)user_task[i].task_handler; // PC

		pPSP--;
		*pPSP = 0xFFFFFFFD; // LR: return to thread with PSP

		for(int j = 0; j < 13; j++)
		{
			pPSP--;
			*pPSP = 0x00000000; // R0 - R12
		}

		user_task[i].psp = (uint32_t)pPSP;
	}
}

void unblock_tasks(void)
{
	for(int i = 1; i < MAX_TASKS; i++) // Start with Task 1 because Idle is never blocked.
	{
		if(user_task[i].current_state == TASK_BLOCKED)
		{
			if(user_task[i].block_count == g_tick_count)
			{
				user_task[i].current_state = TASK_READY;
			}
		}
	}
}

uint32_t get_psp(void)
{
	return user_task[current_task].psp; // Returned via R0 per AAPCS
}

void save_psp(uint32_t current_psp)
{
	user_task[current_task].psp = current_psp;
}

void select_next_task(void)
{
	uint8_t state = TASK_BLOCKED;

	for(int i = 0; i < MAX_TASKS; i++)
	{
		current_task++;
		current_task %= MAX_TASKS; // MAX -> 0
		state = user_task[current_task].current_state;
		if((current_task) && (state == TASK_READY))
			break; // Non-idle tasks is ready
	}

	if(state == TASK_BLOCKED) // All tasks are blocked
		current_task = 0; // Select idle task
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

void task_delay(uint32_t blocked_ticks)
{
	// This task function manipulates global variables
	// Serialize access to globals by briefly disabling interrupts
	INTERRUPT_DISABLE();

	if(current_task) // Not idle task
	{
		user_task[current_task].block_count = g_tick_count + blocked_ticks;
		user_task[current_task].current_state = TASK_BLOCKED;
		schedule();
	}

	// Enable Interrupts
	INTERRUPT_ENABLE();

}

void task0_handler(void) // Idle Task
{
	while(1);
}

void task1_handler(void)
{
	while(1)
	{
		printf("Task1 is running.\n");
		led_on(LED_GREEN);
		task_delay(1000);
		led_off(LED_GREEN);
		task_delay(1000);
	}
}

void task2_handler(void)
{
	while(1)
	{
		printf("Task2 is running.\n");
		led_on(LED_ORANGE);
		task_delay(500);
		led_off(LED_ORANGE);
		task_delay(500);
	}
}

void task3_handler(void)
{
	while(1)
	{
		printf("Task3 is running.\n");
		led_on(LED_RED);
		task_delay(250);
		led_off(LED_RED);
		task_delay(250);
	}
}

void task4_handler(void)
{
	while(1)
	{
		printf("Task4 is running.\n");
		led_on(LED_BLUE);
		task_delay(125);
		led_off(LED_BLUE);
		task_delay(125);
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
