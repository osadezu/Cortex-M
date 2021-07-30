/*
 * main.h
 *
 *  Created on: Jul 29, 2021
 *      Author: oscar
 */

#ifndef MAIN_H_
#define MAIN_H_


// Stack areas calculation
#define TASK_STACK_SIZE		1024U // 1 KiB
#define SCHED_STACK_SIZE	1024U // 1 KiB

#define SRAM_START			0x20000000U
#define SRAM_SIZE			(128U * 1024U)
#define SRAM_END			((SRAM_START) + (SRAM_SIZE))

#define T0_STACK_START		SRAM_END
#define T1_STACK_START		((SRAM_END) - (1 * (TASK_STACK_SIZE)))
#define T2_STACK_START		((SRAM_END) - (2 * (TASK_STACK_SIZE)))
#define T3_STACK_START		((SRAM_END) - (3 * (TASK_STACK_SIZE)))
#define SCHED_STACK_START	((SRAM_END) - (4 * (TASK_STACK_SIZE)))
#define SCHED_STACK_END		((SRAM_END) - (4 * (TASK_STACK_SIZE)) - (SCHED_STACK_SIZE))

#define MAX_TASKS			4

#define TICK_HZ				1000U
#define SYSTICK_CLK			16000000U // HSI: 16 MHz
#define SYST_RVR			((uint32_t*)0xE000E014U)
#define SYST_CSR			((uint32_t*)0xE000E010U)

#define SHCSR				((uint32_t*)0xE000ED24U)

#define INIT_XPSR			0x01000000U // Keep T-bit

// Function prototypes
void task0_handler(void);
void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void enable_processor_faults(void);
void init_systick(uint32_t);
__attribute__ ((naked)) void init_sched_stack(uint32_t);
void init_tasks_stack(void);
uint32_t get_psp_value(void);
__attribute__ ((naked)) void switch_to_psp(void);

#endif /* MAIN_H_ */
