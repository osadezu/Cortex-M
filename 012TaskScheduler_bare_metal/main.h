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
#define T4_STACK_START		((SRAM_END) - (4 * (TASK_STACK_SIZE)))
#define SCHED_STACK_START	((SRAM_END) - (5 * (TASK_STACK_SIZE)))
#define SCHED_STACK_END		((SCHED_STACK_START) - (SCHED_STACK_SIZE))

#define MAX_TASKS			5

#define TASK_READY			0x00
#define TASK_BLOCKED		0xFF

#define TICK_HZ				1000U
#define SYSTICK_CLK			16000000U // HSI: 16 MHz
#define SYST_RVR			((uint32_t*)0xE000E014U)
#define SYST_CSR			((uint32_t*)0xE000E010U)

#define ICSR				((uint32_t*)0xE000ED04U)
#define SHCSR				((uint32_t*)0xE000ED24U)

#define INIT_XPSR			0x01000000U // Keep T-bit

#define INTERRUPT_DISABLE()	__asm volatile ("CPSID i")
#define INTERRUPT_ENABLE()	__asm volatile ("CPSIE i")

// Function prototypes

extern void initialise_monitor_handles(void);

void task0_handler(void); // Idle Task
void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void task4_handler(void);
void enable_processor_faults(void);
void init_systick(uint32_t);
void schedule(void);
__attribute__ ((naked)) void init_sched_stack(uint32_t);
void init_tasks(void);
void unblock_tasks(void);
uint32_t get_psp_value(void);
__attribute__ ((naked)) void switch_to_psp(void);
void task_delay(uint32_t);

#endif /* MAIN_H_ */
