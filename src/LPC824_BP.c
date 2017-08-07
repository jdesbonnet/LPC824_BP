/*
===============================================================================
 Name        : LPC824_BP.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>


#define TICKRATE_HZ 100

//#define UART_BAUD_RATE 115200
#define UART_BAUD_RATE 230400
//#define UART_BAUD_RATE 460800

#define EOL "\r\n"

// Define function to pin mapping. Pin numbers here refer to PIO0_n
// and is not the same as a package pin number.
// Use PIO0_0 and PIO0_4 for UART RXD, TXD (same as ISP)
#define PIN_UART_RXD 0
#define PIN_UART_TXD 4

#include "printf.h"
#include "abpm.h"

volatile uint32_t systick_counter = 0;

// To facilitate tfp_printf()
void myputc (void *p, char c) {
	Chip_UART_SendBlocking(LPC_USART0, &c, 1);
}

/**
 * Microsecond delay.
 */
void delay_ms(uint32_t delay_ms) {

	// Ticks are 100Hz, divide by 10 for ms to ticks
	int ticks = (delay_ms * TICKRATE_HZ) / 1000;

	// TODO: can be optimized by triggering interrupt
	// then wait for interrupt instead of tight loop.
	uint32_t start = systick_counter;
	while ( (systick_counter - start) < ticks) ;
}

void setup_sct_for_timer (void) {

	Chip_SCT_Init(LPC_SCT);

	/* Stop the SCT before configuration */
	Chip_SCTPWM_Stop(LPC_SCT);

	// Match/capture mode register. (ref UM10800 section 16.6.11, Table 232, page 273)
	// Determines if match/capture operate as match or capture. Want all match.
	LPC_SCT->REGMODE_U = 0;

	// Set SCT Counter to count 32-bits and reset to 0 after reaching MATCH0
	Chip_SCT_Config(LPC_SCT, SCT_CONFIG_32BIT_COUNTER );

	Chip_SCT_ClearControl(LPC_SCT, SCT_CTRL_HALT_L | SCT_CTRL_HALT_H);
}



void setup_uart () {
	// Assign pins: use same assignment as serial bootloader
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, PIN_UART_TXD);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, PIN_UART_RXD);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_UART_Init(LPC_USART0);
	Chip_UART_ConfigData(LPC_USART0,
			UART_CFG_DATALEN_8
			| UART_CFG_PARITY_NONE
			| UART_CFG_STOPLEN_1);

	Chip_Clock_SetUSARTNBaseClockRate((UART_BAUD_RATE * 16), true);
	Chip_UART_SetBaud(LPC_USART0, UART_BAUD_RATE);
	Chip_UART_TXEnable(LPC_USART0);
	Chip_UART_Enable(LPC_USART0);
}



/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	systick_counter++;
}


int main(void) {

    //
    // Initialize GPIO
    //
	Chip_GPIO_Init(LPC_GPIO_PORT);


	// Initialize UART
	setup_uart();
	init_printf(NULL,myputc);

	// Use SCT for hi-res timer.
	setup_sct_for_timer();


	// Initialize GPIO
	Chip_GPIO_Init(LPC_GPIO_PORT);
	uint32_t clock_hz = Chip_Clock_GetSystemClockRate();

	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);
	SysTick_Config(clock_hz/TICKRATE_HZ);


    tfp_printf("init abpm...\r\n");
    abpm_init();

	while (1) {

		__WFI();

		// UART simple commands.
		if (LPC_USART0->STAT & 1) {
			char c = LPC_USART0->RXDATA;
			switch (c) {
			case 'M': {
				tfp_printf("measuring BP...\r\n");
				abpm_measure();
				break;
			}
			case 'S': {
				tfp_printf("stop BP...\r\n");
				abpm_stop();
				break;
			}
			case 'N': {
				tfp_printf("bus snoop...\r\n");
				abpm_bus_snoop();
				break;
			}
			}
		}

	}

	return 0;
}
