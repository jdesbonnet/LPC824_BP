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


#define TICKRATE_HZ 10

//#define UART_BAUD_RATE 115200
#define UART_BAUD_RATE 230400
//#define UART_BAUD_RATE 460800

#define EOL "\r\n"

// Define function to pin mapping. Pin numbers here refer to PIO0_n
// and is not the same as a package pin number.
// Use PIO0_0 and PIO0_4 for UART RXD, TXD (same as ISP)
#define PIN_UART_RXD 0
#define PIN_UART_TXD 4

#define PIN_SCL 10
#define PIN_SDA 11
#define PIN_START 14

// TODO: insert other include files here
#include "printf.h"

static volatile int sda_pin_state=1;
static volatile int scl_pin_state=1;
static char i2c_buf[1024];
static volatile int i2c_buf_ptr=0;
static uint32_t last_clock = 0;

// To facilitate tfp_printf()
void myputc (void *p, char c) {
	Chip_UART_SendBlocking(LPC_USART0, &c, 1);
}

void setup_pin_for_interrupt (int interrupt_pin, int interrupt_channel, int riseOrFallEdge) {

	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, interrupt_pin);
	Chip_IOCON_PinSetMode(LPC_IOCON,interrupt_pin,PIN_MODE_PULLUP);

	/* Configure interrupt channel for the GPIO pin in SysCon block */
	Chip_SYSCTL_SetPinInterrupt(interrupt_channel, interrupt_pin);

	/* Configure GPIO pin as input pin */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, interrupt_pin);

	/* Configure channel 7 interrupt as edge sensitive and falling edge interrupt */
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(interrupt_channel));

	if (riseOrFallEdge == 0) {
		Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(interrupt_channel));
	} else {
		Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH(interrupt_channel));
	}
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
 * Microsecond delay.
 */
void delay_us(uint32_t delay) {

	// Clock is 30MHz
	delay *= 30;

	// TODO: can be optimized by triggering interrupt
	// then wait for interrupt instead of tight loop.
	uint32_t start = LPC_SCT->COUNT_U;
	while ( (LPC_SCT->COUNT_U - start) < delay) ;
}

/**
 * Assert START button by pulling low and going into input (high Z)
 * state otherwise. Doing this due the 4.4V logic rail: not sure
 * if exposing a output high (normally at Vdd or 2.2V) to 4.4V is
 * good idea.
 * @state  1 = press button, 0 = normal button released
 */
void press_start_button (int state) {
	if (state == 1) {
		// Pin to 0V and switch to output
		Chip_GPIO_SetPinState(LPC_GPIO_PORT,0,PIN_START,0);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT,0,PIN_START);
	} else {
		// Pin to input (high Z)
		Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT,0,PIN_START);
	}
}

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{

}


/**
 * @brief	SCL rising edge
 * @return	Nothing
 */
void PININT4_IRQHandler(void)
{
	scl_pin_state = 1;

	// Does not work for some reason?
	i2c_buf[i2c_buf_ptr++] = sda_pin_state ? '1':'0';
	//i2c_buf[i2c_buf_ptr++] = Chip_GPIO_GetPinState(LPC_GPIO_PORT,0,PIN_SDA) ? '1':'0';

	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH4);
}

/**
 * @brief	SCL falling edge
 * @return	Nothing
 */
void PININT5_IRQHandler(void)
{
	last_clock = LPC_SCT->COUNT_U;
	scl_pin_state = 0;
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH5);
}

/**
 * @brief	SDA rising edge
 * @return	Nothing
 */
void PININT6_IRQHandler(void)
{
	sda_pin_state = 1;

	// Stop condition
	if (scl_pin_state == 1) {
	//if (Chip_GPIO_GetPinState(LPC_GPIO_PORT,0,PIN_SCL) == 1) {
		i2c_buf[i2c_buf_ptr++] = 'P';
	}
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH6);
}

/**
 * @brief	SDA falling edge
 * @return	Nothing
 */
void PININT7_IRQHandler(void)
{
	sda_pin_state = 0;

	// Start condition
	//if (Chip_GPIO_GetPinState(LPC_GPIO_PORT,0,PIN_SCL)  == 1) {
	if (scl_pin_state  == 1) {

		i2c_buf[i2c_buf_ptr++] = 'S';
	}
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH7);
}



int main(void) {

#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif


	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);



    //
    // Initialize GPIO
    //
	Chip_GPIO_Init(LPC_GPIO_PORT);


	// Initialize UART
	setup_uart();
	init_printf(NULL,myputc);

	// Configure I2C lines to trigger interrupts on both
	// rising and falling edges.
	setup_pin_for_interrupt(PIN_SCL, 4, 1);
	setup_pin_for_interrupt(PIN_SCL, 5, 0);
	setup_pin_for_interrupt(PIN_SDA, 6, 1);
	setup_pin_for_interrupt(PIN_SDA, 7, 0);


	// Enable interrupts in the NVIC
	NVIC_EnableIRQ(PININT4_IRQn);
	NVIC_EnableIRQ(PININT5_IRQn);
	NVIC_EnableIRQ(PININT6_IRQn);
	NVIC_EnableIRQ(PININT7_IRQn);




	// Use SCT for hi-res timer.
	setup_sct_for_timer();

	// Enable watchdog timer
	//setup_wdt();

	// Initialize GPIO
	Chip_GPIO_Init(LPC_GPIO_PORT);
	uint32_t clock_hz = Chip_Clock_GetSystemClockRate();

	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT,0,PIN_START);


    tfp_printf("monitoring I2C bus...\r\n");

    scl_pin_state = Chip_GPIO_GetPinState(LPC_GPIO_PORT,0,PIN_SCL);
    sda_pin_state = Chip_GPIO_GetPinState(LPC_GPIO_PORT,0,PIN_SDA);

	int i = 0;

	// Bit in current byte counter, when 8 we have a byte, 9th bit is ack.
	int bit_count = 0;

	// Number of bytes in transaction. Reset on STOP (not START).
	int byte_count = 0;

	// Bit shift each incoming bit here
	uint8_t current_byte = 0;

	// Frame data (including addr). Reset on STOP (not START).
	uint8_t i2c_bytes[8];

	// Bytes written to EEPROM for BP reading
	uint8_t bp_record[12];
	int bp_record_ptr = 0;

	while (1) {
		//__WFI();

		// UART simple commands. Single char. "0".."2" set sample speeds 50sps,100sps,200sps.2
		if (LPC_USART0->STAT & 1) {
	    		char c = LPC_USART0->RXDATA;
	    		switch (c) {
	    		case 'R':

	    			tfp_printf("measuring BP...\r\n");
	    			// Press for 1 sec to wake, release for 1 sec
	    			// and press again to start measurement
	    			press_start_button(1);
	    			delay_us(1000000);
	    			press_start_button(0);
	    			delay_us(1000000);
	    			press_start_button(1);
	    			delay_us(1000000);
	    			press_start_button(0);
	    			delay_us(1000000);
	    		}
		}

		// One of two formats:
		// read operation: S A0 [ack] addr [ack] S A1 [nak] data-from-eeprom P
		// write operation S A0 [ack] addr [ack] [data-to-write] P
		if ((i2c_buf_ptr > 0) && ((LPC_SCT->COUNT_U - last_clock) > 30000000)) {
			for (i = 0; i < i2c_buf_ptr; i++) {
				char c = i2c_buf[i];
				tfp_printf("%c", c);

				if (c == 'S') {
					bit_count = 0;
				}

				if (c == '0' || c == '1') {
					current_byte <<= 1;
					current_byte |= (c == '1');
					bit_count++;
				}
				if (bit_count == 8) {
					i2c_bytes[byte_count] = current_byte;
					tfp_printf(" [%x] ", current_byte);
				}
				if (bit_count == 9) {
					bit_count = 0;
					byte_count++;
					tfp_printf(" ");
				}
				if (c == 'P') {
					tfp_printf(" bytes=%d\r\n", byte_count);
					if (byte_count == 3) {
						bp_record[bp_record_ptr++] = i2c_bytes[2];
					}
					byte_count = 0;
				}
			}
			i2c_buf_ptr = 0;

			if (bp_record_ptr > 0) {
				tfp_printf("r=%d BP:", bp_record_ptr);
				for (i = 0; i < bp_record_ptr; i++) {
					tfp_printf(" %x", bp_record[i]);
				}
				tfp_printf("\r\n");

				int systolic = (bp_record[5] >> 4) * 100;
				systolic += (bp_record[6] >> 4) * 10;
				systolic += (bp_record[6] & 0xf);

				int diastolic = (bp_record[5] & 0xf) * 100;
				diastolic += (bp_record[7] >> 4) * 10;
				diastolic += (bp_record[7] & 0xf);

				int heart_rate = bp_record[8];

				tfp_printf("sys=%d dia=%d hr=%d\r\n", systolic, diastolic,
						heart_rate);
			}

			bp_record_ptr = 0;

		}
	}
	return 0;
}
