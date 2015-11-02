#include "CU_TM4C123.h"

#define SYS_CLK 16000UL // 16 Mhz reduced by 1000
#define PWM_FREQ 38UL // 38 Khz reduced by 1000
#define PRESCALE 199UL //Clock prescale value
#define us 1000UL // microsecond reduced by 1000
void static init_GPIO(void);// initilize GPIO pins
void static init_PWM(void);//a PWM used to generate 38 kHz square wave
void static enable_PWM(void);// enable PWM output
void static disable_PWM(void);// disable PWM output
void static init_NVIC(void); // set up the NVIC for interrupts
void static init_timer(void);//a timer used to enable/disable the PWM 
void static set_timer(void);//set the count down value for the timer
void static init_frame(uint16_t addr, uint8_t cmd); //initilize the frame before tx

uint32_t frame;
int frame_index = 32;//point to current value in the frame
uint8_t end_of_frame = 0; //indicate end of frame
uint8_t nextbit; // the next bit needed to be transmited
uint8_t isr_count = 0; //count number of ISR entered
uint8_t state = 0; //state of the FSM

//Define states of the FSM
#define start_a 0
#define start_b 1
#define send0_a 2
#define send0_b 3
#define send1_a 4
#define send1_b 5
#define stop 6
#define end 7
//Define function acting for each state
void static f_start_a(void);
void static f_start_b(void);
void static f_send0_a(void);
void static f_send0_b(void);
void static f_send1_a(void);
void static f_send1_b(void);
void static f_stop(void);
void static f_end(void);
int main(void)
{
	uint16_t addr= 0xF7; /*Address: 0000000011110111*/
	uint8_t cmd_on= 0xC0; /* On command: 11000000 */
	init_GPIO();
	init_PWM();
	init_NVIC();
	init_timer();
	init_frame(addr,cmd_on);
	//trigger the FSM by setting the timer
	set_timer();
	//wait
	while (1) {}
}
/* Init GPIO pins
** PWM -> PE4 -> IR transmitter
**
*/
void static init_GPIO(void)
{
	int dummy;
	//Enable GPIOE
  SYSCTL->RCGCGPIO |= (1UL << 4);
  //Do a dummy read to insert a few cycles after enabling the peripheral.
  dummy = SYSCTL->RCGCGPIO;
	//Sets PE4 for digital out
	GPIOE->DIR |= (1UL << 4);
	GPIOE->DEN |= (1UL << 4);
}
/* PWM generate 38 Khz square wave
** 
**
*/
void static init_PWM()
{
	//Configurate the PWM module 0
	SYSCTL->RCGC0 |= (1UL << 20);
	GPIOE->AFSEL |= (1UL << 4);// Alternate function
	GPIOE->PCTL |= (0x4 << 16);// Select M0PWM4
	SYSCTL->RCC |= (1 << 20);// Use PWM divider
	SYSCTL->RCC |= (0x0 << 17);//Devide Clk by 2
	PWM0->_2_LOAD = SYS_CLK/2/PWM_FREQ - 1; // Set PWM frequency to 38 Khz
	PWM0->_2_CMPA = PWM0->_0_LOAD/2; // toggle at half period to get 50% duty cycle
	PWM0->_2_CTL = 0x0UL;// Immediate update to parameters
	PWM0->_2_GENA = 0x8CUL;//Drive PWM high when counter matches LOAD, drive low when matches CMPA
	PWM0->_2_CTL = 0x1UL;// enabled PWM module 0, generator 2
}
/* Enable the PWM ouput
** 
**
*/
void static enable_PWM()
{
	PWM0->ENABLE |= (1UL << 4);// enable PWM module 0
}
/* Disable the PWM output
**
*/
void static disable_PWM()
{
	PWM0->ENABLE &= ~(1UL << 4);// disable PWM module 0
}
/* Set up the NVIC
** Register TIMER0 interrupt with the NVIC
**
*/
void init_NVIC()
{
	//Set up priority = 1 for TIMER0
	NVIC->IP[19] = 1<<5; //TIMER0 is IRQ 19
	//Clear pending bit for TIMER0
	NVIC->ICPR[0] = 1UL<<19;
	//Enable interrupt for TIMER0 at NVIC 
	NVIC->ISER[0] = 1UL<<19;
}
/* Init the Timer 0A 
** Timer used to enable/disable PWM 
** ... in a certain period
** Timer0A reload value is in us
** Timer0A is one-shot countdown timer
*/
void init_timer()
{
	SYSCTL->RCGCTIMER |= (1UL<<0); //Enable TIMER0
	TIMER0->CTL &= ~(1UL<<0); //disable TIMER0A while configuring
	TIMER0->CFG = 0x4UL; // independent 16-bit timers
	TIMER0->TAMR = 0x1UL; // TIMER0A is one-shot countdown timer
	TIMER0->TAPR = PRESCALE; // set CLK prescale value
	TIMER0->ICR = 1UL; // clear time-out interrupt
	TIMER0->CTL |= (1UL << 9); // TBSTALL
	
}
/* Set up count_down value for timer
** and start count down
** count down value in is us
**
*/
void set_timer()
{
	TIMER0->CTL &= ~(1UL<<0); //disable TIMER0A while configuring
	TIMER0->TAILR = 44; // 44 = 562.5 us
	TIMER0->IMR |= 1UL; //arm timer interrupt
	TIMER0->CTL |= (1UL<<0);// enable TIMER0A
}
/* Init the frame before Tx
** concatenate the addr and command and inverse command
** to create a 32 bit frame
**
*/
void init_frame(uint16_t addr, uint8_t command)
{
	uint8_t command_inverse;
	
	frame_index = 32;
	command_inverse = ~command;
	frame = (addr<<16|command<<8|command_inverse);
}
/* TIMER0A interrupt handler
**
**
*/
void TIMER0A_Handler(void)
{
	//clear interrupt at GPTM
	TIMER0->ICR = 1UL;
	//clear pending bit in NVIC
	NVIC->ICPR[0] = 1UL<<19;
	
	//Enter state machine
	switch (state){
		case start_a: /* State start_a */
			f_start_a();
			if (isr_count == 16) 
			{
				isr_count = 0;
				state = start_b;
			}
			else state = start_a; // this state repeat in 16 ISR counts
			break;
		case start_b: /* State start_b */
			f_start_b();
			if (isr_count == 8)
			{
				isr_count = 0;
				if (nextbit == 0) state = send0_a;
				else state = send1_a;
			}		
			else state = start_b;// this state repreat in 8 ISR counts
			break;
		case send0_a: /* State send0_a */
			f_send0_a();
			state = send0_b;
			break;
		case send0_b: /* State send0_b */
			f_send0_b();
			if (end_of_frame) state = stop;
			else if (nextbit == 0) state = send0_a;
			else state = send1_a;
			break;
		case send1_a: /* State send1_a */
			f_send1_a();
			state = send1_b;
			break;
		case send1_b: /* State send1_b */
			f_send1_b();
			if (isr_count == 3)
			{
				isr_count = 0;
				if (end_of_frame) state = stop;
				else if (nextbit == 0) state = send0_a;
				else state = send1_a;
			}
			else state = send1_b;// this state repeat in 3 ISR counts
			break;
		case stop: /* State stop */
			f_stop();
			state = end;
			break;
		case end: /* State end */
			f_end();
			break;
		default:
			state = end;
			break;
	}
	set_timer();
}
/* Start_a state
** Enable PWM
** Increase ISR count
**
*/
void static f_start_a()
{
	//Turn on PWM
	enable_PWM();
	//Increase ISR count
	isr_count++;
}
/* Start_b state
** Disable PWM
** Increase ISR count
**
*/
void static f_start_b()
{
	//Turn off PWM
	disable_PWM();
	//Increase ISR count
	isr_count++;
	//Decrease frame_index if permit
	if (isr_count == 8) frame_index--;
	//determine the bit to trasmit
	if (frame_index < 0) end_of_frame = 1;
	else nextbit = (frame & (1UL<<frame_index))>>frame_index;
}
/* Send0_a state
** Enable PWM
**
*/
void static f_send0_a()
{
	//Turn on PWM
	enable_PWM();
}
/* Send0_b state
** Disable PWM
** Increase frame_index
**
*/
void static f_send0_b()
{
	//Turn off PWM
	disable_PWM();
	//Decrease frame_index
	frame_index--;
	//Determine the bit to trasmit
	if (frame_index < 0) end_of_frame = 1;
	else nextbit = (frame & (1UL<<frame_index))>>frame_index;
}
/* send1_a state
** Enable PWM
**
*/
void static f_send1_a()
{
	//Turn on PWM
	enable_PWM();
}
/* send1_b state
** Disable PWM
** Increase ISR count
** Increase frame_index if ISR count = 3
**
*/
void static f_send1_b()
{
	//Turn off PWM
	disable_PWM();
	//Increase ISR count
	isr_count++;
	// Decrease frame_index if permit
	if (isr_count == 3) frame_index--;
	//determine the bit to trasmit
	if (frame_index < 0) end_of_frame = 1;
	else nextbit = (frame & (1UL<<frame_index))>>frame_index;
}
/* stop state
** Enable PWM
**
*/
void static f_stop()
{
	//Turn on PWM
	enable_PWM();
}
/* end state
** Disable PWM
**
*/
void static f_end()
{
	//Turn off PWM
	disable_PWM();
}
