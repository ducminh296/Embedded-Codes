#include "CU_TM4C123.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BUFFER_SIZE  10//8 bytes in each buffer
#define POOL_SIZE 20 // Pool has 8 buffers
#define PRESCALE 255UL; //Clock prescale value
/* Define a buffer and buffer pool */
typedef char buffer_t[BUFFER_SIZE]; // define buffer
typedef struct{
	buffer_t pool[POOL_SIZE]; // a pool of buffers
	uint8_t head;
	uint8_t tail;
} queue_t;

/* Init functions */
void static UART_Init(void);//Initialize the UART
void static Timer_Init(void);//Initialize the Timer
void static NVIC_Init(void);//Initialize the NVIC

/* Methods to deal with the buffer queue*/
void queue_Init(queue_t* queue);//initialize the queue before using
int is_queue_full(queue_t* queue);//indicate if the queue is full
int is_queue_full(queue_t* queue);//indicate if the queue is empty
buffer_t *get_buffer(queue_t* queue); // return a pointer to the buffer from the queue
int release_buffer(queue_t *queue, buffer_t* out_buffer);//release a buffer back to queue

/* Methods to deal with the UART */
void Tx_message(char *msg_buffer, uint32_t msg_size);//will accept the msg for Tx'ing
void Rx_message(buffer_t *buffer);//return a pointer to an Rx msg
void start_UART();// Kick-start the UART TX at the beginning
	
/* Methods to deal with the Timer */
void start_timer(void);


/* Methods to deal with the Clock */
void set_Clock(char *val);//set the clock to value format THH:MM
void Create_Clock_Display(char *clock_display);//create a clock display
void Increase_Clock(void); //Increase the clock by 1 second
void faster_Clock(void);//faster the clock by 2
void slower_Clock(void);//slower the clock by 2
void pause_Clock(void);//pause the clock
void continue_Clock(void);//continue the clock

/* Misc */
void static stop_execution(void);//stop execution right away
void UART_OutChar(unsigned char data);
/* ------------ GLOBAL Variables ------------- */
queue_t tx_queue, rx_queue; // define a tx_queue, rx_queue
uint8_t hours, mins, secs; //hold the clock variables
uint8_t busy_input = 0; //indicate if user is inputting
/* ------------ queue_init() ------------------
** Initialize the message queue
**
*/
void queue_Init(queue_t* queue)
{
	queue -> head  = 0;
	queue -> tail  = 0;
}
/* ------------ is_queue_full() -----------------
** Indicate if the queue is full
** Return 1 if full, 0 otherwise
*/
int is_queue_full(queue_t* queue)
{
	if ((queue -> tail + 1)%POOL_SIZE == queue -> head)
		return 1;
	return 0;
}
/* ------------ is_queue_empty() -----------------
** Indicate if the queue is empty
** Return 1 if empty, 0 otherwise
*/
int is_queue_empty(queue_t* queue)
{
	if (queue -> tail == queue -> head)
		return 1;
	return 0;
}
/* ------------ get_buffer() --------------------
** This method return an available buffer from the queue
** Input: pointer to the queue
** Output: Pointer to a buffer allocated from the queue
**
*/
buffer_t *get_buffer(queue_t* queue)
{
	buffer_t* buff;
	if (!is_queue_full(queue)) //if queue is not full yet
	{
		buff = &(queue->pool[queue->tail]);
		queue->tail = (queue->tail + 1)%POOL_SIZE;
		return buff;
	}
	else
	{
		stop_execution();
		return NULL;
	}
}
/* ------------ release_buffer() -------------------
** This method release a buffer back to the queue
** Input: pointer to the queue
** Output: Return 1 on success, 0 on failure
**
*/
int release_buffer(queue_t *queue, buffer_t* out_buffer)
{
	buffer_t *buffer= NULL;
	if (!is_queue_empty(queue)) //if queue is not empty (just in case)
	{
		buffer = &(queue -> pool[queue ->head]);
		queue ->head  = (queue ->head  + 1)%POOL_SIZE;
		memcpy(out_buffer,buffer,BUFFER_SIZE);
		//Clean the buffer
		memset(buffer,0x00,BUFFER_SIZE);
		return 1;
	}
	return 0;
}
/* ------------ Tx_message -------------------------
** Transmit the msg_buffer by putting it to tx_queue
**
** If the msg_buffer is higher than BUFFER_SIZE
** The msg_buffer will be truncated and put into several 
** ... buffers
** Input: pointer to msg_buffer, size of msg buffer
** Output: None
*/
void Tx_message(char *msg_buffer, uint32_t msg_size)
{
	buffer_t *buff;
	uint32_t i=0;
	uint8_t n;
	while (i<msg_size)
	{
		__set_PRIMASK(1);//disable all interrupts
		buff = get_buffer(&tx_queue); //get a buffer from the tx_queue
		__set_PRIMASK(0);//enable intterrupts
		//copy msg to the buffer
		n = (msg_size-i>BUFFER_SIZE)?(BUFFER_SIZE-1):msg_size-i;
		strncpy((char*)buff,&msg_buffer[i],n);//truncate the msg_buffer
		(*buff)[n+1]='\0';
		i += n;
		//Unmask Tx interrupt at UART
		UART0 -> IM |= (UART_IM_TXIM);
	}
}
/* ------------ Rx_message --------------------
** Return a buffer from the rx_queue
**
** Input: None
** Output: pointer to buffer containing msg
*/
void Rx_message(buffer_t *buffer)
{
	__set_PRIMASK(1);//disable all interrupts
	release_buffer(&rx_queue,buffer); //release a buffer from the rx_queue
	__set_PRIMASK(0);//enable intterrupts
}
/* ------------ UART_Init ---------------------
** Initialize the UART for 115,200 baud rate (assuming 16 MHz UART clock),
** 8 bit word length, no parity bits, one stop bit, FIFOs enabled
** Input: none
** Output: none 
*/
void UART_Init(void){
  SYSCTL -> RCGCUART |= SYSCTL_RCGCUART_R0; // activate UART0
  SYSCTL -> RCGCGPIO |= SYSCTL_RCGCGPIO_R0; // activate port A
  UART0 -> CTL &= ~UART_CTL_UARTEN;        // disable UART 
	//UART0 -> CTL |= UART_CTL_EOT; //End of Tranmision
  UART0 -> IBRD = 8;                       // IBRD = int(16,000,000 / (16 * 115,200)) = int(8.680555) = 8
  UART0 ->FBRD  = 44;                      // FBRD = int(0.680555 * 64 + 0.5) = 44
                                           // 8 bit word length (no parity bits, one stop bit, enable FIFOs)
  UART0 -> LCRH |= (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  GPIOA -> AMSEL &= ~0x03;                // disable analog functionality on PA
  GPIOA -> AFSEL |= 0x03;                 // enable alt funct on PA1-0
  GPIOA -> DEN |= 0x03;                    // enable digital I/O on PA1-0
                                           // configure PA1-0 as UART   (just in case)
  GPIOA -> PCTL = (GPIOA -> PCTL & 0xFFFFFF00)+0x00000011;
	UART0 -> ICR |= (UART_ICR_TXIC | UART_ICR_RTIC); // clear interrupt
	UART0 -> IM |= (UART_IM_RTIM|UART_IM_TXIM) ; //arm interrupt for RX and TX
	UART0 -> CTL |= UART_CTL_UARTEN;         // enable UART
}

/* ------------- Start UART ---------------------
** The UART TX need to be kick-start by filling 
** ... up the FIFO
**
 */
void start_UART()
{
  char s[]="00:00:00\r\n";
	int i;
	for (i=0;i<sizeof(s);i++)
		UART0->DR = s[i];
}
/* ------------ NVIC_Init -----------
** Initialize the NVIC 
** Register the interrupt from the UART0 TX and UART0 RX
** TIMER0 ISQ = 19; UART0 ISR = 5
** Input: None
** Output: None
*/
void NVIC_Init(void)
{
	/* TIMER0A */
	//Set up priority = 1 for TIMER0A
	NVIC -> IP[19] = 1<<5; //TIMER0A is IRQ 19
	//Clear pending bit for TIMER0A
	NVIC -> ICPR[0] = 1UL<<19;
	//Enable interrupt for TIMER0A at NVIC 
	NVIC -> ISER[0] = 1UL<<19;
	/* TIMER0B */
	//Set up priority = 3 for TIMER0B
	NVIC -> IP[20] = 3<<5; //TIMER0B is IRQ 20
	//Clear pending bit for TIMER0B
	NVIC -> ICPR[0] = 1UL<<20;
	//Enable interrupt for TIMER0B at NVIC 
	NVIC -> ISER[0] = 1UL<<20;
	/* UART0 */
	//Set up priority = 2 for UART0
	NVIC -> IP[5] = 2<<5; //UART0 is IRQ 5
	//Clear pending bit for UART0
	NVIC -> ICPR[0] = 1UL<<5;
	//Enable interrupt for UART0 at NVIC
	NVIC -> ISER[0] = 1UL<<5;
	/* Push Button */
	/* Init for PF4 (SW1) and PF0 (SW2) */
	NVIC->IP[30] = (4 << 5);	//IRQ#30 for GPIOF, see ds pg104
	NVIC->ICPR[0] = (1UL << 30);	//Clear pending bit to be safe
	NVIC->ISER[0] = (1UL << 30);	//Enable interrup at NVIC
}
/* ------------- GPIO_Init --------------------
** Initialize the GPIO used in this lab
**
*/
void GPIO_Init(void)
{
	uint32_t ulLoop;
  // Set up to use LED
	// Enable GPIOF port (used for the on-board LED).
	SYSCTL->RCGCGPIO |= (1UL << 5);
  // Do a dummy read to insert a few cycles after enabling the peripheral.
  ulLoop = SYSCTL->RCGCGPIO;	
  // Enable the GPIO pin for the LED (PF3).
	GPIOF->DIR |= 0x08;       // Set the direction as output
  GPIOF->DEN |= 0x08;       // Enable the GPIO pin for digital function
	// Init the Push Button
	/* Setting up PF4 (SW1) */
	GPIOF->DIR &= ~(1UL << 4);	// Set PF4 as input
	GPIOF->DEN |= (1UL << 4);		// Digital enable PF4
	GPIOF->IM &= ~(1UL << 4);		// Mask PF4 for now while configuring
	GPIOF->IS &= ~(1UL << 4);		// PF4 Edge sensitive
	GPIOF->IBE &= ~(1UL << 4);	// Interrupt generation controlled by IEV
	GPIOF->IEV |= (1UL << 4);		// Interrupt generated on rising edge
	GPIOF->PUR |= (1UL << 4); 	// Weak pull up resistor for PF4
	GPIOF->ICR |= (1UL << 4);		// Clear PF4 Interrupt
	GPIOF->IM |= (1UL << 4);		// Enable Interrupts for PF4
}
/* ------------- Timer_Init() -------------------
** Initialize the Timer used in this lab
** Use Timer0A for Clock
** Use Timer0B for polling RX mesg queue
**
*/
void Timer_Init(void)
{
	SYSCTL->RCGCTIMER |= (1UL<<0); //Enable TIMER0
	TIMER0->CTL &= ~(TIMER_CTL_TAEN); //disable TIMER0A while configuring
	TIMER0->CFG = 0x4UL; // independent 16-bit timers
	TIMER0->TAMR = 0x2UL; // TIMER0A count-down, periodic
	TIMER0->TAPR = PRESCALE; // set CLK prescale value
	TIMER0->CTL &= ~(TIMER_CTL_TBEN); // disable TIMER0B while configuring
	TIMER0->TBMR = 0x2UL; // TIMER0B count-down, periodic
	TIMER0->TBPR = PRESCALE; // set CLK prescale value
	TIMER0->ICR |= (TIMER_ICR_TATOCINT | TIMER_ICR_TBTOCINT); // clear time-out interrupt
	TIMER0->CTL |= (1UL << 9); // TBSTALL
}
/* ------------- start_timer() ------------------------
** Start the periodic timer
** The timer0A initially generates an interrupt every sec
** The timer0b initially generates an interrupt every 200ms
**
*/
void start_timer()
{
	TIMER0->TAILR = 62499; // 1 secs
	TIMER0->TBILR = 16000; // 200 ms
	TIMER0->IMR |= (TIMER_IMR_TATOIM | TIMER_IMR_TBTOIM); //arm timer interrupt
	TIMER0->CTL |= (TIMER_CTL_TAEN | TIMER_CTL_TBEN);// enable TIMER0A and TIMER0B
}
/* ------------- faster_Clock() -----------------------
** Increase the speed of clock by decrease the value by 2
**
*/
void faster_Clock()
{
	TIMER0->CTL &= ~(1UL<<0); //disable TIMER0A while configuring
	TIMER0->TAILR = TIMER0 -> TAILR/2; // decrease value by 2
	TIMER0->CTL |= (1UL<<0);// enable TIMER0A
}
/* ------------- slower_Clock() -----------------------
** Decrease the speed of clock by increase the value by 2
**
*/
void slower_Clock()
{
	TIMER0->CTL &= ~(1UL<<0); //disable TIMER0A while configuring
	TIMER0->TAILR = TIMER0 -> TAILR*2; // increase value by 2
	TIMER0->CTL |= (1UL<<0);// enable TIMER0A
}

/* ------------- set_Clock() --------------------
** Set the clock to a specific value
** value format: "THH:MM"
**
*/
void set_Clock(char *val)
{
	char buff[10];
	char sub[2];
	strncpy(sub,&val[1],2);//extract the HH
	hours = atoi(sub);
	strncpy(sub,&val[4],2);//extract the MM
	mins = atoi(sub);
	secs = 0;
	Create_Clock_Display(buff);//generate clock HH:MM:SS
	Tx_message(buff,sizeof(buff));//transmit the buffer
}
/* ------------- pause_Clock() -----------------------
** Pause the Clock
**
*/
void pause_Clock()
{
	TIMER0->CTL &= ~(TIMER_CTL_TAEN); //disable TIMER0A
}
/* ------------- continue_Clock() -----------------------
** continue the Clock
**
*/
void continue_Clock()
{
	TIMER0->CTL |= (TIMER_CTL_TAEN); //enable TIMER0A
}
/* ------------- Create_Clock_Display-----------
** Create a string representation of the clock
** Display format: HH:MM:SS
**
*/
void Create_Clock_Display(char *clock_display)
{
	sprintf(clock_display, "%02d:%02d:%02d\r\n",hours,mins,secs);
}
/* ------------- Increase_Clock ----------------
** Increase the clock by 1 sec
**
*/
void Increase_Clock()
{
	secs++;//increase clock by 1 sec
	if (secs==60)
	{		
		secs = 0;
		mins++;
	}
	if (mins==60)
	{
		mins = 0;
		hours++;
	}
	if (hours==12)
	{
		hours = 0;
	}
}
/* ------------- Timer0A_Handler ---------------
** Handles the interrupt from timer0A
**
*/
void TIMER0A_Handler(void)
{
	char buff[10];
	//clear interrupt at GPTM
	TIMER0->ICR = TIMER_ICR_TATOCINT;
	//clear pending bit in NVIC
	NVIC->ICPR[0] = 1UL<<19;
	Increase_Clock();//Increase the Clock
	if (!busy_input) // if the user is inputting, dont generate Tx data
	{
		Create_Clock_Display(buff);//generate clock HH:MM:SS
		Tx_message(buff, sizeof(buff));//transmit the buffer
	}
}
/* ------------- Timer0B_Handler ---------------
** Handles the interrupt from timer0B
**
*/
void TIMER0B_Handler(void)
{
	buffer_t buff;
	char c;
	//clear interrupt at GPTM
	TIMER0->ICR = TIMER_ICR_TBTOCINT;
	//clear pending bit in NVIC
	NVIC->ICPR[0] = 1UL<<20;
	//read buffer from the rx_queue
	Rx_message(&buff);
	//Compare actions
	c = buff[0];
	switch (c)
	{
		case 'T':
			set_Clock((char*)buff);
			break;
		case 'P':
			pause_Clock();
			break;
		case 'C':
			continue_Clock();
			break;
		case 'F':
			faster_Clock();
			break;
		case 'S':
			slower_Clock();
			break;
	}
}
/* ------------- UART Interrupt Handler --------------
**
**
**
*/
void UART0_Handler(void)
{
	char byte;
	int i,ret;
	static buffer_t rx_temp_buff;
	static uint8_t buff_count = 0;
	buffer_t *rx_buff;
	buffer_t tx_buff;
	/* TX ISR */
	if ((UART0 -> RIS & UART_RIS_TXRIS) != 0)
	{
		//Empty buffer from the Tx queue
		__set_PRIMASK(1); //Disable all interrupts
		ret = release_buffer(&tx_queue,&tx_buff);//get a released buff from tx_queue
		__set_PRIMASK(0); //Enable all interrupts
		if (ret)
		{
			for (i=0; i<BUFFER_SIZE; i++)
			{
				UART0 -> DR = tx_buff[i];
			}
		}
		else //if no more data to send
		{
			UART0 -> IM &= ~(UART_IM_TXIM) ; //mask TX interrupt at UART
			NVIC -> ICPR[0] = 1UL<<5; //clear pending UART interrupt at NVIC
		}
	}
	/* RX ISR*/
	if ((UART0 -> RIS & UART_RIS_RTRIS) != 0)
	{
		UART0 -> ICR |= UART_ICR_RTIC;//clear interrupt
		busy_input = 1; //indicating that user is inputting
		//transfer data from FIFO to the buffer
		do{
			byte = (unsigned char)(UART0 -> DR & 0xFF);
		  rx_temp_buff[buff_count] = byte;
			buff_count++;
			if (byte == 0x0d)  //if byte is carry return
			{
				buff_count = BUFFER_SIZE;
				busy_input = 0; //user finished inputting
			}
		}while((UART0 -> FR & UART_FR_RXFE)==0 && buff_count<BUFFER_SIZE);
		if (buff_count>=BUFFER_SIZE)// if the temp buffer is full
		{
			//Fill the Rx queue with the temp buffer
			__set_PRIMASK(1); //Disable all interrupts
			rx_buff = get_buffer(&rx_queue); //get a buffer from the rx_queue
			__set_PRIMASK(0); //Enable all interrupts
			//copy the temp_buff to the buff
			memcpy(rx_buff, &rx_temp_buff, sizeof(*rx_buff));
			//reset the temp_buff
			buff_count = 0;
		}
		//transmit the byte back to the terminal for feedback
		tx_buff[0] = byte;
		for (i = 1; i<BUFFER_SIZE; i++)
		{
			tx_buff[i]=0x00;
		}
		Tx_message(tx_buff, sizeof(tx_buff));
	}
}
/* ----------- GPIOF_Handler ---------------
** Handle the push button pressed
**
**/
void GPIOF_Handler(void)
{
	
	// If Interrupt caused by PF4
	if(GPIOF->RIS & (1UL << 4))
	{
		GPIOF->ICR |= (1UL << 4);	// Clear Interrupt
		Tx_message("Button Pressed\r\n", sizeof("Button Pressed\r\n"));
	}
}
/* -------------- Stop execution -------------
** Turn on LED and stop execution
**
*/
void stop_execution(void)
{
	GPIOF->DATA |= 0x08;//Turn on the LED
	__set_PRIMASK(1);//disable all interrupt source
	while (1);
}
int main(void)
{
	// initialize UART
	UART_Init(); 
	// initialize NVIC
	NVIC_Init();
	// Initialize GPIO
	GPIO_Init();
	// Initialize the tx_queue and rx_queue
	queue_Init(&tx_queue);	
	queue_Init(&rx_queue);
	// Initialize the Timer
	Timer_Init();
	// Start the UART
	start_UART();
	// Start the timer
	start_timer();
	while(1);
}
