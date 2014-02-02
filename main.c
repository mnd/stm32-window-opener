#include <stm32l1xx.h>
#include <stdint.h>
#include <stdbool.h>
#define STACK_TOP 0x20004000 // End of memory

void nmi_handler (void);
void hardfault_handler (void);
void delay (void);
int main (void);

void set_button (void);
void repaint_screen (void);

void show_segs (uint8_t bits, int digit);
void show (const int bits[8], const int digit);
void show_num (int num, int dig, bool dot);
void showi (uint16_t num, uint8_t n);
void showh (uint16_t num, uint8_t n);

void configure_exti0_to_pa0 (void);
void configure_tim9 (void);
void configure_usart2_pa23 (void);
void configure_pwm_tim3_pc6 (void);

uint16_t get_temperature (void);

void set_window (uint8_t degree);
uint8_t get_window (void);

void open_window (void);
void close_window (void);
void process_window (int16_t themperature);

#define WIN_OPEN  1740
#define WIN_CLOSE 1000

/* vector table, according to reference documentaion,
   occupied 61 pointer-sized cells  */
unsigned int *myvectors[61] __attribute__ ((section ("vectors"))) =
{
  [0]  = (unsigned int *) STACK_TOP,
  [1]  = (unsigned int *) main,
  [2]  = (unsigned int *) nmi_handler,
  [3]  = (unsigned int *) hardfault_handler,
  [22] = (unsigned int *) set_button, /* EXTI0 interrupt */
  [41] = (unsigned int *) repaint_screen /* Tim9 interrupt */
  //  [54] = (unsigned int *) undefined /* USART2 interrupt on IRQ 38 */
};

volatile uint16_t display_data = 0;
volatile uint8_t  count_timer  = 0;

volatile enum {CLOSE = 0, OPEN = 1, AUTO = 2} window_status;

int
main (void)
{
  int i = 0;
  display_data = 0x05ED;
  count_timer  = 0;
  window_status = AUTO;

  /* Enable GPIOA, GPIOB, GPIOC */
  RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
  /* Set GPIOB Pin 6 and Pin 7 to outputs */
  GPIOB->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0;
  /* Set GPIOA Pin 11 to output */
  GPIOA->MODER |= GPIO_MODER_MODER11_0;

  /* PB6 and PB7 is OFF */
  GPIOB->BSRRH |= 1 << 6 | 1 << 7;
  GPIOA->BSRRH |= 1 << 11;	   /* PA11 is OFF */

  /* LED Screen-pins to GPIO */
  GPIOB->MODER |= 1 << (10 * 2) | 1 << (11 * 2) | 1 << (12 * 2) | 1 << (15 * 2);
  GPIOC->MODER |= 1 << (0 * 2) | 1 << (1 * 2) | 1 << (2 * 2) | 1 << (3 * 2);
  GPIOC->MODER |= 1 << (10 * 2) | 1 << (11 * 2) | 1 << (12 * 2);

  /* Disable all digits */
  GPIOB->BSRRH |= 1 << 10 | 1 << 11 | 1 << 12;


  /* Button */
  configure_exti0_to_pa0 ();
  /* themperature */
  configure_usart2_pa23 ();
  /* screen */
  configure_tim9 ();
  /* servo */
  configure_pwm_tim3_pc6 ();
  while (1)
    {
      /* display_data = TIM9->CNT; */
      int16_t temperature = get_temperature ();
      display_data = temperature;

      switch (window_status)
	{
	case CLOSE:
	  close_window ();
	  GPIOB->BSRRH = 1 << 6;
      	  GPIOB->BSRRH = 1 << 7;
	  GPIOA->BSRRL = 1 << 11; /* red */
	  break;
	case OPEN:
	  open_window ();
	  GPIOB->BSRRH = 1 << 6;
      	  GPIOB->BSRRL = 1 << 7; /* green */
	  GPIOA->BSRRH = 1 << 11;
	  break;
	case AUTO:
	  process_window (temperature);
      	  GPIOB->BSRRL = 1 << 6; /* blue */
      	  GPIOB->BSRRH = 1 << 7;
	  GPIOA->BSRRH = 1 << 11;
	  break;
	}

      for (i = 0; i < 800; i++)
      	delay ();
    }
}

void
show_segs (uint8_t bits, int digit)
{
  GPIOB->BSRRH |= 1 << 10 | 1 << 11 | 1 << 12; /* Disable all digits */
  /* Disable all segments */
  GPIOB->BSRRL |= 1 << 15;
  GPIOC->BSRRL |= 0x1C0F; /* 0001 1100 0000 1111 */
  if (digit > 2) return;

  /* enable right digit, according to argument */
  GPIOB->BSRRL |= 1 << (10 + digit);

  uint16_t digits = 0;
  digits |= bits & 0x0F;	/* first 4 segments */
  digits |= (bits & 0x70) << 6;	/* next  3 segments */

  if (bits & 0x80) GPIOB->BSRRH |= 1 << 15;
  GPIOC->BSRRH |= digits;
}

/* Digit bitmap:
     6
   5   4
     3
   0   2
     1     7
*/
const uint8_t _digits[34] = {
  0x77, /* 0  0111 0111 */
  0x14, /* 1  0001 0100 */
  0x5B, /* 2  0101 1011 */
  0x5E, /* 3  0101 1110 */
  0x3C, /* 4  0011 1100 */
  0x6E, /* 5  0110 1110 */
  0x6F, /* 6  0110 1111 */
  0x54, /* 7  0101 0100 */
  0x7F, /* 8  0111 1111 */
  0x7E, /* 9  0111 1110 */
  0x7D, /* a  0111 1101 */
  0x2F, /* b  0010 1111 */
  0x63, /* c  0110 0011 */
  0x1F, /* d  0001 1111 */
  0x6B, /* e  0110 1011 */
  0x69, /* f  0110 1001 */

  0xF7, /* 0. 1111 0111 */
  0x94, /* 1. 1001 0100 */
  0xDB, /* 2. 1101 1011 */
  0xDE, /* 3. 1101 1110 */
  0xBC, /* 4. 1011 1100 */
  0xEE, /* 5. 1110 1110 */
  0xEF, /* 6. 1110 1111 */
  0xD4, /* 7. 1101 0100 */
  0xFF, /* 8. 1111 1111 */
  0xFE, /* 9. 1111 1110 */
  0xFD, /* a. 1111 1101 */
  0xAF, /* b. 1010 1111 */
  0xE3, /* c. 1110 0011 */
  0x9F, /* d. 1001 1111 */
  0xEB, /* e. 1110 1011 */
  0xE9, /* f. 1110 1001 */

  0x08, /* -  0000 1000 */
  0x88  /* -. 1000 1000 */
};


inline void
show_num (int num, int dig, bool dot)
{
  if (dot && num < 16) num += 16;
  uint16_t n = _digits[num];
  show_segs (n, dig);
}

inline void
showi (uint16_t num, uint8_t n)
{
  switch (n)
    {
    case 0:
      show_num ((num / 100) % 10, 0, false);
      break;
    case 1:
      show_num ((num / 10) % 10,  1, false);
      break;
    case 2:
      show_num (num % 10,         2, false);
      break;
    default:
      break;
    }
}

inline void
showh (uint16_t num, uint8_t n)
{
  switch (n)
    {
    case 0:
      show_num (num & 0xF, 2, false);
      break;
    case 1:
      show_num ((num >> 4) & 0xF, 1, false);
      break;
    case 2:
      show_num ((num >> 8) & 0xF, 0, false);
      break;
    default:
      break;
    }
}

inline void
show_temp (uint16_t num, uint8_t n)
{
  if (num >= 0)
    switch (n)
      {
      case 0:
	show_num (((num >> 4) / 10) % 10, 0, false);
	break;
      case 1:
	show_num (((num >> 4)) % 10,  1, true);
	break;
      case 2:
	show_num (num & 0xF,         2, false);
	break;
      default:
	break;
      }
  else
    switch (n)
      {
      case 0:
	show_num (32, 0, false);
	break;
      case 1:
	show_num ((((-num) >> 4) / 10) % 10, 0, false);
	break;
      case 2:
	show_num ((((-num) >> 4)) % 10,  1, false);
	break;
      default:
	break;
      }
}


void
delay (void)
{
  int i = 80;
  while (i-- > 0)
    {
      asm ("nop");		/* This stops it optimising code out */
    }
}

void
set_button (void)
{
  if (EXTI->PR & (1<<0))
    {                        // EXTI0 interrupt pending?
      EXTI->PR |= (1<<0);    // clear pending interrupt
    }

  window_status = (window_status + 1) % 3;
}

void
configure_exti0_to_pa0 (void)
{
  /* Enable GPIOA on case if it's not enabled */
  RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
  /* Set GPIOA Pin 0 to input floating */
  GPIOA->MODER &= ~GPIO_MODER_MODER0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0;
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;
  /* Connect PA0 to EXTI0 line */
  SYSCFG->EXTICR[0] &= ~0xF;
  SYSCFG->EXTICR[0] |= 0;
  /* Enable interrupts on EXTI0 */
  EXTI->IMR |= 1;
  /* Send event when signal rising */
  EXTI->RTSR |= 1;
  /* Enable IRQ 6 with NVIC Interrupt Set-Pending Register */
  NVIC->ISER[0] = (uint32_t)0x01 << 6;

  /* Set IRQ6 priority. IRQ6 in NVIC_IP[6] register */
  /* NVIC->IP[6] = 0x7F; */
}

void
usart2_set_baud_rate (int rate)
{
  /* baud = Fck / (8 x (2-over8) x usartdiv) */
  /* Assume that we use HSI timer and over8 == 0. USART2->BRR (aka usartdiv)
     is fixed point fractional number ABC.D, and if we interpret it like integer
     number ABCD then (uint16_t) USART2->BRR == 16 * usartdiv.
     So baud = Fck / (8 x (2-0) x usartdiv) = Fck / (16 x usartdiv)
             = Fck / ((uint16_t) USART2->BRR),
     and (uint16_t) USART2->BRR = Fck / rate */
  USART2->BRR = (uint16_t) ((HSI_VALUE) / rate);
}

void
configure_usart2_pa23 (void)
{
  /* Switch from MSI to HSI, e.g. from 2MHz clock to 16 MHz */
  RCC->CR   |= RCC_CR_HSION;	/* enable HSI */
  RCC->CFGR |= RCC_CFGR_SW_HSI;	/* switch to HSI */
  int i = 0;
  do { i++; } while (i < 1000);

  /* Enable GPIOA if not enabled */
  RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;

  /* PA2 to Alternate function mode */
  GPIOA->MODER &= ~GPIO_MODER_MODER2;
  GPIOA->MODER |= GPIO_MODER_MODER2_1;
  /* PA2 mode to Open Drain. Need when connected without shottke diode */
  GPIOA->OTYPER |= GPIO_OTYPER_OT_2;
  /* And set pull-up io */
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0;
  /* PA2 Speed to 40MHz */
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;
  /* Configure PA2 alternative function to USART2 aka AF7*/
  GPIOA->AFR[0] |= 0x7 << (2 * 4);

  /* PA3 to input floating mode */
  GPIOA->MODER &= ~GPIO_MODER_MODER3;
  GPIOA->MODER |= GPIO_MODER_MODER3_1;
  /* PA3 speed to 40MHz */
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
  /* Configure PA3 alternative function to USART2 aka AF7*/
  GPIOA->AFR[0] |= 0x7 << (3 * 4);

  /* Enable USART2 */
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  /* Set word length to 8 bit */
  USART2->CR1 &= ~USART_CR1_M;
  /* Set 1 stop bit */
  USART2->CR2 &= ~USART_CR2_STOP;
  /* Disable parity */
  USART2->CR1 &= ~USART_CR1_PCE;
  /* Disable hardware flow control */
  USART2->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
  /* RX|TX mode enable */
  USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

  /* Set baud rate to 115.2 KBps */
  usart2_set_baud_rate (115200);

  /* Enable USART2 */
  USART2->CR1 |= USART_CR1_UE;
}

uint8_t
ow_usart2_reset ()
{
  usart2_set_baud_rate (9600);

  /* clear transmission complite flag */
  USART2->SR &= ~USART_SR_TC;
  /* Send 0xf0 byte throw USART2 */
  USART2->DR = 0xF0;
  /* wait while transmission is not completed */
  do {} while ((USART2->SR & USART_SR_TC) == 0);
  uint8_t answer = USART2->DR;

  usart2_set_baud_rate (115200);

  return answer;
}

/* bit == 0xFF || 0x00 */
inline void
ow_usart2_write_bit (uint8_t bit)
{
  /* clear transmission complite flag */
  USART2->SR &= ~USART_SR_TC;
  /* Send byte throw USART2 */
  USART2->DR = bit;
  /* wait while transmission is not completed */
  do {} while ((USART2->SR & USART_SR_TC) == 0);

  return;
}

/* return value 0x00 or 0x01 */
inline uint8_t
ow_usart2_read_bit ()
{
  /* clear transmission complite flag */
  USART2->SR &= ~USART_SR_TC;
  /* Send 0xf0 byte throw USART2 */
  USART2->DR = 0xFF;
  /* wait while transmission is not completed */
  do {} while ((USART2->SR & USART_SR_TC) == 0);
  uint8_t answer = USART2->DR;

  return (answer == 0xFF ? 0x01 : 0x0);
}

uint16_t
get_temperature ()
{
  uint16_t result = 0;
  uint8_t convert[16] =
    {0x00, 0x00, 0XFF, 0xFF, 0x00, 0x00, 0XFF, 0xFF, /* 0xCC -- SKIP ROM */
     0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00}; /* 0x44 -- CONVERT T */
  uint8_t request[16] =
    {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, // 0xcc SKIP ROM
     0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF}; // 0xbe READ SCRATCH

  int i, j;

  /* reset 1wire */
  ow_usart2_reset ();
  /* write 0xCC 0x44 */
  for (i = 0; i < 16; ++i)
    {
      ow_usart2_write_bit (convert[i]);
    }
  /* wait for calculation */
  for (i = 0; i < 100000; ++i);

  ow_usart2_reset ();
  for (i = 0; i < 16; ++i)
    {
      ow_usart2_write_bit (request[i]);
    }

  /* read 2 bytes answer */
  for (i = 0; i < 16; ++i)
    {
      uint8_t bit = ow_usart2_read_bit ();
      result |= bit << i ;
      for (j = 0; j < 100; ++j);
    }
  return result;
}

void
configure_tim9 (void)
{
  /* Enable tim9 timer */
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
  /* Set clock dividion to 1x */
  TIM9->CR1 &= ~(3 << 8);
  /* interrupt on update event */
  TIM9->DIER |= 1;
  /* Set timer frequency every 0x1 + 1 tick */
  TIM9->PSC = 0x1;
  /* Set timer auto reload value, when timer count more than 0x100 */
  TIM9->ARR = 0x100;
  /* Enable timer */
  TIM9->CR1 |= 1;

  /* enable IRQ channel for TIM9 */
  NVIC->ISER[0] = (uint32_t) 1 << 25;
  return;
}

void
configure_pwm_tim3_pc6 ()
{
  /* Enable GPIOC if not enabled */
  RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
  /* Enable alternative mode for PC6 */
  GPIOC->MODER &= ~GPIO_MODER_MODER6;
  GPIOC->MODER |= GPIO_MODER_MODER6_1;
  /* PC6 Speed to 40MHz */
  GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6;
  /* Configure PC6 alternative function to TIM3 (TIM3..TIM5) aka AF2*/
  GPIOC->AFR[0] |= 0x2 << (6 * 4);

  /* Enable tim3 timer */
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  /* Set clock interrupt on every sampling clock interrupt */
  TIM3->CR1 &= ~TIM_CR1_CKD;
  /* Enable ARR auto reload */
  TIM3->CR1 |= TIM_CR1_ARPE;
  /* Edge aligned mode */
  TIM3->CR1 &= ~TIM_CR1_CMS;
  /* Upcount */
  TIM3->CR1 &= ~TIM_CR1_DIR;

  /* Select PWM mode 1 in output capture mode */
  TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
  /* Enable preload register in tim3_ccr1 */
  TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
  /* Counter clock frequency = psc + 1 = at every tick. */
  /* If we use 16MHz HSI clock, then tim3 would work at 1MHz rate. */
  TIM3->PSC = 0xF;
  /* Set timer autoreload value on every 10000ticks
     (10 miliseconds according previous configuration) */
  TIM3->ARR = 10000;
  /* enable output */
  TIM3->CCER |= TIM_CCER_CC1E;

  /* Set PWM width to 0° and enable auto window mode */
  window_status = AUTO;

  /* Initialize shadowed registers */
  TIM3->EGR |= TIM_EGR_UG;
  /* Enable TIM3 counter */
  TIM3->CR1 |= TIM_CR1_CEN;
}

uint8_t degree;
/* set_window (0) to close window.
   set_window (100) to open window.
   Argument takes with step 1. */
void
set_window (uint8_t deg)
{
  degree = deg;
  uint32_t step = (WIN_OPEN - WIN_CLOSE); /* make it dividable by 16 */
  TIM3->CCR1 = (degree == 100) ? WIN_OPEN
                               : WIN_CLOSE + (((uint32_t) deg) * step)/100;
}

inline uint8_t
get_window (void)
{
  return degree;
}

void
open_window (void)
{
  if (get_window () != 100)
      set_window (100);
}

void
close_window (void)
{
  if (get_window () != 0)
    set_window (0);
}

void
semiopen_window (void)
{
  if (get_window () != 55)
    set_window (55);
}

void
process_window (int16_t themperature)
{
  if (themperature < (20 << 4))	/* less than 20.0°C */
    close_window ();
  else if (themperature > (24 << 4)) /* more than 24.4°C */
    open_window ();
  else if (themperature > (21 << 4) && themperature < (23 << 4))
    /* 21°C < t < 23 °C */
    semiopen_window ();
}

void
repaint_screen (void)
{
  if (TIM9->SR & (1 << 0))
    TIM9->SR &= ~(1 << 0);

  count_timer = (count_timer + 1) % 3; /* Light (tick % 3) symbol on led screen */
  show_temp (display_data, count_timer);
}

void
nmi_handler (void)
{
  return;
}

void
hardfault_handler (void)
{
  return;
}

