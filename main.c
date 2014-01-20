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

/* vector table, according to reference documentaion, occupied 61 pointer-sized cells  */
unsigned int *myvectors[61] __attribute__ ((section ("vectors"))) =
{
  [0]  = (unsigned int *) STACK_TOP,
  [1]  = (unsigned int *) main,
  [2]  = (unsigned int *) nmi_handler,
  [3]  = (unsigned int *) hardfault_handler,
  [22] = (unsigned int *) set_button, /* EXTI0 interrupt */
  [41] = (unsigned int *) repaint_screen /* Tim9 interrupt interrupt */
};

volatile uint16_t display_data = 0;
volatile uint8_t  count_timer  = 0;

int
main (void)
{
  int n = 0;
  int i = 0;
  display_data = 0;
  count_timer  = 0;

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

  configure_exti0_to_pa0 ();
  configure_tim9 ();

  while (1)
    {
      if (++display_data > 999) display_data %= 1000;	/* p is displayed number */
      /* display_data = TIM9->CNT; */
      
      for (i = 0; i < 800; i++)
	delay ();

      n++;			/* Count the delays */
      if (n & 1)		/* 1 / 1 ticks */
      	{
      	  GPIOB->BSRRL = 1 << 6;
      	}
      else
      	{
      	  GPIOB->BSRRH = 1 << 6;
      	}
      if (n & 2)  /* 2 / 2 ticks */
      	{
      	  GPIOB->BSRRL = 1 << 7;
      	}
      else
      	{
      	  GPIOB->BSRRH = 1 << 7;
      	}
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

  GPIOA->ODR ^= 1 << 11;	/* Change PA11 output state */

  return;
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
repaint_screen (void)
{
  if (TIM9->SR & (1 << 0))
    TIM9->SR &= ~(1 << 0);

  count_timer = (count_timer + 1) % 3; /* Light (tick % 3) symbol on led screen */
  showi (display_data, count_timer);
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

