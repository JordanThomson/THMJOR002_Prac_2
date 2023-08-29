
//********************************************************************
//*                    EEE2046F C template                           *
//*==================================================================*
//* WRITTEN BY: Jordan Thomson                                       *
//* DATE CREATED: 09/05/2023                                         *
//* MODIFIED: 10/05/2023                                             *
//*==================================================================*
//* PROGRAMMED IN: Visual Studio Code                                *
//* TARGET:        STM32F0                                           *
//*==================================================================*
//* DESCRIPTION:                                                     *
//* Coding for Practical 4. The objective is to change the way the   *
//* LEDs turn on and off when different buttons are pressed           *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
#include "lcd_stm32f0.h"
#include "stdint.h"
#include "stdio.h"
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
#define DELAY1 100
#define DELAY2 8000
#define TRUE 1
#define FALSE 0
//====================================================================
// GLOBAL VARIABLES
//====================================================================

typedef uint8_t flag_t;

flag_t startFlag = FALSE;
flag_t lapFlag = FALSE;
flag_t stopFlag = FALSE;
flag_t resetFlag = TRUE;
uint8_t minutes = 0;
uint8_t seconds = 0;
uint8_t hundredths = 0;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void initGPIO(void);
void initTIM14(void);
void TIM14_IRQHandler(void);
void display(void);
void checkPB(void);
void convert2BCDASCII(const uint8_t min, const uint8_t sec, const uint8_t hund, char* resultPtr);
//====================================================================
// MAIN FUNCTION
//====================================================================
int main(void)
{   
    init_LCD();
    initGPIO();
    initTIM14();
    
    for (;;)
    {
        TIM14->CR1 |= TIM_CR1_CEN;
        while (!(TIM14->SR & TIM_SR_UIF));  // Wait for the update event interrupt flag to be set
        display();                         // Call the display function
        TIM14->SR &= ~TIM_SR_UIF;           // Clear the update event interrupt flag
    }
}							// End of main

//====================================================================
// FUNCTION DEFINITIONS
//====================================================================

// Set the clock and ports to appropraite settings
void initGPIO(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOB->MODER |= GPIO_MODER_MODER0_0;
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
    GPIOB->MODER |= GPIO_MODER_MODER2_0;
    GPIOB->MODER |= GPIO_MODER_MODER3_0;
    GPIOB->MODER |= GPIO_MODER_MODER4_0;
    GPIOB->MODER |= GPIO_MODER_MODER5_0;
    GPIOB->MODER |= GPIO_MODER_MODER6_0;
    GPIOB->MODER |= GPIO_MODER_MODER7_0;
    GPIOB->MODER |= GPIO_MODER_MODER10_0;
    GPIOB->MODER |= GPIO_MODER_MODER11_0;

    GPIOB->ODR &= ~GPIO_ODR_0;
    GPIOB->ODR &= ~GPIO_ODR_1;
    GPIOB->ODR &= ~GPIO_ODR_2;
    GPIOB->ODR &= ~GPIO_ODR_3;
    GPIOB->ODR &= ~GPIO_ODR_4;
    GPIOB->ODR &= ~GPIO_ODR_5;
    GPIOB->ODR &= ~GPIO_ODR_6;
    GPIOB->ODR &= ~GPIO_ODR_7;
    GPIOB->ODR &= ~GPIO_ODR_10;
    GPIOB->ODR &= ~GPIO_ODR_11;

    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    GPIOA->MODER &= ~GPIO_MODER_MODER1;
    GPIOA->MODER &= ~GPIO_MODER_MODER2;
    GPIOA->MODER &= ~GPIO_MODER_MODER3;

    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR3_0;
}

void initTIM14(void) {
    // Enable TIM14 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

    // Set the Auto-Reload Register (ARR) and Prescaler (PSC)
    TIM14->PSC = 7;
    TIM14->ARR = 59999;    

    // Configure the timer in upcounting mode
    TIM14->CR1 &= ~TIM_CR1_DIR;  // Upcounting mode

    // Enable update event interrupts
    TIM14->DIER |= TIM_DIER_UIE;

    // Stop the timer
    TIM14->CR1 &= ~TIM_CR1_CEN;

    // Clear any pending update interrupt
    NVIC_ClearPendingIRQ(TIM14_IRQn);

    // Enable Timer 14 update event interrupts in NVIC
    NVIC_EnableIRQ(TIM14_IRQn);
}


void TIM14_IRQHandler(void) {
    // Check if the update event interrupt flag is set
    if (TIM14->SR & TIM_SR_UIF) {
        // Clear the update event interrupt flag
        TIM14->SR &= ~TIM_SR_UIF;

        // Increment the hundredths
        hundredths++;
        if (hundredths >= 100) {
            hundredths = 0;
            seconds++;
            if (seconds >= 60) {
                seconds = 0;
                minutes++;
                if (minutes >= 60) {
                    minutes = 0;
                }
            }
        }

        // Update the stopwatch time on the LCD
        display();
    }
}


void display(void) {
    checkPB();
    if (!startFlag && !lapFlag && !stopFlag && resetFlag) {
        // Display "Stopwatch: Press SW0..." message
        minutes = 0;
        seconds = 0;
        hundredths = 0;
        lcd_command(CLEAR);
        lcd_putstring("Stopwatch");
        lcd_command(0xC0); // Move cursor to the second line
        lcd_putstring("Press SW0...");
        uint8_t bitpattern1 = 0b00001000;
        GPIOB->ODR = bitpattern1;
    } else if (startFlag && !lapFlag && !stopFlag && !resetFlag) {
        // Display current time
        char timeString[9];
        convert2BCDASCII(minutes, seconds, hundredths, timeString); // Convert to BCD ASCII format
        lcd_command(CLEAR);
        lcd_putstring("Time");
        lcd_command(0xC0); // Move cursor to the second line
        lcd_putstring(timeString);
        uint8_t bitpattern1 = 0b10000000;
        GPIOB->ODR = bitpattern1;
    } else if (startFlag && lapFlag && !stopFlag && !resetFlag) {
        // Display current lap time
        char lapTimeString[9];
        convert2BCDASCII(minutes, seconds, hundredths, lapTimeString); // Convert to BCD ASCII format
        lcd_command(CLEAR);
        lcd_putstring("Time");
        lcd_command(0xC0); // Move cursor to the second line
        lcd_putstring(lapTimeString);
        uint8_t bitpattern1 = 0b00000010;
        GPIOB->ODR = bitpattern1;
    } else if (startFlag && !lapFlag && stopFlag && !resetFlag) {
        // Display current time (stopped)
        TIM14->CR1 &= ~TIM_CR1_CEN;
        char stopTimeString[9];
        convert2BCDASCII(minutes, seconds, hundredths, stopTimeString); // Convert to BCD ASCII format
        lcd_command(CLEAR);
        lcd_putstring("Time");
        lcd_command(0xC0); // Move cursor to the second line
        lcd_putstring(stopTimeString);
        uint8_t bitpattern1 = 0b00000100;
        GPIOB->ODR = bitpattern1;
    }
}

void checkPB(void){
    if((GPIOA->IDR & GPIO_IDR_0)==0){
       startFlag = TRUE;
       lapFlag = FALSE;
       stopFlag = FALSE;
       resetFlag= FALSE;
    }
    if((GPIOA->IDR & GPIO_IDR_1)==0){
       startFlag = TRUE;
       lapFlag = TRUE;
       stopFlag = FALSE;
       resetFlag= FALSE;
    }
    if((GPIOA->IDR & GPIO_IDR_2)==0){
       startFlag = TRUE;
       lapFlag = FALSE;
       stopFlag = TRUE;
       resetFlag= FALSE; 
    }
    if((GPIOA->IDR & GPIO_IDR_3)==0){
       startFlag = FALSE;
       lapFlag = FALSE;
       stopFlag = FALSE;
       resetFlag= TRUE; 
    }

}

void convert2BCDASCII(const uint8_t min, const uint8_t sec, const uint8_t hund, char* resultPtr) {
    // Convert minutes and seconds to BCD format
    uint8_t bcdMin = ((min / 10) << 4) | (min % 10);
    uint8_t bcdSec = ((sec / 10) << 4) | (sec % 10);

    // Separate the hundredths into tens and units
    uint8_t hundTens = hund / 10;
    uint8_t hundUnits = hund % 10;

    // Convert BCD values to ASCII characters
    resultPtr[0] = (bcdMin / 10) + '0';
    resultPtr[1] = (bcdMin % 10) + '0';
    resultPtr[2] = ':';
    resultPtr[3] = (bcdSec / 10) + '0';
    resultPtr[4] = (bcdSec % 10) + '0';
    resultPtr[5] = '.';
    resultPtr[6] = hundTens + '0';
    resultPtr[7] = hundUnits + '0';
    resultPtr[8] = '\0';
}






//********************************************************************
// END OF PROGRAM
//********************************************************************