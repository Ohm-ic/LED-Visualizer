#include "stm32f4xx.h"
#include <stdint.h>

#define SAMPLE_FREQ 10000 // Sample frequency in Hz
#define CUTOFF_FREQ 1000  // Cutoff frequency in Hz

float inputBuffer[2]; // Buffer to hold input samples
float outputBuffer[2]; // Buffer to hold output samples

void init_ADC() {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock

    // Configure ADC1
    ADC1->CR2 |= ADC_CR2_ADON; // Turn on ADC1
    ADC1->CR2 |= ADC_CR2_CONT; // Enable continuous conversion mode
    ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversion

    // Configure ADC1 regular channel 0 (PA0)
    ADC1->SQR3 |= 0; // Select channel 0
}

void init_Timer() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock

    TIM2->PSC = SystemCoreClock / SAMPLE_FREQ - 1; // Set prescaler
    TIM2->ARR = 1; // Set auto-reload value
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt
    TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2
}

void TIM2_IRQHandler() {
    if (TIM2->SR & TIM_SR_UIF) { // If update interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag

        // Read ADC value
        ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversion
        while (!(ADC1->SR & ADC_SR_EOC)); // Wait for conversion to complete
        inputBuffer[0] = ADC1->DR / 4095.0; // Normalize ADC value to range [0, 1]

        // Implement low-pass filter
        outputBuffer[0] = 0.999 * outputBuffer[1] + 0.001 * inputBuffer[0];

        // Update output for next iteration
        outputBuffer[1] = outputBuffer[0];
    }
}

int main(void) {
    init_ADC(); // Initialize ADC
    init_Timer(); // Initialize Timer

    while (1) {
        // Main loop
