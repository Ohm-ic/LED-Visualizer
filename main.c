#include "stm32f4xx.h"
#include "arm_math.h"
#include "audio_processing.h"

#define FFT_SIZE 1024 // FFT size
#define NUM_LEDS 16   // Number of LEDs

#define MIC_BUFFER_SIZE 1024 // Microphone buffer size
#define AUDIO_BUFFER_SIZE (FFT_SIZE * 2) // Audio buffer size

uint16_t micBuffer[MIC_BUFFER_SIZE]; // Microphone buffer
uint16_t audioBuffer[AUDIO_BUFFER_SIZE]; // Audio buffer

uint8_t ledColors[NUM_LEDS]; // LED colors

arm_rfft_instance_q15 fftInstance; // FFT instance
q15_t fftBuffer[FFT_SIZE * 2]; // FFT buffer
q15_t fftOutput[FFT_SIZE]; // FFT output

void init_ADC() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
    GPIOC->MODER |= GPIO_MODER_MODE1; // Configure PC1 as analog mode

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock
    ADC1->CR2 |= ADC_CR2_ADON; // Turn on ADC1

    // Configure ADC1 Channel 11 (PC1)
    ADC1->SQR3 |= 11; // Select channel 11
}

void init_DMA() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // Enable DMA1 clock
    DMA1_Stream4->CR &= ~DMA_SxCR_EN; // Disable DMA stream 4

    // Configure DMA1 Stream 4
    DMA1_Stream4->PAR = (uint32_t)&(ADC1->DR); // Set peripheral address
    DMA1_Stream4->M0AR = (uint32_t)micBuffer; // Set memory 0 address
    DMA1_Stream4->NDTR = MIC_BUFFER_SIZE; // Set number of data to transfer
    DMA1_Stream4->CR |= DMA_SxCR_CHSEL_0; // Select channel 0
    DMA1_Stream4->CR |= DMA_SxCR_MSIZE_0; // Set peripheral data size to 16-bit
    DMA1_Stream4->CR |= DMA_SxCR_PSIZE_0; // Set memory data size to 16-bit
    DMA1_Stream4->CR |= DMA_SxCR_MINC; // Enable memory increment mode
    DMA1_Stream4->CR |= DMA_SxCR_CIRC; // Enable circular mode
    DMA1_Stream4->CR |= DMA_SxCR_TCIE; // Enable transfer complete interrupt

    // Configure NVIC for DMA1 Stream 4
    NVIC_EnableIRQ(DMA1_Stream4_IRQn); // Enable DMA1 Stream 4 interrupt
    NVIC_SetPriority(DMA1_Stream4_IRQn, 0); // Set priority to highest

    DMA1_Stream4->CR |= DMA_SxCR_EN; // Enable DMA stream 4
}

void init_TIM() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 84 - 1; // Set prescaler to 84
    TIM2->ARR = 500; // Set auto-reload value to 500
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt
    TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2
}

void init_LEDs() {
    // Configure GPIOB for LEDs
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock
    GPIOB->MODER |= GPIO_MODER_MODER0_0; // Set pin PB0 as output
}

void DMA1_Stream4_IRQHandler() {
    if (DMA1->HISR & DMA_HISR_TCIF4) { // If transfer complete interrupt flag is set
        DMA1->HIFCR |= DMA_HIFCR_CTCIF4; // Clear transfer complete interrupt flag

        // Process audio buffer
        processAudioBuffer();

        // Restart DMA
        DMA1_Stream4->CR &= ~DMA_SxCR_EN; // Disable DMA stream 4
        DMA1_Stream4->NDTR = MIC_BUFFER_SIZE; // Reset number of data to transfer
        DMA1_Stream4->CR |= DMA_SxCR_EN; // Enable DMA stream 4
    }
}

void TIM2_IRQHandler() {
    if (TIM2->SR & TIM_SR_UIF) { // If update interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag

        // Perform FFT
        arm_rfft_q15(&fftInstance, (q15_t*)audioBuffer, fftOutput);

        // Calculate LED colors based on FFT output
        calculateLEDColors();

        // Display LED colors
        displayLEDColors();
    }
}

int main(void) {
    // Initialize peripherals
    init_ADC();
    init_DMA();
    init_TIM();
    init_LEDs();

    // Initialize FFT
    arm_rfft_init_q15(&fftInstance, FFT_SIZE, 0, 1);

    while (1) {
        // Main loop
    }
}