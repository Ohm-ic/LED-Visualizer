# LED-Visualizer
# Audio Processing with STM32F4xx

This repository contains code for audio processing using an STM32F4xx microcontroller. The code includes functionalities for capturing audio from a microphone, performing Fast Fourier Transform (FFT) 
and controlling LEDs based on the audio spectrum.

## Overview

The code is designed to run on an STM32F4xx microcontroller, which is a powerful ARM Cortex-M based microcontroller commonly used in embedded systems.
It utilizes the STM32CubeMX HAL library for peripheral initialization and ARM CMSIS DSP library for signal processing operations.

## Features

- **Audio Sampling**: The code initializes the ADC peripheral to capture audio samples from an external microphone connected to the microcontroller.
  
- **DMA Transfer**: Direct Memory Access (DMA) is used for efficient data transfer between the ADC and memory buffers, reducing CPU overhead.

- **Fast Fourier Transform (FFT)**: ARM CMSIS DSP library is used to perform FFT on the audio samples to analyze the frequency spectrum.

- **LED Visualization**: The frequency spectrum obtained from the FFT is used to control LEDs, providing visual feedback of the audio spectrum.

## Dependencies

- STM32CubeMX HAL library: Provides low-level drivers for STM32 microcontrollers.
- ARM CMSIS DSP library: Provides optimized DSP functions for ARM Cortex-M processors.

## Usage

1. Set up your STM32 development environment and import the code.
2. Configure the project using STM32CubeMX to match your hardware setup.
3. Build and flash the code to your STM32F4xx microcontroller.
4. Connect a microphone and LEDs to the appropriate pins.
5. Power on the system and observe the LED visualization of the audio spectrum.

## Contributors

- Hariom Agrahari

## Acknowledgments

- ARM for providing the CMSIS DSP library.
- STMicroelectronics for providing the STM32CubeMX HAL library.

## References

- [ARM CMSIS DSP library](https://developer.arm.com/ip-products/processors/cortex-m/cortex-microcontroller-software-interface-standard/performance-libraries)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
