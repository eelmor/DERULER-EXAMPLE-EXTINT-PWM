# DERULER-EXAMPLE-EXTINT-PWM

Example project for DERULER (www.deruler.com)

## What is EXTINT?

This examples shows how to use basic peripherals for external interrupts and pulse-width modulation (PWM) to improve our application. Peripherals in a microcontroller are built-in hardware functions that can be used to off-load the processor. Instead of continuously reading the value of the BUTTON1 and BUTTON2 pins in our code at some interval, we can get notified through an interrupt when the pin value has changed due to the button being pressed. When an interrupt occurs the microcontroller will immediately stop working on what it was doing and jump to the code we've written for what to do when the interrupt occurs. After this code has finished executing the microcontroller will go back to doing what it was doing previously.

## Using PWM for RGB LED control

By using the timer peripheral in PWM mode we can improve the RGB LED functionality by controlling the intensity of each color. This lets us mix red, green and blue to create virtually any color. In our application we set the timer to count from 0 to 255 (256 steps). Each channel (color) can then be set to any value between 0-256 where 0 means fully off and 256 means fully on. With each color having 256 different intensities to select from we can create a total of 256 * 256 * 256 = 16.7M different colors. The 256 value is selected as it represents 8 bits. This means we get 8-bit color per channel, or 24-bit total color information (8-bit * 3 colors, red + green + blue). When BUTTON1 is pressed we start cycling between colors by adding or subtracting different amounts to the the different color. If the button is pressed again we stop cycling the colors. In order to generate sound using the magnetic buzzer in our circuit we can also rely on a timer in PWM mode. By adjusting the timer frequency and PWM duty cycle we can generate different sounds. In this example we configure the timer to generate a 100 Hz square wave signal at 50% duty cycle which is activated when BUTTON2 is pressed. If the button is pressed again the frequency is doubled. If the frequency reaches more than 20 KHz we stop the PWM output.

## Notes
- The code for our main application is in this file main.c. The code for handling interrupts is in stm32l0xx_it.c.
- The audio frequenies we'll hear are: 100 Hz, 200 Hz, 400 Hz, 800 Hz, 1600 Hz, 3200 Hz, 6400 Hz, 12800 Hz
