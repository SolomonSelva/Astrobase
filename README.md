## This project is done as an Assignment to Astrobase Space Technologies.

# Using STM32F767ZI NUCLEO board, you are required to implement the following: 
1) In the package you received, there is a ADXL345 triaxial accelerometer. You are expected to sample at the highest possible sampling rate that device is capable of.  

2) In addition to the above, attempt these
 a) Generate a sine-wave of frequency 100 Hz using on-board PWM GPIO. Run this every micro cycle.
 b) Sample these values through the on-board adc every minor cycle and store the values for the duration of the major cycle.  
c) Send the stored ADC data (ie data saved during the major cycle ) to your PC via ethernet (TCP/IP) every major cycle. (Incomplete)
 d) Log the data you receive from the STM32 on your PC for 5 mins.(Not implemented)
 e) Note: micro = 0.5 ms; minor = 2 ms; major = 500 ms
