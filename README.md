# stepper
C code for stepper motors using timer interupts on a STM32F407

This code provides basic low level example of using timers for generating the pulse train for a stepper motor.
An interupter fires after each step(pulse of the timer) and calculates the needed timing for next step in real time
for smooth ramp up and ramp down when changing speed. 

Using prinicable found here https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/

At low level the speed of the stepper is governed by the count in the ARR of the timer, the larger the number the 
timer has to count up to the slower the step rate of the motor

A video of the code running can be seen here https://www.youtube.com/watch?v=ryh_fY4fxnU&feature=youtu.be
