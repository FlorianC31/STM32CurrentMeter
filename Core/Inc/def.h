#ifndef DEF_H
#define DEF_H

#define bool int
#define true 1
#define false 0

#ifndef UNUSED
#define UNUSED (void)
#endif

#define CLOCK_FREQ      216                 // MHz
#define NB_SAMPLES      128
#define TIM_CONSTANT    0.3125              // µs
#define TIM_PERIOD      TIM_CONSTANT * CLOCK_FREQ * NB_SAMPLES


#define NB_CURRENTS 5                   // 5 current channels
#define NB_ADC_PINS NB_CURRENTS + 2     // + 1 channel for the tension and 1 for Vref

#define NB_ADC1_PINS 4
#define NB_ADC3_PINS 3

#endif  // DEF_H
