#ifndef DEF_H
#define DEF_H

#define bool int
#define true 1
#define false 0

#ifndef UNUSED
#define UNUSED (void)
#endif

#define CLOCK_FREQ      216                                     // MHz
#define NB_SAMPLES      256
#define ANALYZED_PERIOD 20.480                                  // ms (A little bit more than 1/50Hz = 20ms)
#define TIM_PERIOD      ANALYZED_PERIOD * 1000 / NB_SAMPLES     // µs
#define TIM_DIVIDER     TIM_PERIOD * CLOCK_FREQ

#define NB_CURRENTS 5                   // 5 current channels
#define NB_ADC_PINS NB_CURRENTS + 2     // + 1 channel for the tension and 1 for Vref

#define NB_ADC1_PINS 4
#define NB_ADC3_PINS 3

#endif  // DEF_H
