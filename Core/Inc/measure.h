#ifndef MEASURE_H
#define MEASURE_H

#include <math.h>
#include <stdint.h>
#include "def.h"

#define NB_FFT_CHANNELS 2

#define ADC_BITS            12.
#define ADC_COEFF_A         (500. / pow(2., ADC_BITS))
#define ADC_COEFF_B         127.
#define TENSION_COEFF       0.412572
#define DATA_SENT_PERIOD    10.   // 5. * 60.    // 5 minutes

typedef struct {
    float temp;
    float mean;
    float max;
    float min;
} Rms;

typedef enum {
    INIT = 0,
    WAITING_ZC,
    NORMAL_PHASE
} InitState;

typedef struct {
    float val;
    float prevVal;
    float freqMean;
    float freqMax;
    float freqMin;
    Rms rms;
} Tension;

typedef struct {
    float val;
    float prevVal;
    Rms rms;               // Irms (A)
    float energy;          // I*U.dt (W.s)
} Current;

void measureInit();
void measureAdcCallback(uint32_t* data);

void tensionSampleCalc(float deltaT, bool lastSample);
void tensionPeriodCalc();
void currentSampleCalc(uint32_t* data, float deltaT, bool lastSample);
void currentPeriodCalc();

bool isCrossingZero(float* czPoint);
void initTension(uint32_t* data);
void sendEnergyData();
void tensionInit(Tension* tension);
void currentInit(Current* current);
void rmsInit(Rms* rms);
void saveRms(Rms* rms);
void updateRms(Rms* rms, float val, float deltaT);

void sendError(ErrorCode errorCode, float value);

#endif  // MEASURE_H
