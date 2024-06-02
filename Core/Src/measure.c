#include "measure.h"

static Current current[NB_CURRENTS];

static const float CURRENT_COEFFS[] = {1.2, 1.12, 1.54, 1.23, 1.42};

static Tension tension;

//static uint16_t fftId;
//static uint8_t fftBuffer[NB_FFT_CHANNELS][NB_SAMPLES];
 
//static uint32_t energyCounter;
//static uint32_t val5min;
 
static float zeroIndex;
static float prevZeroIndex;

static bool fftRunning;

static InitState initState;

static float timerPeriod;

static float totalMeasureTime;
static float periodTime;

static float tensionBuffer[NB_SAMPLES];
static float currentBuffers[NB_CURRENTS][NB_SAMPLES];

static int iBuffer;

static float periodTimeBuffer[1024];
static uint16_t iPeriodTimeBuffer = 0;

void measureInit()
{
    timerPeriod = TIM_PERIOD / 1000000.;    // seconds
    float freq = 1. / timerPeriod;

    if (freq > MAX_AC_FREQ && (freq < MIN_AC_FREQ)) {
        sendError(INIT_ERROR, freq);
    }
 
    tensionInit(&tension);
    for (uint8_t i = 0; i != NB_CURRENTS; i++) {
        currentInit(&(current[i]));
    }

    //energyCounter = 0;

    zeroIndex = 0.;
    prevZeroIndex = 0.;

    fftRunning = false;

    initState = INIT;

    iBuffer = 0;
}


void measureAdcCallback(uint32_t* data)
{
    periodTimeBuffer[iPeriodTimeBuffer] = periodTime;
    iPeriodTimeBuffer++;

    tension.prevVal = tension.val;
    tension.val = TENSION_COEFF * ((float)data[5] - (float)data[6]);

    float czPoint;
    float deltaT;

    switch(initState) {
        case INIT:
            initState = WAITING_ZC;
            return;
        break;

        case WAITING_ZC:
            if (isCrossingZero(&czPoint)) {
                deltaT = timerPeriod * (1. - czPoint);
                totalMeasureTime = 0.;
                tensionSampleCalc(deltaT, false);
                currentSampleCalc(data, deltaT, false);
                periodTime = deltaT;
                initState = NORMAL_PHASE;
            }
            return;
        break;

        default:
        case NORMAL_PHASE:
        break;
    }

    if (isCrossingZero(&czPoint)) {

        // Robustess check
        if (periodTime < (1. / MAX_AC_FREQ)) {
            sendError(AC_FREQ_ERROR, 1. / periodTime);
        }

        // Calculation of the last point of the previous period
        deltaT = timerPeriod * czPoint;
        tensionSampleCalc(deltaT, true);
        currentSampleCalc(data, deltaT, true);
        
        // update current time with the last step of the previous period
        periodTime += deltaT;

        // Calculation of the complete previous period
        tensionPeriodCalc();
        currentPeriodCalc();

        // add the last period time to the the total Measure Time
        totalMeasureTime += periodTime;
        
        // Calculation of the first point of the new period
        deltaT = timerPeriod * (1. - czPoint);
        tensionSampleCalc(deltaT, false);
        currentSampleCalc(data, deltaT, false);

        // initialize current time with the first step of the new period
        periodTime = deltaT;
        
        // After 5 minutes, send the set of data through the UART and reset the data set
        if (totalMeasureTime > DATA_SENT_PERIOD) {
            sendEnergyData();
            totalMeasureTime = 0.;
        }
    }
    else {
        tensionSampleCalc(timerPeriod, false);
        currentSampleCalc(data, timerPeriod, false);
        periodTime += timerPeriod;
        
        // Storage of the tension and currents val in the buffers
        tensionBuffer[iBuffer] = tension.val;
        for (uint8_t i = 0; i < NB_CURRENTS; i++) {
            currentBuffers[i][iBuffer] = current[i].val;
        }

        iBuffer++;
        if (iBuffer == NB_SAMPLES) {
            iBuffer = 0;
        }



        if (periodTime > (1. / MIN_AC_FREQ)) {
            sendError(AC_FREQ_ERROR, 1. / periodTime);
        }
    }

    return;
}

void tensionSampleCalc(float deltaT, bool lastSample)
{
    if (!lastSample) {
        updateRms(&(tension.rms), tension.val, deltaT);
    }
}

void currentSampleCalc(uint32_t* data, float deltaT, bool lastSample)
{
    for (uint8_t i = 0; i != NB_CURRENTS; i++) {
        current[i].prevVal = current[i].val;
        current[i].val = CURRENT_COEFFS[i] * ((float)data[i] - (float)data[6]);
        float I = current[i].val;

        if (lastSample) {
            // If the calculation is the last sample, then make a linear interpolation to get the current corresponding to deltaT
            I = current[i].prevVal + (current[i].val - current[i].prevVal) * deltaT;
            // For the last sample, energy is not updated since U is considered as nul
        }
        else {
            current[i].energy += tension.val * I;
        }

        updateRms(&(current[i].rms), I, deltaT);
    }
}

void currentPeriodCalc() 
{
    for (uint8_t i = 0; i != NB_CURRENTS; i++) {
        saveRms(&(current[i].rms));
    }
}

void tensionPeriodCalc()
{
    float freq = 1. / periodTime;
    if (freq < tension.freqMin) {
        tension.freqMin = freq;
    }
    if (freq > tension.freqMax) {
        tension.freqMax = freq;
    }
    tension.freqMean = (tension.freqMean * totalMeasureTime + freq * periodTime) / (totalMeasureTime + periodTime);
    saveRms(&(tension.rms));
}


bool isCrossingZero(float* czPoint)
{
    if (tension.val >= 0. && tension.prevVal < 0.) {
        // Linear interpolation to find the exact index where the tension curve is crossing zero
        float a = tension.val - tension.prevVal;
        float b = tension.prevVal;
        *czPoint = -b / a;

        if (*czPoint > 1 || *czPoint < 0) {
            sendError(IMPOSSIBLE_VALUE_ERROR, *czPoint);
        }

        return true;
    }
    else {
        return false;
    }
}


void sendEnergyData()
{
    bool stop = true;
    UNUSED(stop);
}

void tensionInit(Tension* tension)
{
    tension->val = 0.;
    tension->prevVal = 0.;
    rmsInit(&(tension->rms));
    tension->freqMean = 0.;
    tension->freqMin = 999999.;
    tension->freqMax = 0.;
}

void currentInit(Current* current)
{
    current->val = 0.;
    current->prevVal = 0.;
    current->energy = 0.;
    rmsInit(&(current->rms));
}


void rmsInit(Rms* rms)
{
    rms->max = 0.;
    rms->mean = 0.;
    rms->min = 999999.;
}

void saveRms(Rms* rms)
{
    float rmsVal = pow(rms->temp / periodTime, 0.5);
    rms->temp = 0.;

    if (rmsVal < rms->min) {
        rms->min = rmsVal;
    }
    if (rmsVal > rms->max) {
        rms->max = rmsVal;
    }

    rms->mean = (rms->mean * totalMeasureTime + rmsVal * periodTime) / (totalMeasureTime + periodTime);
}

void updateRms(Rms* rms, float val, float deltaT)
{
    rms->temp += pow(val, 2) * deltaT;
}

void sendError(ErrorCode errorCode, float value)
{
    UNUSED(errorCode);
    UNUSED(value);
}
