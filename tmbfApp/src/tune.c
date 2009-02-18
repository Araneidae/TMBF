/* Tune measurement. */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>

#include "hardware.h"
#include "test_error.h"
#include "epics_device.h"

#include "tune.h"



#define TUNE_LENGTH     (MAX_DATA_LENGTH/4)


#define SQR(x)  ((x)*(x))


/* Computes MulSS(x,y) = 2^-32 * x * y in a single fast instruction. */
inline int MulSS(int x, int y)
{
    unsigned int result, temp; 
    __asm__("smull   %1, %0, %2, %3" :
        "=&r"(result), "=&r"(temp) : "r"(x), "r"(y)); 
    return result; 
}




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            Frequency Scale                                */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Display scale for tune response, also used for computing tune. */
static float ScaleWaveform[TUNE_LENGTH];

/* Waveforms for compensating IQ by DDC_skew.
 * 
 * The DDC skew is a group delay in ns.  We translate this into a phase
 * advance of
 *
 *      phase = 0.5 * skew * 2pi * frequency / 936
 *
 * Here the frequency is in tunes, one revolution per turn, which we convert
 * into one revolution per bunch, or half a revolution per nanosecond (at
 * 500MHz RF frequency).
 *    For economy of calculation, we precompute the rotations as scaled
 * integers so that the final computation can be a simple integer
 * multiplication (one instruction when the compiler is in the right mood). */
static int DDC_skew = 0;
static int rotate_I[TUNE_LENGTH];  // 2**30 * cos(phase)
static int rotate_Q[TUNE_LENGTH];  // 2**30 * sin(phase)


static unsigned int tune_to_freq(double tune)
{
    return (unsigned int) round(pow(2,33)*tune/936.0); 
}

static double freq_to_tune(unsigned int freq)
{
    return 936.0 * freq / pow(2, 33);
}

/* Set NCO */
static void set_homfreq (double new_freq)
{
    write_NCO_frequency(tune_to_freq(new_freq));
}

/* Set Sweep Start Freq */
static void set_sweepstartfreq (double new_freq)
{
    write_SweepStartFreq(tune_to_freq(new_freq));
}  

/* Set Sweep Stop Freq */
static void set_sweepstopfreq (double new_freq)
{
    write_SweepStopFreq(tune_to_freq(new_freq));
}

static void set_freq_step(double new_freq)
{
    write_SweepStep(tune_to_freq(new_freq));
}



static void update_tune_scale()
{
    double start = freq_to_tune(read_SweepStartFreq());
    double stop  = freq_to_tune(read_SweepStopFreq());
    double step  = freq_to_tune(read_SweepStep());

    double phase_scale = - DDC_skew / 936.0 / 2.0;
    double two30 = (double) (1 << 30);
    for (int i = 0; i < TUNE_LENGTH; i++)
    {
        ScaleWaveform[i] = (float) (start + i * step);
        if (ScaleWaveform[i] > stop)
            ScaleWaveform[i] = stop;

        double cycle;
        double phase = 2 * M_PI * modf(
            phase_scale * ScaleWaveform[i], &cycle);
        rotate_I[i] = (int) round(two30 * cos(phase));
        rotate_Q[i] = (int) round(two30 * sin(phase));
    }
}

static void set_tune_scale(float * buffer)
{
    update_tune_scale();
    memcpy(buffer, ScaleWaveform, sizeof(ScaleWaveform));
}

static void update_ddc_skew(int new_skew)
{
    DDC_skew = new_skew;
    update_tune_scale();
}



PUBLISH_SIMPLE_WRITE(ao, "HOMFREQ_S", set_homfreq)

PUBLISH_SIMPLE_WRITE(ao, "SWPSTARTFREQ_S", set_sweepstartfreq)
PUBLISH_SIMPLE_WRITE(ao, "SWPSTOPFREQ_S", set_sweepstopfreq)
PUBLISH_SIMPLE_WRITE(ao, "SWPFREQSTEP_S", set_freq_step)
PUBLISH_SIMPLE_WAVEFORM(float, "TUNESCALE", TUNE_LENGTH, set_tune_scale)

PUBLISH_SIMPLE_WRITE(longout, "DDCSKEW_S", update_ddc_skew)




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            Tune Measurement                               */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* I & Q readings reduced from raw buffer and skew compensated. */
static int buffer_I[TUNE_LENGTH];
static int buffer_Q[TUNE_LENGTH];

static int tune_power[TUNE_LENGTH];
static double peak_power_tune;
static double peak_power_phase;

static int raw_cumsum_I[TUNE_LENGTH];
static int raw_cumsum_Q[TUNE_LENGTH];
static int cumsum_I[TUNE_LENGTH];
static int cumsum_Q[TUNE_LENGTH];
static double cumsum_tune;
static double cumsum_phase;





static void update_IQ()
{
    short int hb_buf_lower[MAX_DATA_LENGTH];
    short int hb_buf_upper[MAX_DATA_LENGTH];
    read_DataSpace(hb_buf_lower, hb_buf_upper);
    
    for (int i = 0; i < TUNE_LENGTH; i ++)
    {
        int I =
            hb_buf_lower[4*i + 0] + hb_buf_lower[4*i + 1] + 
            hb_buf_lower[4*i + 2] + hb_buf_lower[4*i + 3];
        int Q =
            hb_buf_upper[4*i + 0] + hb_buf_upper[4*i + 1] + 
            hb_buf_upper[4*i + 2] + hb_buf_upper[4*i + 3];
        int rI = rotate_I[i];
        int rQ = rotate_Q[i];
        
        buffer_I[i] = MulSS(I, rI) - MulSS(Q, rQ);
        buffer_Q[i] = MulSS(I, rQ) + MulSS(Q, rI);
    }
}


static void compute_power_tune()
{
    /* Convert the extracted IQ waveform into a power spectrum.  After
     * accumulation and rotation, the I and Q values are signed 16 bit values
     * again, so can use them as they are. */
    for (int i = 0; i < TUNE_LENGTH; i ++)
        tune_power[i] = SQR(buffer_I[i]) + SQR(buffer_Q[i]);
    
    /* Find the index of the peak power. */
    int PeakValue = 0;
    int PowerPeak = 0;
    for (int i = 0; i < TUNE_LENGTH; i ++)
    {
        if (tune_power[i] > PeakValue)
        {
            PeakValue = tune_power[i];
            PowerPeak = i;
        }
    }

    double harmonic;
    peak_power_tune = modf(ScaleWaveform[PowerPeak], &harmonic);
    peak_power_phase = 180.0 / M_PI *
        atan2(-buffer_Q[PowerPeak], -buffer_I[PowerPeak]);

}



/* The given difference is accumulated over the waveform. */
static void accumulate_difference(int difference, int length, int waveform[])
{
    div_t d = div(difference, length);
    int residue = 0;
    int correction = 0;
    int step = difference >= 0 ? 1 : -1;
    int fixup = difference >= 0 ? length : -length;
    for (int i = 0; i < length; i ++)
    {
        correction += d.quot;
        residue += d.rem;
        if (abs(residue) >= length)
        {
            residue -= fixup;
            correction += step;
        }
        waveform[i] -= correction;
    }
}


static void update_cumsum_IQ()
{
    /* First compute the raw cumulative sum. */
    int sum_i = 0;
    int sum_q = 0;
    for (int i = 0; i < TUNE_LENGTH; i ++)
    {
        sum_i += buffer_I[i];
        raw_cumsum_I[i] = sum_i;
        sum_q += buffer_Q[i];
        raw_cumsum_Q[i] = sum_q;
    }

    /* Compute the corrected cumulative sum integrating to zero.  Although
     * not strictly correct, this produces a better result. */
    memcpy(cumsum_I, raw_cumsum_I, sizeof(cumsum_I));
    accumulate_difference(sum_i, TUNE_LENGTH, cumsum_I);
    memcpy(cumsum_Q, raw_cumsum_Q, sizeof(cumsum_Q));
    accumulate_difference(sum_q, TUNE_LENGTH, cumsum_Q);

    /* Perform peak search across the cumulative sum. */
    int cumsum_peak = 0;
    long long int PeakValue = 0;
    for (int i = 0; i < TUNE_LENGTH; i ++)
    {
        long long int value =
            SQR((long long int) cumsum_I[i]) +
            SQR((long long int) cumsum_Q[i]);
        if (value > PeakValue)
        {
            PeakValue = value;
            cumsum_peak = i;
        }
    }

    /* Compute the tune and phase from the maximum magnitude. */
    double harmonic;
    cumsum_tune = modf(ScaleWaveform[cumsum_peak], &harmonic);
    cumsum_phase = 180.0 / M_PI *
        atan2(-buffer_Q[cumsum_peak], -buffer_I[cumsum_peak]);
}



/* This is called each time a tune measurement needs to be done. */

static void process_tune()
{
    /* First capture the IQ readings from the last tune scan. */
    update_IQ();
    
    /* Compute the tune from the peak power. */
    compute_power_tune();

    /* Similarly compute the tune from the cumulative sum. */
    update_cumsum_IQ();
}




/* The following records are all updated as part of the tune scan. */

PUBLISH_METHOD("PROCESS_TUNE", process_tune)

PUBLISH_READ_WAVEFORM(int, "DDC_I", TUNE_LENGTH, buffer_I)
PUBLISH_READ_WAVEFORM(int, "DDC_Q", TUNE_LENGTH, buffer_Q)
PUBLISH_READ_WAVEFORM(int, "TUNEPOWER", TUNE_LENGTH, tune_power)
PUBLISH_VARIABLE_READ(ai, "TUNE", peak_power_tune)
PUBLISH_VARIABLE_READ(ai, "TUNEPHASE", peak_power_phase)

PUBLISH_READ_WAVEFORM(int, "RAWCUMSUM_I", TUNE_LENGTH, raw_cumsum_I)
PUBLISH_READ_WAVEFORM(int, "RAWCUMSUM_Q", TUNE_LENGTH, raw_cumsum_Q)
PUBLISH_READ_WAVEFORM(int, "CUMSUM_I", TUNE_LENGTH, cumsum_I)
PUBLISH_READ_WAVEFORM(int, "CUMSUM_Q", TUNE_LENGTH, cumsum_Q)
PUBLISH_VARIABLE_READ(ai, "CUMSUMTUNE", cumsum_tune)
PUBLISH_VARIABLE_READ(ai, "CUMSUMPHASE", cumsum_phase)




#ifndef __DEFINE_EPICS__
#include "tune.EPICS"
#endif

bool InitialiseTune()
{
    return PUBLISH_EPICS_DATA();
}
