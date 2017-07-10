/* Control of FIR. */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"
#include "tmbf.h"

#include "fir.h"


/* Represents the internal state of a single FIR bank. */
struct fir_bank {
    unsigned int index;
    unsigned int cycles;
    unsigned int length;
    double phase;
    float *current_taps;
    float *set_taps;
    struct epics_record *taps_waveform;
    bool use_waveform;
    bool waveform_set;
};

static size_t fir_filter_length;
static struct fir_bank banks[FIR_BANKS];


/* Given a ratio cycles:length and a phase compute the appropriate filter. */
static void compute_fir_taps(struct fir_bank *bank)
{
    double tune = (double) bank->cycles / bank->length;

    /* Calculate FIR coeffs and the mean value. */
    float *taps = bank->current_taps;
    for (unsigned int i = 0; i < bank->length; i++)
        taps[i] = (float) sin(2*M_PI * (tune * (i+0.5) + bank->phase / 360.0));
    /* Pad end of filter with zeros. */
    for (size_t i = bank->length; i < fir_filter_length; i++)
        taps[i] = 0;
}


/* After any update to current_taps ensures that the hardware and readback
 * record are in step. */
static void update_taps(struct fir_bank *bank)
{
    int taps[fir_filter_length];
    float_array_to_int(fir_filter_length, bank->current_taps, taps, 16, 0);
    hw_write_fir_taps(bank->index, taps);
    trigger_record(bank->taps_waveform);
}


/* This is called any time any of the FIR control parameters have changed.
 * Recompute and reload the FIR taps as appropriate.  No effect if not in
 * use_waveform mode. */
static bool reload_fir(void *context, const bool *value)
{
    struct fir_bank *bank = context;
    if (!bank->use_waveform)
    {
        compute_fir_taps(bank);
        update_taps(bank);
    }
    return true;
}

static void copy_taps(const float taps_in[], float taps_out[])
{
    memcpy(taps_out, taps_in, fir_filter_length * sizeof(float));
}

/* This is called when the waveform TAPS_S is updated.  If we're using the
 * waveform settings then the given waveform is written to hardware, otherwise
 * we just hang onto it. */
static void set_fir_taps(void *context, float *taps, size_t *length)
{
    struct fir_bank *bank = context;
    copy_taps(taps, bank->set_taps);
    if (bank->use_waveform)
    {
        copy_taps(bank->set_taps, bank->current_taps);
        update_taps(bank);
    }
    *length = fir_filter_length;
}


/* Called to switch between waveform and settings mode. */
static bool set_use_waveform(void *context, const bool *use_waveform)
{
    struct fir_bank *bank = context;
    if (*use_waveform != bank->use_waveform)
    {
        bank->use_waveform = *use_waveform;
        if (bank->use_waveform)
            copy_taps(bank->set_taps, bank->current_taps);
        else
            compute_fir_taps(bank);
        update_taps(bank);
    }
    return true;
}


static void publish_bank(unsigned int ix, struct fir_bank *bank)
{
    bank->index = ix;
    bank->current_taps = calloc(fir_filter_length, sizeof(float));
    bank->set_taps     = calloc(fir_filter_length, sizeof(float));

    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "FIR:%d:%s", ix, name), buffer)

    /* This is triggered when any coefficient is changed. */
    PUBLISH(bo, FORMAT("RELOAD"), reload_fir, .context = bank);

    /* Selects whether to use waveform or parameters for FIR taps. */
    PUBLISH(bo, FORMAT("USEWF"),
        set_use_waveform, .context = bank, .persist = true);

    /* These writes all forward link to reload_fir above. */
    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("LENGTH"), bank->length);
    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("CYCLES"), bank->cycles);
    PUBLISH_WRITE_VAR_P(ao, FORMAT("PHASE"), bank->phase);

    PUBLISH_WAVEFORM(
        float, FORMAT("TAPS_S"), fir_filter_length, set_fir_taps,
        .context = bank, .persist = true);
    bank->taps_waveform = PUBLISH_WF_READ_VAR_I(
        float, FORMAT("TAPS"), fir_filter_length, bank->current_taps);
#undef FORMAT
};



bool initialise_fir(void)
{
    fir_filter_length = hw_read_fir_length();
    PUBLISH_WRITER_P(mbbo, "FIR:GAIN", hw_write_fir_gain);
    PUBLISH_WRITER_P(ulongout, "FIR:DECIMATION", hw_write_fir_decimation);
    PUBLISH_WRITER_P(mbbo, "FIR:DECIMATE:GAIN", hw_write_fir_decimation_gain);
    for (unsigned int i = 0; i < FIR_BANKS; i ++)
        publish_bank(i, &banks[i]);
    return true;
}
