/* Control of FIR. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"

#include "fir.h"


/* Represents the internal state of a single FIR bank. */
struct fir_bank {
    int index;
    int cycles;
    int length;
    double phase;
    int *current_taps;
    int *set_taps;
    struct epics_record *taps_waveform;
    bool use_waveform;
    bool waveform_set;
};

static int fir_filter_length;
static struct fir_bank banks[FIR_BANKS];

/* Number of bytes in a filter: we use this count quite a bit. */
#define FILTER_SIZE     (fir_filter_length * sizeof(int))


/* Given a ratio cycles:length and a phase compute the appropriate filter. */
static void compute_fir_taps(struct fir_bank *bank)
{
    double tune = (double) bank->cycles / bank->length;

    /* Calculate FIR coeffs and the mean value. */
    int *taps = bank->current_taps;
    int sum = 0;
    double max_int = (1 << 15) - 1;
    for (int i = 0; i < bank->length; i++)
    {
        int tap = (int) round(max_int *
            sin(2*M_PI * (tune * (i+0.5) + bank->phase / 360.0)));
        taps[i] = tap;
        sum += tap;
    }
    /* Pad end of filter with zeros. */
    for (int i = bank->length; i < fir_filter_length; i++)
        taps[i] = 0;

    /* Fixup the DC offset introduced by the residual sum.  Turns out that this
     * is generally quite miniscule (0, 1 or -1), so it doesn't hugely matter
     * where we put it... unless we're really unlucky (eg, tune==1), in which
     * case it really hardly matters! */
    taps[0] -= sum;
}


/* After any update to current_taps ensures that the hardware and readback
 * record are in step. */
static void update_taps(struct fir_bank *bank)
{
    hw_write_fir_taps(bank->index, bank->current_taps);
    trigger_record(bank->taps_waveform, 0, NULL);
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

/* This is called when the waveform TAPS_S is updated.  If we're using the
 * waveform settings then the given waveform is written to hardware, otherwise
 * we just hang onto it. */
static void set_fir_taps(void *context, int *taps, size_t *length)
{
    struct fir_bank *bank = context;
    memcpy(bank->set_taps, taps, FILTER_SIZE);
    if (bank->use_waveform)
    {
        memcpy(bank->current_taps, bank->set_taps, FILTER_SIZE);
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
            memcpy(bank->current_taps, bank->set_taps, FILTER_SIZE);
        else
            compute_fir_taps(bank);
        update_taps(bank);
    }
    return true;
}


static void publish_bank(int ix, struct fir_bank *bank)
{
    bank->index = ix;
    bank->current_taps = malloc(FILTER_SIZE);
    bank->set_taps     = malloc(FILTER_SIZE);

    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "FIR:%d:%s", ix, name), buffer)

    /* This is triggered when any coefficient is changed. */
    PUBLISH(bo, FORMAT("RELOAD"), reload_fir, .context = bank);

    /* Selects whether to use waveform or parameters for FIR taps. */
    PUBLISH(bo, FORMAT("USEWF"),
        set_use_waveform, .context = bank, .persist = true);

    /* These writes all forward link to reload_fir above. */
    PUBLISH_WRITE_VAR_P(longout, FORMAT("LENGTH"), bank->length);
    PUBLISH_WRITE_VAR_P(longout, FORMAT("CYCLES"), bank->cycles);
    PUBLISH_WRITE_VAR_P(ao, FORMAT("PHASE"), bank->phase);

    PUBLISH_WAVEFORM(
        int, FORMAT("TAPS_S"), fir_filter_length, set_fir_taps,
        .context = bank, .persist = true);
    bank->taps_waveform = PUBLISH_WF_READ_VAR_I(
        int, FORMAT("TAPS"), fir_filter_length, bank->current_taps);
#undef FORMAT
};



bool initialise_fir(void)
{
    fir_filter_length = hw_read_fir_length();
    PUBLISH_WRITER_P(mbbo, "FIR:GAIN", hw_write_fir_gain);
    for (int i = 0; i < FIR_BANKS; i ++)
        publish_bank(i, &banks[i]);
    return true;
}
