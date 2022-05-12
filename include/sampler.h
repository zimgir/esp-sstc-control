#ifndef __SAMPLER_H__
#define __SAMPLER_H__

struct fft_analysis_t
{
    uint16_t min_smpl;
    uint16_t max_smpl;
    float avg_smpl;
    float energy;
    float min_mag;
    float max_mag;
    float avg_mag;
    float max_bin_freq;
    float max_est_freq;
};

typedef void (*fft_callback_t)(struct fft_analysis_t *);
 
void sampler_init(fft_callback_t callback);
void sampler_start(void);
void sampler_stop(void);

#endif