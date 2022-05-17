#include <Arduino.h>


#include "config.h"
#include "utils.h"
#include "ledc.h"
#include "timer.h"
#include "sampler.h"
#include "control.h"


static TaskHandle_t DRAM_ATTR __control_task;
static control_mode_t DRAM_ATTR __cur_mode;
static volatile control_mode_t DRAM_ATTR __set_mode;


control_mode_t control_get_mode(void)
{
    return __cur_mode;
}

static void __fft_callback(struct fft_analysis_t *analysis)
{
  LOG("\n==========================================================\n");
  LOG("min_smpl: %d\n", analysis->min_smpl);
  LOG("max_smpl: %d\n", analysis->max_smpl);
  LOG("avg_smpl: %f\n", analysis->avg_smpl);
  LOG("energy: %f\n", analysis->energy);
  LOG("max_bin_freq: %f\n", analysis->max_bin_freq);
  LOG("max_est_freq: %f\n", analysis->max_est_freq);
  LOG("\n==========================================================\n");
}

void control_init(void)
{
  sampler_init(fft_callback);
  sampler_start();

    __proc_flag.store(0);
    __enable_flag.store(0);
    
    // sampler task will run on the 2nd core
    ret = xTaskCreatePinnedToCore(TASK_sampler_main, "CONTROL", FFT_TASK_STACK_SIZE, NULL, 1, &__sampler_task, FFT_TASK_CORE);

    BUG(ret != pdPASS);
}
}