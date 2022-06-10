#include <Arduino.h>

#include <driver/ledc.h>

#include <esp_check.h>
#include <soc/clk_ctrl_os.h>
#include <soc/ledc_struct.h>

// copied and modified the below code from ESP EDF source code in order to fix ledc_set_freq function
// to not reset timer after parameter update which causes glitches in pwm output

#define LEDC_TIMER_DIV_NUM_MAX (0x3FFFF)

#define LEDC_CHECK(a, str, ret_val) ESP_RETURN_ON_FALSE(a, ret_val, LEDC_TAG, "%s", str)
#define LEDC_ARG_CHECK(a, param) ESP_RETURN_ON_FALSE(a, ESP_ERR_INVALID_ARG, LEDC_TAG, param " argument is invalid")

#define ledc_hal_get_duty_resolution(hal, timer_sel, duty_resolution) ledc_ll_get_duty_resolution((hal)->dev, (hal)->speed_mode, timer_sel, duty_resolution)
static inline void ledc_ll_get_duty_resolution(ledc_dev_t *hw, ledc_mode_t speed_mode, ledc_timer_t timer_sel, uint32_t *duty_resolution)
{
    *duty_resolution = hw->timer_group[speed_mode].timer[timer_sel].conf.duty_resolution;
}

#define ledc_hal_get_clock_source(hal, timer_sel, clk_src) ledc_ll_get_clock_source((hal)->dev, (hal)->speed_mode, timer_sel, clk_src)
static inline void ledc_ll_get_clock_source(ledc_dev_t *hw, ledc_mode_t speed_mode, ledc_timer_t timer_sel, ledc_clk_src_t *clk_src)
{
    if (hw->timer_group[speed_mode].timer[timer_sel].conf.tick_sel)
    {
        *clk_src = LEDC_APB_CLK;
    }
    else
    {
        *clk_src = LEDC_REF_TICK;
    }
}

typedef struct
{
    ledc_dev_t *dev;
    ledc_mode_t speed_mode;
} ledc_hal_context_t;

typedef struct
{
    ledc_hal_context_t ledc_hal; /*!< LEDC hal context*/
} ledc_obj_t;

static const char *LEDC_TAG = "ledc";
static const char *LEDC_NOT_INIT = "LEDC is not initialized";
static const char *LEDC_SLOW_CLK_INVALID = "Slow clock not supported";

static ledc_obj_t *p_ledc_obj[LEDC_SPEED_MODE_MAX] = {0};

static uint32_t s_ledc_slow_clk_8M = 0;

static esp_err_t ledc_hal_get_clk_cfg(ledc_hal_context_t *hal, ledc_timer_t timer_sel, ledc_clk_cfg_t *clk_cfg)
{
    ledc_clk_src_t clk_src = LEDC_APB_CLK;
    ledc_hal_get_clock_source(hal, timer_sel, &clk_src);
    if (clk_src == LEDC_REF_TICK)
    {
        *clk_cfg = LEDC_USE_REF_TICK;
    }
    else
    {
        *clk_cfg = LEDC_USE_APB_CLK;
        if (hal->speed_mode == LEDC_LOW_SPEED_MODE)
        {
            LEDC_CHECK(0, LEDC_SLOW_CLK_INVALID, ESP_ERR_INVALID_STATE);
        }
    }
    return ESP_OK;
}

static uint32_t ledc_get_src_clk_freq(ledc_clk_cfg_t clk_cfg)
{
    uint32_t src_clk_freq = 0;
    if (clk_cfg == LEDC_USE_APB_CLK)
    {
        src_clk_freq = LEDC_APB_CLK_HZ;
    }
    else if (clk_cfg == LEDC_USE_REF_TICK)
    {
        src_clk_freq = LEDC_REF_CLK_HZ;
    }
    else if (clk_cfg == LEDC_USE_RTC8M_CLK)
    {
        src_clk_freq = s_ledc_slow_clk_8M;
#if SOC_LEDC_SUPPORT_XTAL_CLOCK
    }
    else if (clk_cfg == LEDC_USE_XTAL_CLK)
    {
        src_clk_freq = rtc_clk_xtal_freq_get() * 1000000;
#endif
    }
    return src_clk_freq;
}

esp_err_t ledc_set_freq_no_reset(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t freq_hz)
{
    if (p_ledc_obj[speed_mode] == NULL)
    {
        p_ledc_obj[speed_mode] = (ledc_obj_t *)heap_caps_calloc(1, sizeof(ledc_obj_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (p_ledc_obj[speed_mode] == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
        p_ledc_obj[speed_mode]->ledc_hal.dev = &LEDC;
        p_ledc_obj[speed_mode]->ledc_hal.speed_mode = speed_mode;
    }

    LEDC_ARG_CHECK(speed_mode < LEDC_SPEED_MODE_MAX, "speed_mode");
    LEDC_ARG_CHECK(timer_num < LEDC_TIMER_MAX, "timer_num");
    LEDC_CHECK(p_ledc_obj[speed_mode] != NULL, LEDC_NOT_INIT, ESP_ERR_INVALID_STATE);

    ledc_clk_cfg_t clk_cfg = LEDC_USE_APB_CLK;
    uint32_t duty_resolution = 0;
    ledc_hal_get_clk_cfg(&(p_ledc_obj[speed_mode]->ledc_hal), timer_num, &clk_cfg);
    ledc_hal_get_duty_resolution(&(p_ledc_obj[speed_mode]->ledc_hal), timer_num, &duty_resolution);

    uint32_t div_param = 0;
    uint32_t precision = (0x1 << duty_resolution);
    ledc_clk_src_t timer_clk_src = LEDC_APB_CLK;

    // Calculate the divisor
    // User specified source clock(RTC8M_CLK) for low speed channel
    if ((speed_mode == LEDC_LOW_SPEED_MODE) && (clk_cfg == LEDC_USE_RTC8M_CLK))
    {
        LEDC_CHECK(0, LEDC_SLOW_CLK_INVALID, ESP_ERR_INVALID_STATE);
    }
    else
    {
        // Automatically select APB or REF_TICK as the source clock.
        if (clk_cfg == LEDC_AUTO_CLK)
        {
            // Try calculating divisor based on LEDC_APB_CLK
            div_param = ((uint64_t)LEDC_APB_CLK_HZ << 8) / freq_hz / precision;
            if (div_param > LEDC_TIMER_DIV_NUM_MAX)
            {
                // APB_CLK results in divisor which too high. Try using REF_TICK as clock source.
                timer_clk_src = LEDC_REF_TICK;
                div_param = ((uint64_t)LEDC_REF_CLK_HZ << 8) / freq_hz / precision;
            }
            else if (div_param < 256)
            {
                // divisor is too low
                goto error;
            }
            // User specified source clock(LEDC_APB_CLK_HZ or LEDC_REF_TICK)
        }
        else
        {
            timer_clk_src = (clk_cfg == LEDC_USE_REF_TICK) ? LEDC_REF_TICK : LEDC_APB_CLK;
            uint32_t src_clk_freq = ledc_get_src_clk_freq(clk_cfg);
            div_param = ((uint64_t)src_clk_freq << 8) / freq_hz / precision;
        }
    }
    if (div_param < 256 || div_param > LEDC_TIMER_DIV_NUM_MAX)
    {
        goto error;
    }
    if (speed_mode == LEDC_LOW_SPEED_MODE)
    {
        LEDC_CHECK(0, LEDC_SLOW_CLK_INVALID, ESP_ERR_INVALID_STATE);
    }
    // Set the divisor
    ledc_timer_set(speed_mode, timer_num, div_param, duty_resolution, timer_clk_src);
    // reset the timer
    // ledc_timer_rst(speed_mode, timer_num);  // NO DONT - IT CAUSES GLITCHES
    return ESP_OK;
error:
    ESP_LOGE(LEDC_TAG, "requested frequency and duty resolution can not be achieved, try reducing freq_hz or duty_resolution. div_param=%d",
             (uint32_t)div_param);
    return ESP_FAIL;
}
