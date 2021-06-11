#ifdef ESP32

#include "MakerMatty_TouchPin.h"

#include "esp_attr.h"
#include "esp_intr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"

///////////////////////////////////////////////////////////////////////////////////////////////

TouchPinRaw::voidFuncPtr TouchPinRaw::m_touchInterruptHandlers[TOUCH_PAD_MAX]({
    NULL,
});
intr_handle_t TouchPinRaw::m_touchInterruptHandle(NULL);
bool TouchPinRaw::m_initialized(false);

void IRAM_ATTR TouchPinRaw::touchHandler(void* arg)
{
    // uint32_t pad_intr = READ_PERI_REG(SENS_SAR_TOUCH_CTRL2_REG) & 0x3ff;
    // uint32_t rtc_intr = READ_PERI_REG(RTC_CNTL_INT_ST_REG);
    // uint8_t i = 0;
    // //clear interrupt
    // WRITE_PERI_REG(RTC_CNTL_INT_CLR_REG, rtc_intr);
    // SET_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_MEAS_EN_CLR);

    // if (rtc_intr & RTC_CNTL_TOUCH_INT_ST) {
    //     for (i = 0; i < 10; ++i) {
    //         if ((pad_intr >> i) & 0x01) {
    //             if(m_touchInterruptHandlers[i]){
    //                 m_touchInterruptHandlers[i]();
    //             }
    //         }
    //     }
    // }

    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();
    for (int i = 0; i < TOUCH_PAD_MAX; i++) {
        if ((pad_intr >> i) & 0x01) {
            if (m_touchInterruptHandlers[i]) {
                m_touchInterruptHandlers[i]();
            }
        }
    }
}

TouchPinRaw::TouchPinRaw(touch_pad_t pad)
    : m_value(0)
    , m_pad(pad)
{
    if (!m_initialized) {
        m_initialized = true;

        // Initialize touch pad peripheral.
        // The default fsm mode is software trigger mode.
        touch_pad_init();

        // Set reference voltage for charging/discharging
        // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
        // The low reference voltage will be 0.5
        // The larger the range, the larger the pulse count value.
        touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

        // [default] sleep_cycle = 4096, meas_cycle = 32767
        touch_pad_set_meas_time(0, 1024);

        touch_pad_intr_enable();
        touch_pad_isr_register(touchHandler, nullptr);
    }
    
    touch_pad_io_init(m_pad);

    touch_pad_config(m_pad, 0); // set initial treshold for triggering interrupts to 0

    // [default] slope = TOUCH_PAD_SLOPE_4, opt = TOUCH_PAD_TIE_OPT_LOW
    touch_pad_set_cnt_mode(m_pad, TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_HIGH);
}

uint8_t TouchPinRaw::readRaw8()
{
    touch_pad_read(m_pad, &m_value);
    return m_value > 255 ? 255 : m_value;
}

uint16_t TouchPinRaw::readRaw()
{
    touch_pad_read(m_pad, &m_value);
    return m_value;
}

void TouchPinRaw::info()
{
    touch_cnt_slope_t slope;
    touch_tie_opt_t opt;
    touch_pad_get_cnt_mode(m_pad, &slope, &opt);
    Serial.printf("[touch_pad %u] slope = %u, opt = %u\n", (uint8_t)m_pad, (uint8_t)slope, (uint8_t)opt);

    uint16_t sleep_cycle;
    uint16_t meas_cycle;
    touch_pad_get_meas_time(&sleep_cycle, &meas_cycle);
    Serial.printf("[touch_pad %u] sleep_cycle = %u, meas_cycle = %u\n", (uint8_t)m_pad, sleep_cycle, meas_cycle);
}

touch_pad_t TouchPinRaw::getPin()
{
    return m_pad;
}

void TouchPinRaw::attachInterrupt(isr , uint16_t treshold)
{

    touch_pad_clear_status();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);

    
  


    __touchInterruptHandlers[pad] = userFunc;

    //clear touch force ,select the Touch mode is Timer
    CLEAR_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN_M|SENS_TOUCH_START_FORCE_M);

    //interrupt when touch value < threshold
    CLEAR_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_OUT_SEL);
    //Intr will give ,when SET0 < threshold
    SET_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_OUT_1EN);
    //Enable Rtc Touch Module Intr,the Interrupt need Rtc out  Enable
    SET_PERI_REG_MASK(RTC_CNTL_INT_ENA_REG, RTC_CNTL_TOUCH_INT_ENA);

    //set threshold
    uint8_t shift = (pad & 1) ? SENS_TOUCH_OUT_TH1_S : SENS_TOUCH_OUT_TH0_S;
    SET_PERI_REG_BITS((SENS_SAR_TOUCH_THRES1_REG + (pad / 2) * 4), SENS_TOUCH_OUT_TH0, threshold, shift);

    uint32_t rtc_tio_reg = RTC_IO_TOUCH_PAD0_REG + pad * 4;
    WRITE_PERI_REG(rtc_tio_reg, (READ_PERI_REG(rtc_tio_reg)
                      & ~(RTC_IO_TOUCH_PAD0_DAC_M))
                      | (7 << RTC_IO_TOUCH_PAD0_DAC_S)//Touch Set Slope
                      | RTC_IO_TOUCH_PAD0_TIE_OPT_M   //Enable Tie,Init Level
                      | RTC_IO_TOUCH_PAD0_START_M     //Enable Touch Pad IO
                      | RTC_IO_TOUCH_PAD0_XPD_M);     //Enable Touch Pad Power on

    //Enable Digital rtc control :work mode and out mode
    SET_PERI_REG_MASK(SENS_SAR_TOUCH_ENABLE_REG,
        (1 << (pad + SENS_TOUCH_PAD_WORKEN_S)) | (1 << (pad + SENS_TOUCH_PAD_OUTEN2_S)) | (1 << (pad + SENS_TOUCH_PAD_OUTEN1_S)));
}



///////////////////////////////////////////////////////////////////////////////////////////////

TouchPin::TouchPin(touch_pad_t pad, const uint16_t tap_ms, const uint16_t press_ms)
    : TouchPinRaw(pad)
    , counter(0)
    , maximum(0)
    , treshold(0)
    , current(0)
    , tap_us(tap_ms * 1000)
    , press_us(press_ms * 1000)
    , history { .value = 0 }
    , readings { .value = 0 }
    , updated_micros(0)
    , contact_micros(0)
    , contact(false)
    , contact_(false)
    , contact__(false)
    , release(false)
    , touch(false)
    , tap(false)
    , press(false)
    , press_(false)
{
}

uint8_t TouchPin::update(bool force_update)
{
    int64_t current_time = esp_timer_get_time();
    uint16_t cycles_delta = (uint16_t)((current_time >> 14) - (updated_micros >> 14)); // (x >> 14) == (x / 16384)
    uint8_t reading = readRaw8();

    readings.value = (readings.value << 8) | reading;

    if ((readings.bytes[3] > readings.bytes[2] && readings.bytes[2] > readings.bytes[1] && readings.bytes[1] > readings.bytes[0])
        || (readings.bytes[3] <= readings.bytes[2] && readings.bytes[2] <= readings.bytes[1] && readings.bytes[1] <= readings.bytes[0])) {
        current = readings.bytes[0];
    } else {
        current = history.bytes[0];
    }

    if (cycles_delta > 0) {
        history.value = (history.value << 8) | current;

        if ((counter += cycles_delta) >= 0xFF) {
            maximum -= (counter / 0xFF);
            counter %= 0xFF;
        }

        if (current > maximum) {
            maximum = current;
        }
    }

    uint8_t delta = maximum - current;

    //if delta is bigger than maximum / 16 (7% of maximum), then it's touching and
    //if the value is under treshold and also dropped quickly (in 8 cycles)
    if (delta > (maximum >> 3) + 1) {
        if (((int16_t)history.bytes[7] - (int16_t)history.bytes[0]) > (int16_t)(maximum >> 3)) {
            contact_ = true;
        }
    } else {
        if (delta <= (maximum >> 4)) {
            contact_ = false;
        }
    }

    if (contact_ && !contact__) {
        contact__ = true;
        contact_micros = current_time;

        press_ = false;

        contact = true;
    }

    if (!contact_ && contact__) {
        contact__ = false;
        contact_micros = current_time - contact_micros;

        touch = true;

        release = true;

        if (contact_micros >= tap_us && contact_micros < press_us) {
            tap = true;
        }
    }

    if (contact__ && !press_ && current_time - contact_micros >= press_us) {
        press_ = true;

        press = true;
    }

    updated_micros = current_time;

    return reading;
}

uint8_t TouchPin::getValue()
{
    return current;
}

uint8_t TouchPin::getMax()
{
    return maximum;
}

uint32_t TouchPin::touching()
{
    if (contact_) {
        return (uint32_t)((updated_micros - contact_micros) / (int64_t)1000);
    } else {
        return 0;
    }
}

uint32_t TouchPin::touched()
{
    if (touch) {
        touch = false;
        return (uint32_t)(contact_micros / (int64_t)1000);
    } else {
        return 0;
    }
}

bool TouchPin::contacted()
{
    bool ret = contact;
    contact = false;
    return ret;
}

bool TouchPin::released()
{
    bool ret = release;
    release = false;
    return ret;
}

bool TouchPin::tapped()
{
    bool ret = tap;
    tap = false;
    return ret;
}

bool TouchPin::pressed()
{
    bool ret = press;
    press = false;
    return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////

#endif