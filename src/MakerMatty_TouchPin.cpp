#ifdef ESP32

#include "MakerMatty_TouchPin.h"

///////////////////////////////////////////////////////////////////////////////////////////////

uint32_t TouchPinRaw::m_initializedFlags(0x00);

TouchPinRaw::TouchPinRaw(touch_pad_t pad)
    : m_value(0)
    , m_pad(pad)
{
    log_v("Contructing TouchPinRaw");

    if (m_pad >= TOUCH_PAD_MAX) {
        log_e("Invalid pad number");
        return;
    }

    if (m_initializedFlags & (1 << m_pad)) {
        log_e("Pad is already active. This will cause problems");
        return;
    }

    if (!m_initializedFlags) {
        log_d("installing TouchPin driver");
        // Initialize touch pad peripheral.
        // The default fsm mode is software trigger mode.
        touch_pad_init();

        // Set reference voltage for charging/discharging
        // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
        // The low reference voltage will be 0.5
        // The larger the range, the larger the pulse count value.
        touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

        // [default] sleep_cycle = 4096, meas_cycle = 32767
        touch_pad_set_meas_time(0, 2048);
    }

    // init touch pad
    touch_pad_io_init(m_pad);
    m_initializedFlags |= (1 << m_pad);

    // [default] slope = TOUCH_PAD_SLOPE_4, opt = TOUCH_PAD_TIE_OPT_LOW
    touch_pad_set_cnt_mode(m_pad, TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_HIGH);
}

TouchPinRaw::TouchPinRaw(TouchPinRaw&& other)
{
    m_pad = other.m_pad;
    other.m_pad = TOUCH_PAD_MAX;
    m_value = other.m_value;
    other.m_value = 0;
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

touch_pad_t TouchPinRaw::getPad()
{
    return m_pad;
}

TouchPinRaw::~TouchPinRaw()
{
    log_v("Destroying TouchPinRaw");

    if (m_pad < TOUCH_PAD_MAX) {
        if (!(m_initializedFlags & (1 << m_pad))) {
            log_e("Pad is already inactive. That shouldn't be.. Wierd");
            return;
        }

        m_initializedFlags &= ~(1UL << m_pad);

        if (!m_initializedFlags) {
            log_d("uninstalling TouchPin driver");
            touch_pad_deinit();
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////

TouchPin::TouchPin(const touch_pad_t pad, const uint16_t tap_ms, const uint16_t press_ms, const uint8_t knock_count)
    : TouchPinRaw(pad)
    , counter(0)
    , maximum(0)
    , treshold(0)
    , current(0)
    , knock_counter(0)
    , knock_count(knock_count)
    , tap_ms(tap_ms)
    , press_ms(press_ms)
    , history { .value = 0 }
    , readings { .value = 0 }
    , updated_millis(0)
    , contact_millis(0)
    , contact(false)
    , contact_(false)
    , contact__(false)
    , release(false)
    , touch(false)
    , tap(false)
    , press(false)
    , press_(false)
    , knock(false)
{
}

uint8_t TouchPin::update(const bool force_update, bool debug_print)
{
    const uint32_t current_millis = millis();
    const uint8_t reading = readRaw8();

    // TARGETTING 100 UPDATES PER SECOND
    if(current_millis - updated_millis < 10) {
        return reading;
    }

    readings.value = (readings.value << 8) | reading;

    if ((readings.bytes[3] >= readings.bytes[2] && readings.bytes[2] >= readings.bytes[1] && readings.bytes[1] >= readings.bytes[0])
        || (readings.bytes[3] <= readings.bytes[2] && readings.bytes[2] <= readings.bytes[1] && readings.bytes[1] <= readings.bytes[0])) {
        current = readings.bytes[0];
    } else {
        current = history.bytes[0];
    }

    history.value = (history.value << 8) | current;

    if (counter++ >= 256) {
        maximum -= (counter / 256);
        counter %= 256;
    }

    if (current > maximum) {
        maximum = current;
    }

    const uint8_t delta = maximum - current;

    //if delta is bigger than maximum / 2 (50% of maximum), then it's touching and
    //if the value is under treshold and also dropped quickly (in 8 cycles)
    if (delta > (maximum >> 1) + 1) {
        if (((int16_t)history.bytes[7] - (int16_t)history.bytes[0]) > (int16_t)(maximum >> 3)) {
            contact_ = true;
        }
    } else {
        // maximum / 8 (12.5% of maximum)
        if (delta <= (maximum >> 3)) {
            contact_ = false;
        }
    }

    if (contact_ && !contact__) {
        contact__ = true;

        press_ = false;
        contact = true;

        const uint32_t released_duration = current_millis - contact_millis;

        if (released_duration >= 10) {
            if (released_duration < 300) {
                ++knock_counter;
            } else {
                knock_counter = 0;
            }
        }

        contact_millis = current_millis;
    }

    if (!contact_ && contact__) {
        contact__ = false;

        touch = true;
        release = true;

        const uint32_t contacted_duration = current_millis - contact_millis;

        if (knock_counter == 0 && contacted_duration >= tap_ms && contacted_duration < press_ms) {
            tap = true;
        }

        if (knock_counter >= knock_count) {
            knock_counter = 0;

            knock = true;
        }

        contact_millis = current_millis;
    }

    if (contact__ && !press_ && current_millis - contact_millis >= press_ms) {
        press_ = true;

        press = true;
    }

    updated_millis = current_millis;

    if(debug_print) {
        Serial.printf("rea:%u,cur:%u,max:%u,del:%u,con:%u\n", reading, current, maximum, delta, contact_ * 10);
    }

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
        return updated_millis - contact_millis;
    } else {
        return 0;
    }
}

uint32_t TouchPin::touched()
{
    if (touch) {
        touch = false;
        return contact_millis;
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

bool TouchPin::knocked()
{
    bool ret = knock;
    knock = false;
    return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////

#endif