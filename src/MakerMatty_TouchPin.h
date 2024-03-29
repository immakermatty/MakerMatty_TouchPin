/** 
 * Author	: @makermatty (maker.matejsuchanek.cz)
 * Date		: 15-6-2020
 */

#ifdef ESP32

#ifndef _MM_TOUCH_PIN_h
#define _MM_TOUCH_PIN_h

#include <Arduino.h>

#ifndef CONFIG_PLATFORM_0_3_X
#include "driver/touch_sensor.h"
#endif

#include "MakerMatty_Curves.h"
#include "MakerMatty_TemplateMath.h"

///////////////////////////////////////////////////////////////////////////////////////////////

class TouchPinRaw {

    static uint32_t m_initializedFlags;

public:
    TouchPinRaw(touch_pad_t pad);
    TouchPinRaw(const TouchPinRaw& other) = delete;
    TouchPinRaw(TouchPinRaw&& other);
    ~TouchPinRaw();

    uint16_t readRaw();
    uint8_t readRaw8();

    void info();
    touch_pad_t getPad();

private:
    uint16_t m_value;
    touch_pad_t m_pad;
};

typedef TouchPinRaw MakerMatty_TouchPinRaw;

///////////////////////////////////////////////////////////////////////////////////////////////

class TouchPin : public TouchPinRaw {

public:
    TouchPin(const touch_pad_t pad, const uint16_t tap_ms, const uint16_t press_ms, const uint8_t knock_count);
    TouchPin(const TouchPin& other) = delete;
    TouchPin(TouchPin&& other) = default;

    // called before any of the other functions
    // duration cca 185 us
    // returns current read, that was updated
    uint8_t update(const bool force_update = false, const bool skip_read = false, bool debug_print = false);

    // get the processed value of touchpin
    uint8_t getValue();
    // get the maximum value of touchpin
    uint8_t getAverage();

    // 0 if not currently touching, else number of ms spent touching - returns value during touching
    uint32_t touching();
    // 0 if not touched, else number of ms spent holding - returns value after touching
    uint32_t touched();

    bool contacted();
    bool released();
    
    bool tapped();
    bool pressed();
    bool knocked();

private:
    uint16_t counter;
    uint16_t knock_counter;
    
    uint8_t maximum;
    uint8_t treshold;
    uint8_t current;
    uint8_t knock_count;

    uint16_t tap_ms;
    uint16_t press_ms;
   
    union {
        uint8_t bytes[8];
        uint64_t value;
    } history;

    union {
        uint8_t bytes[4];
        uint32_t value;
    } readings;

    uint32_t updated_millis;
    uint32_t contact_millis;

    MA<uint8_t> reading;
    EMA average;

    bool contact, contact_, contact__;
    bool release;
    bool touch;
    bool tap, tap_;
    bool press, press_;
    bool knock;
};

typedef TouchPin MakerMatty_TouchPin;

///////////////////////////////////////////////////////////////////////////////////////////////

#endif
#endif
