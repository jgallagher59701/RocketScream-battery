//
// Read the battery voltage for the Leaf node using a 1M | 1M voltage
// divider and the RocketScream/M0 ADC.
//
// James Gallagher <jgallagher@opendap.org>
// 9/26/21

#include "get_battery_voltage.h"

#include <Arduino.h>

#define USE_AREF_2V23 1
#define USE_AREF_1V0 0
#define V_BAT A5

#define ADC_BITS 10
#define ADC_MAX_VALUE 1023.0
#define VALUES_TO_AVG 10
#define VOLTAGE_OFFSET 0.0

/**
 * @brief Read the current voltage of the battery
 *
 * @note this version assumes that a voltage divider reduces Vbat by 1/2
 * @return The battery voltage x 100 as an int
 */
int get_battery_voltage() {
    // Take one sample, discard it, then 10 samples
    // and return their average. Time delays, etc., can be set in
    // setup() using ADC calls. See:
    // https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/

    (void)analogRead(V_BAT);  // discard the first read

    int raw = 0;
    for (int i = 0; i < VALUES_TO_AVG; ++i) {
        raw += analogRead(V_BAT);
    }

    float avg_adc_reading = (float)raw / VALUES_TO_AVG;

    // The 1st multiplication by 2 is due to the 0.5 gain. The 2nd multiplication 
    // by 2 is due to the half voltage divider. Reference is 1.0V
    constexpr int k = 2 * 2;
    float voltage = avg_adc_reading * (k / 1023.0);

    return (int)roundf((voltage + VOLTAGE_OFFSET) * 100.0);  // voltage * 100
}

void get_battery_voltage_setup() {
    // See https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/
    // And assume an input impedance of 500k for the 1M | 1M V divider
    ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(3);

    // This is from a private communication with Lim Phang Moh or RocketScream.
    // The idea is that we want to get accurate readings even as the battery voltage
    // drops way down but, by reading the wiring_analog.c in the Arduino SAMD21
    // library, he found that both the AR_INTERNAL1V65 and AR_INTERNAL2V23 use
    // Vcc and that likely explains why the ADC fails to read accurately as the
    // battery (i.e., Vcc) falls below 3V3. This _should_ work, but some of the 
    // code int eh Arduino library may not be correct. However, the AR_INTERNAL1V0 
    // does not appear to be tied to Vcc and using that yields correct values down to 
    // Vbat == 2V2 at which point the MCU just stops. Set the gain here to be 0.5. 
    // jhrg 11/25/23
    while (ADC->STATUS.bit.SYNCBUSY == 1);
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;  // 0.5 gain Factor Selection
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val; // 1.0V voltage reference
    while (ADC->STATUS.bit.SYNCBUSY == 1);
}
