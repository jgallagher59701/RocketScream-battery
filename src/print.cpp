
/**
 * A version of printf for the Arduino.
 * 11/20/22
 * James Gallagher <jhrg@mac.com>
 */

#include <Arduino.h>
#include <stdarg.h>

/**
 * @brief A printf() clone
 *
 * Use sprintf to format data for printing and pass the resulting string to
 * Arduino Serial.print(). Does not print a newline, so include that in 'fmt.'
 *
 * @note The printf family on the Atmel 328 processors does not support float
 * or double values without rebuilding one of the libraries. Use %d and round()
 * or break the float into two parts.
 *
 * @param fmt A sprintf format string
 * @param ... Variadic arguments matching fmt
 * @return void
 */
void print(const char *fmt, ...) {
    char msg[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);  // copies args
    va_end(ap);

    Serial.print(msg);
}

void print(const __FlashStringHelper *flash_fmt, ...) {
    // Copy the string from flash to RAM for use with vsnprintf()
    char fmt[128];
    strncpy_P(fmt, (const char *)flash_fmt, sizeof(fmt));

    va_list ap;
    va_start(ap, flash_fmt);
    char msg[128];
    vsnprintf(msg, sizeof(msg), fmt, ap);  // copies args
    va_end(ap);

    Serial.print(msg);
}

void flush() {
    Serial.flush();
}