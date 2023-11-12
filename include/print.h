
/**
 * A version of printf for the Arduino.
 * 11/20/22
 * James Gallagher <jhrg@mac.com>
 */

void print(const char *fmt, ...);
void print(const __FlashStringHelper *fmt, ...);

// Print using %02d
// TODO needs work
// #define frac(f) (int)((f > 0.0) ? (int(f*100)%100) : round((f+ceil(f))*100.0))
#define frac(f) (int(f * 100) % 100)

void flush();

// Using F() in this macro reduced RAM use from 67% to 50% in ~1200 LOC
#if DEBUG
#define DPRINTV(fmt, ...) print(F(fmt), __VA_ARGS__)
#define DPRINT(fmt) print(F(fmt))
#define DPRINTF(fmt, f) \
    Serial.print(fmt);  \
    Serial.println(f)
#else
#define DPRINTV(fmt, ...)
#define DPRINT(fmt)

#define DPRINTF(fmt, f)
#endif
