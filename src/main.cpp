/// Test reading the battery voltage as it dips below 3.3v
///
/// James Gallagher 2023

#include <Arduino.h>
#include <RHDatagram.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <RTCZero.h>

#include "debug.h"
#include "get_battery_voltage.h"
#include "messages.h"
#include "print.h"

#define MAIN_NODE_ADDRESS 0

// Define in the platformio file. jhrg 7/31/21
#ifndef NODE_ADDRESS
#define NODE_ADDRESS 4
#endif

#define Serial SerialUSB  // Needed for RS. jhrg 7/26/20
#define SERIAL_CONNECT_TRIES 10
#define SERIAL_CONNECT_INTERVAL 1000  // ms

// Pin assignments

#define RFM95_INT 2  // RF95 Interrupt
#define FLASH_CS 4   // CS for 2MB onboard flash on the SPI bus
#define RFM95_CS 5   // RF95 SPI CS

#define STATUS_LED 13

#define USE_AREF_2V23 1
#define V_BAT A1  // A5

// Constants

// Channel 0 is 902.3, others are + 200KHz for BW = 125 KHz. There are 64 channels.
// 915.0 MHz is the no-channel nominal freq
// Define in the platformio file. jhrg 7/31/21
#ifndef FREQUENCY
#define FREQUENCY 902.3
#endif

// Use these settings:
#define BANDWIDTH 125000
#define SPREADING_FACTOR 10
#define CODING_RATE 5

#ifndef STANDBY_INTERVAL_S
#define STANDBY_INTERVAL_S 300  // seconds to wait/sleep before next transmission
#endif

#ifndef TIME_REQUEST_SAMPLE_PERIOD
#define TIME_REQUEST_SAMPLE_PERIOD 24  // Ask the time once for every N data samples
#endif

#define BOOT_SAFETY_DELAY 10000  // 10s

#define WAIT_AVAILABLE 5000    // ms to wait for reply from main node
#define SD_POWER_ON_DELAY 200  // ms

#define ADC_BITS 12
#define ADC_MAX_VALUE 4096

#define STATUS_OK 0x00

// These errors are reset on every iteration of loop()
#define RFM95_SEND_ERROR 0x01
#define RFM95_NO_REPLY 0x02

// These codes indicate errors at boot time. They are persistent.
#define RFM95_INIT_ERROR 0x40

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// Singleton instance for the reliable datagram manager
RHReliableDatagram rf95_manager(rf95, NODE_ADDRESS);

unsigned int tx_power = 23;  // dBm; 5 to 23 for RF95

// Singleton for the Real Time Clock
RTCZero rtc;

uint8_t status = STATUS_OK;

#if 0
/**
 * @brief Send a short message for debugging using the LoRa
 * @param msg The message; null terminated string
 * @param to Send to this node
 */
void lora_debug(const char *msg, uint8_t to) {
    rf95_manager.sendtoWait((uint8_t *)msg, strlen(msg) + 1, to);
}
#endif

/**
 * @brief Send a message to a node
 * @param data Address of the data to send
 * @param to Destination node number (0-255)
 * @param size Number of bytes to send
 */
void send_message(uint8_t *data, uint8_t to, uint32_t size) {
    // This may block for up to CAD_TIMEOUT seconds
    if (!rf95_manager.sendtoWait(data, size, to)) {
        status |= RFM95_SEND_ERROR;
    }

    // This is not needed if the 'TO' address above is a specific node. If
    // RH_BROADCAST_ADDRESS is used, then we should wait
    if (to == RH_BROADCAST_ADDRESS) {
        if (!rf95_manager.waitPacketSent(WAIT_AVAILABLE)) {
            status |= RFM95_SEND_ERROR;
        }
    }
}

/**
 * Read a message. Each time this is called, the static storage used to hold
 * the message is cleared. This function waits for 5s (see WAIT_AVAILABLE)
 * for a message to appear.
 *
 * If no message is received, the global 'status' is set with the code
 * RFM95_NO_REPLY.
 *
 * @return The message, held in static storage. Returns nullptr if no message
 * was received.
 */
uint8_t *receive_message() {
    // Used to hold any reply from the main node
    static uint8_t rf95_buf[RH_RF95_MAX_MESSAGE_LEN];
    memset(rf95_buf, 0, sizeof(rf95_buf));

    // Now wait for a reply
    uint8_t len = sizeof(rf95_buf);
    uint8_t from;

    // Should be a reply message for us now
    if (rf95_manager.waitAvailableTimeout(WAIT_AVAILABLE)) {
        if (rf95_manager.recvfromAck(rf95_buf, &len, &from)) {
            return rf95_buf;
        } else {
            status |= RFM95_NO_REPLY;
        }
    } else {
        status |= RFM95_NO_REPLY;
    }

    return nullptr;
}

/**
 * Update the node's time using 'main_node_time' if the difference between
 * the two times is greater than one second.
 *
 * @param main_node_time The unix time from the main node
 * @return True if the time was updated, false if not.
 */
bool update_time(uint32_t main_node_time) {
    int32_t delta_time = main_node_time - rtc.getEpoch();

    IO(Serial.print("Main node time: "));
    IO(Serial.print(main_node_time));
    IO(Serial.print(", Time from this node: "));
    IO(Serial.print(rtc.getEpoch()));
    IO(Serial.print(", Delta: "));
    IO(Serial.println(delta_time));

    // update the time if the delta is more than a second
    if (abs(delta_time) > 1) {
        rtc.setEpoch(main_node_time);
        return true;
    }

    return false;
}

void setup() {
    // Blanket pin mode settings
    // Switch unused pins as input and enabled built-in pullup
    for (unsigned int pinNumber = 0; pinNumber < 23; pinNumber++) {
        pinMode(pinNumber, INPUT_PULLUP);
    }

    for (unsigned int pinNumber = 32; pinNumber < 42; pinNumber++) {
        pinMode(pinNumber, INPUT_PULLUP);
    }

    pinMode(25, INPUT_PULLUP);
    pinMode(26, INPUT_PULLUP);

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

#if DEBUG
    Serial.begin(115200);
    int tries = 0;
    // Wait for serial port to be available
    while ((tries < SERIAL_CONNECT_TRIES) && !Serial) {
        delay(SERIAL_CONNECT_INTERVAL);
        ++tries;
    };
#else
    delay(10000);
#endif

    get_battery_voltage_setup();

    IO(Serial.print(F("Initializing LORA...")));

    if (!rf95_manager.init()) {
        IO(Serial.println(F("LoRa init failed.")));
        status |= RFM95_INIT_ERROR;
    }

    rf95_manager.setRetries(2);  // default is 3
    // the value based on the ACK time (6 Octets == 327ms given SF 10, CR 5, BW 125kHz)
    rf95_manager.setTimeout(400);

    // Setup ISM frequency
    if (!rf95.setFrequency(FREQUENCY)) {
        IO(Serial.println(F("LoRa frequency out of range.")));
        status |= RFM95_INIT_ERROR;
    }

    // Setup Power,dBm
    rf95.setTxPower(tx_power);
    // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
    // Lower BandWidth for longer distance.
    rf95.setSignalBandwidth(BANDWIDTH);
    // Setup Spreading Factor (6 ~ 12)
    rf95.setSpreadingFactor(SPREADING_FACTOR);
    // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8)
    rf95.setCodingRate4(CODING_RATE);
    // 10 seconds
    rf95.setCADTimeout(RH_CAD_DEFAULT_TIMEOUT);

    IO(Serial.println(F(" Done.")));

    // USBDevice.detach();
    digitalWrite(STATUS_LED, LOW);
}

void loop() {
    digitalWrite(STATUS_LED, HIGH);
    int v = get_battery_voltage();

    char buf[TEXT_BUF_LEN];
    snprintf(buf, TEXT_BUF_LEN, "t: %ld, v: %d \n", (unsigned long)rtc.getEpoch(), v);
    IO(Serial.print("Text message: "));
    IO(Serial.println(buf));

    text_t t_msg;
    build_text_message(&t_msg, NODE_ADDRESS, TEXT_BUF_LEN, (uint8_t *)buf /* TEXT_BUF_LEN */);

#if 0
    IO(Serial.print("text bytes: "));
    for (int i = 0; i < sizeof(text_t); ++i) {
        IO(Serial.print(((const char *)&t_msg)[i], 16));
    }
    IO(Serial.println());
#endif

#if 0
    send_message((uint8_t *)&t_msg, MAIN_NODE_ADDRESS, sizeof(text_t));
#endif
    digitalWrite(STATUS_LED, LOW);
    delay(1000);
}
