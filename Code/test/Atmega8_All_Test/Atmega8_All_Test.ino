#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <math.h>

/*
 * ATmega8A DIP-28 test firmware
 *
 * Tested pins:
 *   PC0 / ADC0  - NTC voltage divider
 *   PC4 / SDA   - I2C
 *   PC5 / SCL   - I2C
 *   PD2         - DS18B20 data
 *   PD5         - relay 1
 *   PD6         - relay 2
 *   PB0         - activity LED
 *
 * UART diagnostics:
 *   PD0/RXD, PD1/TXD, 9600 baud
 *
 * This sketch uses the ATmega8 as an I2C MASTER for the I2C scanner.
 * Do not run the scanner on the same bus while the Nano is also acting
 * as an active I2C master. Use it first with the ATmega8 and peripherals
 * connected, or set I2C_SCAN_ENABLED to 0 for the final slave setup.
 */

// ---------- Hardware configuration ----------
#define DS18B20_BIT          PD2
#define RELAY1_BIT           PD5
#define RELAY2_BIT           PD6
#define ACTIVITY_LED_BIT     PB0

// Set to 0 if your relay modules are active HIGH.
#define RELAY_ACTIVE_LOW     1

// NTC divider: +5 V -> NTC -> ADC pin -> fixed resistor -> GND.
#define NTC_FIXED_OHMS       10000.0f
#define NTC_NOMINAL_C        22.3f
#define NTC_NOMINAL_OHMS 47000.0f
#define NTC_BETA         3950.0f

// Test sequence is deliberately slow so relay states are visible.
#define RELAY_TEST_ENABLED   1
#define RELAY_STEP_MS        3000UL

#define I2C_SCAN_ENABLED     1
#define I2C_SCAN_PERIOD_MS   5000UL

// ---------- Small helpers ----------
static void activityLed(bool on)
{
    if (on) PORTB |= _BV(ACTIVITY_LED_BIT);
    else    PORTB &= (uint8_t)~_BV(ACTIVITY_LED_BIT);
}

static void relayWrite(uint8_t bit, bool on)
{
    bool pinHigh = RELAY_ACTIVE_LOW ? !on : on;

    if (pinHigh) PORTD |= _BV(bit);
    else         PORTD &= (uint8_t)~_BV(bit);
}

static void allRelaysOff()
{
    relayWrite(RELAY1_BIT, false);
    relayWrite(RELAY2_BIT, false);
}

// ---------- ADC / NTC ----------
static uint16_t readAdc0()
{
    // AVCC reference, ADC0 = PC0, right-adjusted result.
    ADMUX = _BV(REFS0);
    ADCSRA = _BV(ADEN) |
             _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // F_CPU / 128

    delayMicroseconds(100);
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {}
    return ADC;
}

static bool ntcTemperatureC(float &temperatureC, uint16_t &adcValue, float &resistance)
{
    adcValue = readAdc0();

    if (adcValue == 0 || adcValue >= 1023) {
        resistance = 0.0f;
        return false;
    }

    // NTC is connected to +5 V; fixed resistor goes from ADC to GND.
    resistance = NTC_FIXED_OHMS *
             ((1023.0f - adcValue) / adcValue);

    const float t0Kelvin = NTC_NOMINAL_C + 273.15f;
    const float invT = (1.0f / t0Kelvin) +
                      (log(resistance / NTC_NOMINAL_OHMS) / NTC_BETA);

    temperatureC = (1.0f / invT) - 273.15f;
    return true;
}

// ---------- Minimal DS18B20 driver; no OneWire library required ----------
static inline void dsLow()
{
    PORTD &= (uint8_t)~_BV(DS18B20_BIT);
    DDRD |= _BV(DS18B20_BIT);
}

static inline void dsRelease()
{
    // External 4.7 kΩ pull-up is required on the DATA line.
    PORTD &= (uint8_t)~_BV(DS18B20_BIT);
    DDRD &= (uint8_t)~_BV(DS18B20_BIT);
}

static inline bool dsReadPin()
{
    return (PIND & _BV(DS18B20_BIT)) != 0;
}

static bool dsReset()
{
    dsLow();
    delayMicroseconds(480);
    dsRelease();
    delayMicroseconds(70);
    bool present = !dsReadPin();
    delayMicroseconds(410);
    return present;
}

static void dsWriteBit(bool bitValue)
{
    dsLow();

    if (bitValue) {
        delayMicroseconds(6);
        dsRelease();
        delayMicroseconds(64);
    } else {
        delayMicroseconds(60);
        dsRelease();
        delayMicroseconds(10);
    }
}

static bool dsReadBit()
{
    dsLow();
    delayMicroseconds(3);
    dsRelease();
    delayMicroseconds(10);
    bool value = dsReadPin();
    delayMicroseconds(53);
    return value;
}

static void dsWriteByte(uint8_t value)
{
    for (uint8_t i = 0; i < 8; i++) {
        dsWriteBit(value & 0x01);
        value >>= 1;
    }
}

static uint8_t dsReadByte()
{
    uint8_t value = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (dsReadBit()) value |= _BV(i);
    }

    return value;
}

static uint8_t dsCrc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;

    while (length--) {
        uint8_t inByte = *data++;

        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ inByte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inByte >>= 1;
        }
    }

    return crc;
}

static bool dsStartConversion()
{
    if (!dsReset()) return false;

    dsWriteByte(0xCC); // SKIP ROM
    dsWriteByte(0x44); // CONVERT T
    return true;
}

static bool dsReadTemperature(float &temperatureC, int16_t &rawValue)
{
    uint8_t scratchpad[9];

    if (!dsReset()) return false;

    dsWriteByte(0xCC); // SKIP ROM
    dsWriteByte(0xBE); // READ SCRATCHPAD

    for (uint8_t i = 0; i < 9; i++) {
        scratchpad[i] = dsReadByte();
    }

    if (dsCrc8(scratchpad, 8) != scratchpad[8]) return false;

    rawValue = (int16_t)((uint16_t)scratchpad[1] << 8 | scratchpad[0]);
    temperatureC = (float)rawValue / 16.0f;
    return true;
}

// ---------- I2C scanner ----------
static void scanI2c()
{
    Serial.println(F("I2C scan start (ATmega8 master)"));
    uint8_t found = 0;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print(F("  I2C device: 0x"));
            if (address < 16) Serial.print('0');
            Serial.println(address, HEX);
            found++;
        }
    }

    if (!found) Serial.println(F("  no I2C devices found"));
    Serial.println(F("I2C scan end"));
}

// ---------- Test state ----------
static uint32_t lastActivityMs = 0;
static uint32_t lastRelayStepMs = 0;
static uint32_t lastI2cScanMs = 0;
static uint32_t lastReportMs = 0;
static uint32_t dsConversionStartMs = 0;
static uint8_t relayTestStep = 0;
static bool dsConversionRunning = false;
static bool dsPresent = false;

static void runRelayTest(uint32_t now)
{
#if RELAY_TEST_ENABLED
    if (now - lastRelayStepMs < RELAY_STEP_MS) return;
    lastRelayStepMs = now;

    relayTestStep = (relayTestStep + 1) % 4;

    switch (relayTestStep) {
        case 0:
            relayWrite(RELAY1_BIT, false);
            relayWrite(RELAY2_BIT, false);
            break;
        case 1:
            relayWrite(RELAY1_BIT, true);
            relayWrite(RELAY2_BIT, false);
            break;
        case 2:
            relayWrite(RELAY1_BIT, false);
            relayWrite(RELAY2_BIT, true);
            break;
        case 3:
            relayWrite(RELAY1_BIT, true);
            relayWrite(RELAY2_BIT, true);
            break;
    }

    Serial.print(F("Relay test step: "));
    Serial.println(relayTestStep);
#else
    (void)now;
#endif
}

void setup()
{
    // Outputs first, then force a safe relay state.
    DDRB |= _BV(ACTIVITY_LED_BIT);
    DDRD |= _BV(RELAY1_BIT) | _BV(RELAY2_BIT);
    DDRD &= (uint8_t)~_BV(DS18B20_BIT);
    PORTD &= (uint8_t)~_BV(DS18B20_BIT);

    activityLed(false);
    allRelaysOff();

    Serial.begin(9600);
    Wire.begin();

    Serial.println();
    Serial.println(F("ATmega8 peripheral test"));
    Serial.println(F("PC0=NTC, PC4=SDA, PC5=SCL, PD2=DS18B20"));
    Serial.println(F("PD5=RELAY1, PD6=RELAY2, PB0=ACTIVITY"));
    Serial.println(F("Relays start OFF"));

    scanI2c();
    lastActivityMs = millis();
    lastRelayStepMs = millis();
    lastI2cScanMs = millis();
    lastReportMs = millis();
}

void loop()
{
    const uint32_t now = millis();

    // Activity LED heartbeat: 500 ms ON / 500 ms OFF.
    if (now - lastActivityMs >= 500) {
        lastActivityMs = now;
        activityLed((PORTB & _BV(ACTIVITY_LED_BIT)) == 0);
    }

    runRelayTest(now);

    // Start a DS18B20 conversion approximately every 2 seconds.
    if (!dsConversionRunning &&
        (now - dsConversionStartMs >= 2000UL || dsConversionStartMs == 0)) {
        dsPresent = dsStartConversion();
        dsConversionRunning = dsPresent;
        dsConversionStartMs = now;
    }

    // 12-bit DS18B20 conversion needs up to 750 ms.
    if (dsConversionRunning && now - dsConversionStartMs >= 800UL) {
        float dsTemperature = 0.0f;
        int16_t dsRaw = 0;

        if (!dsReadTemperature(dsTemperature, dsRaw)) {
            dsPresent = false;
        }

        dsConversionRunning = false;

        Serial.print(F("DS18B20: "));
        if (dsPresent) {
            Serial.print(dsTemperature, 2);
            Serial.print(F(" C, raw="));
            Serial.println(dsRaw);
        } else {
            Serial.println(F("ERROR / not detected / CRC error"));
        }
    }

    if (now - lastReportMs >= 2000UL) {
        lastReportMs = now;

        uint16_t adc = 0;
        float ntcTemperature = 0.0f;
        float ntcResistance = 0.0f;
        bool ntcOk = ntcTemperatureC(ntcTemperature, adc, ntcResistance);

        Serial.print(F("NTC ADC="));
        Serial.print(adc);
        Serial.print(F(" V="));
        Serial.print((float)adc * 5.0f / 1023.0f, 3);

        if (ntcOk) {
            Serial.print(F(" R="));
            Serial.print(ntcResistance, 0);
            Serial.print(F(" ohm T="));
            Serial.print(ntcTemperature, 2);
            Serial.println(F(" C"));
        } else {
            Serial.println(F(" NTC OUT OF RANGE"));
        }
    }

#if I2C_SCAN_ENABLED
    if (now - lastI2cScanMs >= I2C_SCAN_PERIOD_MS) {
        lastI2cScanMs = now;
        scanI2c();
    }
#endif
}
