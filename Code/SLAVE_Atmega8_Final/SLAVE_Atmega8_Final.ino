#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <math.h>

// HomeLink Audio Pro - ATmega8 final slave
// PC0 NTC, PC4/PC5 I2C slave, PD2 DS18B20, PD5 RLY1, PD6 RLY2, PB0 LED.
// PD0/PD1 are diagnostic UART at 9600 baud. No external crystal.

#define I2C_ADDRESS          0x12
#define DS18B20_BIT          PD2
#define TACHO_BIT            PD3
#define TACHO_MODE_BIT       PD4
#define RELAY1_BIT           PD5
#define RELAY2_BIT           PD6
#define ACTIVITY_LED_BIT     PB0
#define RELAY_ACTIVE_LOW     1

// Divider: +5V -> NTC -> PC0 -> 10k -> GND.
#define NTC_FIXED_OHMS       10000.0f
#define NTC_NOMINAL_OHMS     47000.0f
#define NTC_NOMINAL_C        25.0f
#define NTC_BETA             3950.0f

#define DEFAULT_FAN_ON_X10   350
#define DEFAULT_FAN_OFF_X10  300
#define DEFAULT_OVERTEMP_X10 700
#define COMM_TIMEOUT_MS      3000UL
#define TACHO_PULSES_PER_REV 2
#define TACHO_WINDOW_MS      1000UL
#define TACHO_START_GRACE_MS 3000UL
#define TACHO_MIN_RPM        200
#define CONFIG_MAGIC         0xA8C1
#define CONFIG_VERSION       1

enum Command : uint8_t {
    CMD_SET_POWER      = 0x01,
    CMD_SET_THRESHOLDS = 0x02,
    CMD_SAVE_CONFIG    = 0x03,
    CMD_LOAD_DEFAULTS  = 0x04
};

struct Config {
    uint16_t magic;
    uint8_t version;
    int16_t fanOnX10;
    int16_t fanOffX10;
    int16_t overtempX10;
    uint8_t checksum;
};

static Config config;
static volatile bool powerRequested = false;
static volatile bool configDirty = false;
static volatile uint32_t lastNanoCommand = 0;
static volatile uint8_t commandSequence = 0;

static float ntcC = 0.0f;
static float dsC = 0.0f;
static uint16_t ntcAdc = 0;
static bool ntcOk = false;
static bool dsOk = false;
static bool rly1On = false;
static bool rly2On = false;
static bool overtemp = false;
static bool tachoEnabled = false;
static bool tachoFault = false;
static uint16_t fanRpm = 0;
static bool dsBusy = false;
static uint32_t dsStarted = 0;
static uint32_t lastSensor = 0;
static uint32_t saveDue = 0;
static uint32_t lastLed = 0;
static bool ledState = false;
static volatile uint16_t tachoPulses = 0;
static uint32_t tachoWindowStart = 0;
static uint32_t fanStartTime = 0;
static bool previousFanState = false;

ISR(INT1_vect)
{
    if (tachoPulses != 0xFFFF) tachoPulses++;
}

static uint8_t configChecksum(const Config &c)
{
    const uint8_t *p = reinterpret_cast<const uint8_t *>(&c);
    uint8_t result = 0;
    for (uint8_t i = 0; i < sizeof(Config) - 1; i++) result ^= p[i];
    return result;
}

static void setDefaults()
{
    config.magic = CONFIG_MAGIC;
    config.version = CONFIG_VERSION;
    config.fanOnX10 = DEFAULT_FAN_ON_X10;
    config.fanOffX10 = DEFAULT_FAN_OFF_X10;
    config.overtempX10 = DEFAULT_OVERTEMP_X10;
    config.checksum = configChecksum(config);
}

static bool configIsValid(const Config &c)
{
    return c.magic == CONFIG_MAGIC &&
           c.version == CONFIG_VERSION &&
           c.fanOffX10 < c.fanOnX10 &&
           c.overtempX10 > c.fanOnX10 &&
           configChecksum(c) == c.checksum;
}

static void loadConfig()
{
    EEPROM.get(0, config);
    if (!configIsValid(config)) {
        setDefaults();
        EEPROM.put(0, config);
    }
}

static void saveConfig()
{
    config.checksum = configChecksum(config);
    EEPROM.put(0, config);
    configDirty = false;
}

static void setActivity(bool on)
{
    if (on) PORTB |= _BV(ACTIVITY_LED_BIT);
    else    PORTB &= (uint8_t)~_BV(ACTIVITY_LED_BIT);
}

static void writeRelay(uint8_t bit, bool on)
{
    const bool high = RELAY_ACTIVE_LOW ? !on : on;
    if (high) PORTD |= _BV(bit);
    else PORTD &= (uint8_t)~_BV(bit);
}

static uint16_t readAdc0()
{
    ADMUX = _BV(REFS0); // AVCC reference, ADC0/PC0
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    delayMicroseconds(100);
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {}
    return ADC;
}

static void readNtc()
{
    ntcAdc = readAdc0();
    if (ntcAdc == 0 || ntcAdc >= 1023) {
        ntcOk = false;
        return;
    }

    const float resistance = NTC_FIXED_OHMS *
        ((1023.0f - (float)ntcAdc) / (float)ntcAdc);
    const float t0 = NTC_NOMINAL_C + 273.15f;
    const float invT = (1.0f / t0) +
        (log(resistance / NTC_NOMINAL_OHMS) / NTC_BETA);
    ntcC = (1.0f / invT) - 273.15f;
    ntcOk = true;
}

// Minimal DS18B20 driver; DATA requires an external 4.7k pull-up to +5V.
static inline void dsLow()
{
    PORTD &= (uint8_t)~_BV(DS18B20_BIT);
    DDRD |= _BV(DS18B20_BIT);
}

static inline void dsRelease()
{
    PORTD &= (uint8_t)~_BV(DS18B20_BIT);
    DDRD &= (uint8_t)~_BV(DS18B20_BIT);
}

static inline bool dsPinHigh()
{
    return (PIND & _BV(DS18B20_BIT)) != 0;
}

static bool dsReset()
{
    dsLow(); delayMicroseconds(480);
    dsRelease(); delayMicroseconds(70);
    const bool present = !dsPinHigh();
    delayMicroseconds(410);
    return present;
}

static void dsWriteBit(bool value)
{
    dsLow();
    if (value) {
        delayMicroseconds(6); dsRelease(); delayMicroseconds(64);
    } else {
        delayMicroseconds(60); dsRelease(); delayMicroseconds(10);
    }
}

static bool dsReadBit()
{
    dsLow(); delayMicroseconds(3); dsRelease(); delayMicroseconds(10);
    const bool value = dsPinHigh();
    delayMicroseconds(53);
    return value;
}

static void dsWriteByte(uint8_t value)
{
    for (uint8_t i = 0; i < 8; i++) { dsWriteBit(value & 1); value >>= 1; }
}

static uint8_t dsReadByte()
{
    uint8_t value = 0;
    for (uint8_t i = 0; i < 8; i++) if (dsReadBit()) value |= _BV(i);
    return value;
}

static uint8_t dsCrc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    while (length--) {
        uint8_t value = *data++;
        for (uint8_t i = 0; i < 8; i++) {
            const uint8_t mix = (crc ^ value) & 1;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            value >>= 1;
        }
    }
    return crc;
}

static bool dsStart()
{
    if (!dsReset()) return false;
    dsWriteByte(0xCC); // SKIP ROM
    dsWriteByte(0x44); // CONVERT T
    return true;
}

static bool dsRead()
{
    uint8_t data[9];
    if (!dsReset()) return false;
    dsWriteByte(0xCC); dsWriteByte(0xBE);
    for (uint8_t i = 0; i < 9; i++) data[i] = dsReadByte();
    if (dsCrc8(data, 8) != data[8]) return false;
    const int16_t raw = (int16_t)((uint16_t)data[1] << 8 | data[0]);
    dsC = (float)raw / 16.0f;
    return true;
}

static int16_t x10(float value)
{
    if (value > 3276.7f) return 32767;
    if (value < -3276.8f) return -32768;
    return (int16_t)(value * 10.0f);
}

static void putI16(uint8_t *p, uint8_t index, int16_t value)
{
    p[index] = (uint8_t)value;
    p[index + 1] = (uint8_t)((uint16_t)value >> 8);
}

static int16_t getI16(const uint8_t *p, uint8_t index)
{
    return (int16_t)((uint16_t)p[index] | ((uint16_t)p[index + 1] << 8));
}

static void onReceive(int count)
{
    if (count <= 0 || !Wire.available()) return;

    uint8_t data[8];
    uint8_t length = 0;
    while (Wire.available() && length < sizeof(data)) data[length++] = Wire.read();

    lastNanoCommand = millis();
    commandSequence++;

    switch (data[0]) {
        case CMD_SET_POWER:
            if (length >= 2) powerRequested = data[1] != 0;
            break;
        case CMD_SET_THRESHOLDS:
            if (length >= 7) {
                const int16_t on = getI16(data, 1);
                const int16_t off = getI16(data, 3);
                const int16_t over = getI16(data, 5);
                if (off < on && over > on) {
                    config.fanOnX10 = on;
                    config.fanOffX10 = off;
                    config.overtempX10 = over;
                    configDirty = true;
                    saveDue = millis() + 1000UL;
                }
            }
            break;
        case CMD_SAVE_CONFIG:
            configDirty = true;
            saveDue = millis();
            break;
        case CMD_LOAD_DEFAULTS:
            setDefaults();
            configDirty = true;
            saveDue = millis();
            break;
    }
}

static void onRequest()
{
    // 12 bytes: NTCx10, DSx10, flags, fanOnx10, fanOffx10, overx10, seq.
    uint8_t packet[12] = {};
    putI16(packet, 0, ntcOk ? x10(ntcC) : 0x7FFF);
    putI16(packet, 2, dsOk ? x10(dsC) : 0x7FFF);

    uint8_t flags = 0;
    if (ntcOk) flags |= _BV(0);
    if (dsOk) flags |= _BV(1);
    if (rly1On) flags |= _BV(2);
    if (rly2On) flags |= _BV(3);
    if (overtemp) flags |= _BV(4);
    if (powerRequested) flags |= _BV(5);
    if (configIsValid(config)) flags |= _BV(6);
    packet[4] = flags;
    putI16(packet, 5, config.fanOnX10);
    putI16(packet, 7, config.fanOffX10);
    putI16(packet, 9, config.overtempX10);
    packet[11] = commandSequence;
    Wire.write(packet, sizeof(packet));
}

static void applyRelays()
{
    const uint32_t now = millis();
    const bool nanoAlive = now - lastNanoCommand <= COMM_TIMEOUT_MS;
    overtemp = ntcOk && x10(ntcC) >= config.overtempX10;

    // RLY1: DSP converter. Safe OFF on communication or NTC failure.
    rly1On = powerRequested && nanoAlive && ntcOk && !overtemp && !tachoFault;

    // RLY2: fan with hysteresis. Fan ON on NTC fault.
    if (!ntcOk || overtemp || x10(ntcC) >= config.fanOnX10) rly2On = true;
    else if (x10(ntcC) <= config.fanOffX10) rly2On = false;

    writeRelay(RELAY1_BIT, rly1On);
    writeRelay(RELAY2_BIT, rly2On);
}

static void updateTacho()
{
    const uint32_t now = millis();

    if (rly2On && !previousFanState) fanStartTime = now;
    if (!rly2On) {
        fanRpm = 0;
        tachoFault = false;
    }

    if (now - tachoWindowStart >= TACHO_WINDOW_MS) {
        uint16_t pulses;
        noInterrupts();
        pulses = tachoPulses;
        tachoPulses = 0;
        interrupts();

        const uint32_t elapsed = now - tachoWindowStart;
        tachoWindowStart = now;
        fanRpm = (uint16_t)(((uint32_t)pulses * 60000UL) /
                            ((uint32_t)TACHO_PULSES_PER_REV * elapsed));

        if (tachoEnabled && rly2On &&
            now - fanStartTime >= TACHO_START_GRACE_MS) {
            tachoFault = fanRpm < TACHO_MIN_RPM;
        }
    }

    previousFanState = rly2On;
}

static void diagnostics()
{
    static uint32_t last = 0;
    const uint32_t now = millis();
    if (now - last < 2000UL) return;
    last = now;

    Serial.print(F("NTC="));
    if (ntcOk) Serial.print(ntcC, 1); else Serial.print(F("ERR"));
    Serial.print(F(" DS="));
    if (dsOk) Serial.print(dsC, 1); else Serial.print(F("ERR"));
    Serial.print(F(" RLY1=")); Serial.print(rly1On ? 1 : 0);
    Serial.print(F(" RLY2=")); Serial.print(rly2On ? 1 : 0);
    Serial.print(F(" TACHO=")); Serial.print(tachoEnabled ? 1 : 0);
    Serial.print(F(" RPM=")); Serial.print(fanRpm);
    Serial.print(F(" TACH_ERR=")); Serial.println(tachoFault ? 1 : 0);
}

void setup()
{
    DDRB |= _BV(ACTIVITY_LED_BIT);
    DDRD |= _BV(RELAY1_BIT) | _BV(RELAY2_BIT);
    DDRD &= (uint8_t)~(_BV(TACHO_BIT) | _BV(TACHO_MODE_BIT));
    PORTD |= _BV(TACHO_MODE_BIT); // internal pull-up; jumper to GND enables TACHO
    DDRD &= (uint8_t)~_BV(DS18B20_BIT);
    PORTD &= (uint8_t)~_BV(DS18B20_BIT);

    setActivity(false);
    writeRelay(RELAY1_BIT, false);
    writeRelay(RELAY2_BIT, false);
    Serial.begin(9600);
    loadConfig();

    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    tachoEnabled = (PIND & _BV(TACHO_MODE_BIT)) == 0;
    // INT1 / PD3, falling edge. The transistor output is pulled up to +5 V.
    // ATmega8 uses MCUCR/GICR, unlike newer AVR devices using EICRA/EIMSK.
    MCUCR = (MCUCR & (uint8_t)~(_BV(ISC11) | _BV(ISC10))) | _BV(ISC11);
    GICR |= _BV(INT1);
    tachoWindowStart = millis();

    wdt_enable(WDTO_2S);
    lastNanoCommand = millis();
    lastSensor = millis() - 500UL;
    dsStarted = millis() - 2000UL;
}

void loop()
{
    const uint32_t now = millis();
    wdt_reset();

    if (now - lastLed >= 500UL) {
        lastLed = now;
        ledState = !ledState;
        setActivity(ledState);
    }

    if (now - lastSensor >= 500UL) {
        lastSensor = now;
        readNtc();
        applyRelays();
        updateTacho();
        applyRelays();
    }

    if (!dsBusy && now - dsStarted >= 2000UL) {
        dsStarted = now;
        dsBusy = dsStart();
        if (!dsBusy) dsOk = false;
    }

    if (dsBusy && now - dsStarted >= 800UL) {
        dsOk = dsRead();
        dsBusy = false;
    }

    if (configDirty && (int32_t)(now - saveDue) >= 0) saveConfig();
    updateTacho();
    applyRelays();
    diagnostics();
}
