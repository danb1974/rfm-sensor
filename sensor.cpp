#include <avr/wdt.h>
#include <avr/eeprom.h>
#include "sensor.h"
#include "hal.h"
#ifndef SENSOR_NO_OTA
#include "SPIFlash.h"
#endif

#ifndef SENSOR_NO_DEFAULT_SPI
#include "SPI.h"
#endif

#ifndef SENSOR_NO_SLEEP
#include <LowPower.h>
#endif

#define DEBUG 0

#ifndef SENSOR_NO_OTA
SPIFlash flash(8);
#endif

#if DEBUG
void debugHex(const char *prefix, uint8_t addr, uint8_t *data, uint8_t size)
{
    char aux[100];
    sprintf(aux, "%s(%d):", prefix, addr);
    Serial.print(aux);
    while (size--)
    {
        uint8_t b = *data++;
        if (b < 16)
            Serial.print('0');
        Serial.print(b, HEX);
    }
    Serial.println();
}
#else
#define debugHex(a, b, c, d)
#endif

Sensor *self;

#define CONFIG_MAGIC_KEY 0x3A157FA4
typedef struct
{
    uint32_t magicKey;
    uint8_t key[16];
    uint8_t id;
    uint8_t gwId;
#define CONFIG_FLAG_IS_HW 0x01
    uint8_t flags;
} Config;

typedef enum
{
    Data = 0x01,
    Ack = 0x02,
    Nack = 0x03,
} MsgType;

#ifndef SENSOR_NO_INTERRUPTS
void radioInterrupt()
{
    if (self)
        self->interrupt();
}
#endif

Sensor::Sensor(
#ifdef SENSOR_NO_DEFAULT_SPI
    spiTransferFunction spiTransfer
#endif
#ifndef SENSOR_NO_INTERRUPTS
#ifdef SENSOR_NO_DEFAULT_SPI
    ,
#endif
    bool useInterrupts
#endif
    )
    : _radio(
#ifdef SENSOR_NO_DEFAULT_SPI
          spiTransfer,
#else
          spi_Transfer,
#endif
          millis)
{
#ifndef SENSOR_NO_INTERRUPTS
    _useInterrupts = useInterrupts;
#endif
    _data = NULL;
    _size = 0;
    _retries = 0;
    _handler = NULL;

    _oldReceiveNonce = createNonce();
    _nextReceiveNonce = createNonce();
    _nextSendNonce = createNonce();

    digitalWrite(SS, HIGH);
    pinMode(SS, OUTPUT);

#ifndef SENSOR_NO_DEFAULT_SPI
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV4);
#endif
}

bool Sensor::init(uint8_t id, uint8_t gwId, const uint8_t *key, bool isRfm69Hw, bool write)
{
    if (write)
    {
        Config config, oldConfig;

        config.magicKey = CONFIG_MAGIC_KEY;
        config.id = id;
        config.gwId = gwId;
        config.flags = isRfm69Hw ? CONFIG_FLAG_IS_HW : 0;
        memcpy(config.key, key, 16);

#ifndef SENSOR_NO_OTA
        if (flash.initialize())
        {
            flash.readBytes(CONFIG_FLASH_ADDRESS, &oldConfig, sizeof(Config));
            if (memcmp(&config, &oldConfig, sizeof(Config)) != 0) {
                flash.blockErase4K(CONFIG_FLASH_ADDRESS);
                while (flash.busy())
                {
                }

                flash.writeBytes(CONFIG_FLASH_ADDRESS, &config, sizeof(Config));
                while (flash.busy())
                {
                }

                flash.sleep();
            }
        }
        else
#endif
        {
            eeprom_read_block(&oldConfig, 0, sizeof(Config));
            if (memcmp(&config, &oldConfig, sizeof(Config)) != 0) {
                eeprom_update_block(&config, 0, sizeof(Config));
            }
        }
    }

    _id = id;
    _gwId = gwId;

    if (!_radio.initialize(RF69_433MHZ, id, 1, isRfm69Hw)) 
    {
        return false;
    }

#ifndef SENSOR_NO_INTERRUPTS
    if (_useInterrupts)
    {
        self = this;
        attachInterrupt(0, radioInterrupt, RISING);
    }
#endif

    if (key)
    {
        _radio.encrypt(key);
    }

    return true;
}

void Sensor::powerLevel(uint8_t level)
{
    _radio.setPowerLevel(level);
}

bool Sensor::init()
{
    Config config;

#ifndef SENSOR_NO_OTA
    bool flashReadFailed = true;
    
    if (flash.initialize())
    {
        flash.readBytes(CONFIG_FLASH_ADDRESS, &config, sizeof(config));
        flashReadFailed = config.magicKey != CONFIG_MAGIC_KEY;
    }
    else
    {
        flashReadFailed = false;
    }
#else
    bool flashReadFailed = false;
#endif

    if (flashReadFailed)
    {
        eeprom_read_block(&config, 0, sizeof(config));
    }

    if (config.magicKey == CONFIG_MAGIC_KEY)
    {
        return init(config.id, config.gwId, config.key, (config.flags & CONFIG_FLAG_IS_HW) != 0, flashReadFailed);
    }
    else
    {
        return false;
    }
}

void Sensor::interrupt()
{
    _packet.size = 0;
    _radio.receive(_packet);
}

void Sensor::update()
{
#ifndef SENSOR_NO_INTERRUPTS
    if (!_useInterrupts)
#endif
    {
        _packet.size = 0;
        _radio.receive(_packet);
    }

    if (_packet.size > 0 && _packet.from == _gwId)
    {
        debugHex("RX", _packet.from, _packet.data, _packet.size);
        onPacketReceived();
        _packet.size = 0;
    }

    if (_retries && millis() - _lastSendTime > RETRY_INTERVAL)
    {
        _retries--;
        if (_retries == 0)
        {
            sendDone();
            _sendOk = false;
        }
        else
        {
            sendData();
        }
    }
}

void Sensor::onPacketReceived()
{
    uint8_t *data = _packet.data;
    uint8_t size = _packet.size - 1;
    switch (*data++)
    {
    case MsgType::Data:
    {
        uint32_t nonce = readLong(data);
        if (nonce == _oldReceiveNonce)
        {
            //already received this data
            sendResponse(nonce, true);
            break;
        }

        if (nonce != _nextReceiveNonce)
        {
            //this is not what we're expecting
            sendResponse(nonce, false);
            break;
        }

        data += 4; //skip nonce
        size -= 4;

        _oldReceiveNonce = _nextReceiveNonce;
        do
        {
            _nextReceiveNonce = createNonce();
        } while (_nextReceiveNonce == _oldReceiveNonce);
        sendResponse(nonce, true);
        handlePacket(data, size);
    }
    break;

    case MsgType::Ack:
    {
        uint32_t ackNonce = readLong(data);
        if (size != 9 || ackNonce != _nextSendNonce)
            break;

        _nextSendNonce = readLong(&data[4]);
        sendDone();
        _sendOk = true;
    }
    break;

    case MsgType::Nack:
    {
        uint32_t nackNonce = readLong(data);
        if (size != 9 || nackNonce != _nextSendNonce)
            break;

        _nextSendNonce = readLong(&data[4]);
        sendData();
    }
    break;
    }
}

uint32_t Sensor::readLong(const uint8_t *data)
{
    uint32_t value = *data++;
    value |= (uint32_t)*data++ << 8;
    value |= (uint32_t)*data++ << 16;
    value |= (uint32_t)*data++ << 24;
    return value;
}

uint16_t Sensor::readShort(const uint8_t *data)
{
    uint16_t value = *data++;
    value |= (uint32_t)*data++ << 8;
    return value;
}

void Sensor::writeNonce(uint8_t *data, uint32_t nonce)
{
    *data++ = nonce;
    *data++ = nonce >> 8;
    *data++ = nonce >> 16;
    *data++ = nonce >> 24;
}

uint32_t Sensor::createNonce()
{
    return random();
}

void Sensor::sendResponse(uint32_t nonce, bool ack)
{
    uint8_t data[10] = {ack ? MsgType::Ack : MsgType::Nack};
    writeNonce(&data[1], nonce);
    writeNonce(&data[5], _nextReceiveNonce);
    data[9] = _packet.rssi;
    debugHex("TX", _gwId, data, sizeof(data));
    _radio.send(_gwId, data, sizeof(data));
}

void Sensor::sendData()
{
    if (!_data)
        return;
    writeNonce(&_data[1], _nextSendNonce);
    debugHex("TX", _gwId, _data, _size);
    _radio.send(_gwId, _data, _size);
    _lastSendTime = millis();
}

void Sensor::sendDone()
{
    if (_data)
        free(_data);
    _data = NULL;
    _retries = 0;
    _size = 0;
}

bool Sensor::send(const uint8_t *data, uint8_t size)
{
    sendDone();
    _size = size + 5;
    _data = (uint8_t *)malloc(_size);
    if (_data == NULL)
        return false;
    _data[0] = MsgType::Data;
    _retries = SEND_RETRIES;
    memcpy(&_data[5], data, size);
    sendData();
    return true;
}

bool Sensor::sendAndWait(const uint8_t *data, uint8_t size)
{
    if (!send(data, size))
        return false;

    while (_retries)
        update();

    return _sendOk;
}

void Sensor::onMessage(DataReceivedHandler handler)
{
    _handler = handler;
}

void Sensor::handlePacket(const uint8_t *data, uint8_t size)
{
    switch (*data)
    {
#ifndef SENSOR_NO_OTA
    case 0xCA:
    {
        data++;
        size--;

        //begin OTA
        if (!flash.initialize())
        {
            uint8_t error[2] = {0xCA, 0xE1}; //no flash error
            send(error, sizeof(data));
            return;
        }

        flash.blockErase32K(0);
        while (flash.busy())
        {
            //wait for erase to finish
            update();
        }
        uint8_t reply = 0xCA;
        send(&reply, 1);
    }
    break;
    case 0xCB: //write at address
    {
        data++;
        size--;

        while (flash.busy())
        {
            //wait for previous operation
            update();
        }

        //write OTA part
        uint16_t flashAddress = readShort(data);
        data += 2;
        size -= 2;
        flash.writeBytes(flashAddress, data, size);
    }
    break;
#endif
    case 0xCC: //reset
    {
        wdt_enable(WDTO_15MS);
        while (1)
        {
            // wait for watchdog reset
        }
    }
    break;
    default:
        if (_handler)
            _handler(data, size, _packet.rssi);
        break;
    }
}

void Sensor::powerDown()
{
    _radio.sleep();
}

void Sensor::powerUp()
{
    _radio.wake();
}

#ifndef SENSOR_NO_SLEEP

void Sensor::sleep(uint16_t seconds)
{
    _seconds = seconds;
    if (_seconds == 0)
    {
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    }
    while (_seconds > 8)
    {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        _seconds -= 8;
    }
    while (_seconds > 4)
    {
        LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
        _seconds -= 4;
    }
    while (_seconds > 2)
    {
        LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
        _seconds -= 2;
    }
    while (_seconds > 0)
    {
        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_ON);
        _seconds -= 1;
    }
}

void Sensor::wake()
{
    _seconds = 0;
}

#endif

uint16_t Sensor::readVoltage()
{
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC))
    {
        // wait for ADC
    }
    return 1126400 / ADC;
}
