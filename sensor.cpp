#include "sensor.h"
#include "hal.h"

#define DEBUG 0

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

typedef enum {
    Data = 0x01,
    Ack = 0x02,
    Nack = 0x03,
} MsgType;

void radioInterrupt()
{
    if (self)
        self->interrupt();
}

Sensor::Sensor(uint8_t id, uint8_t gwId)
    : _radio(spi_Transfer, millis, true), _id(id), _gwId(gwId)
{
    _data = NULL;
    _size = 0;
    _retries = 0;
    _handler = NULL;

    _oldReceiveNonce = createNonce();
    _nextReceiveNonce = createNonce();
    _nextSendNonce = createNonce();

    digitalWrite(SS, HIGH);
    pinMode(SS, OUTPUT);

    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV4);

    _radio.initialize(RF69_433MHZ, id);

    self = this;
    attachInterrupt(0, radioInterrupt, RISING);
}

void Sensor::init(const uint8_t *key)
{
    _radio.encrypt(key);
}

void Sensor::interrupt()
{
    _packet.size = 0;
    _radio.interrupt(_packet);
}

void Sensor::update()
{
    if (_packet.size > 0 && _packet.from == _gwId)
    {
        debugHex("RX", _packet.from, _packet.data, _packet.size);
        onPacketReceived();
        _packet.size = 0;
    }

    if (_retries && millis() > _lastSendTime + RETRY_INTERVAL)
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
        uint32_t nonce = readNonce(data);
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

        if (_handler)
        {
            _handler(data, size);
        }
    }
    break;

    case MsgType::Ack:
    {
        uint32_t ackNonce = readNonce(data);
        if (size != 9 || ackNonce != _nextSendNonce)
            break;

        _nextSendNonce = readNonce(&data[4]);
        sendDone();
        _sendOk = true;
    }
    break;

    case MsgType::Nack:
    {
        uint32_t nackNonce = readNonce(data);
        if (size != 9 || nackNonce != _nextSendNonce)
            break;

        _nextSendNonce = readNonce(&data[4]);
        sendData();
    }
    break;
    }
}

uint32_t Sensor::readNonce(const uint8_t *data)
{
    uint32_t nonce = *data++;
    nonce |= (uint32_t)*data++ << 8;
    nonce |= (uint32_t)*data++ << 16;
    nonce |= (uint32_t)*data++ << 24;
    return nonce;
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
