#include <Rfm69.h>

#ifndef SEND_RETRIES
#define SEND_RETRIES 5
#endif
#ifndef RETRY_INTERVAL
#define RETRY_INTERVAL 200
#endif

#ifndef CONFIG_FLASH_SIZE
#define CONFIG_FLASH_SIZE (512UL * 1024UL)
#endif
#ifndef CONFIG_FLASH_ADDRESS
#define CONFIG_FLASH_ADDRESS (CONFIG_FLASH_SIZE - 4096UL)
#endif

typedef void (*DataReceivedHandler)(const uint8_t *data, uint8_t length);

class Sensor
{
public:
  Sensor(bool useInterrupts = true);

  void init();
  void init(uint8_t id, uint8_t gwId, const uint8_t *key, bool isRfm69Hw = true, bool write = true);
  void interrupt();
  void update();
  bool send(const uint8_t *data, uint8_t size);
  bool sendAndWait(const uint8_t *data, uint8_t size);
  void onMessage(DataReceivedHandler handler);
  uint16_t readVoltage();
  void powerDown();
  void powerUp();
#ifndef SENSOR_NO_SLEEP
  void sleep(uint16_t seconds);
  void wake();
#endif

private:
  bool _useInterrupts;
  RFM69 _radio;
  uint8_t _id, _gwId;
  RfmPacket _packet;
  uint32_t _nextSendNonce;
  DataReceivedHandler _handler;

  uint32_t _oldReceiveNonce, _nextReceiveNonce;
  uint8_t *_data, _size, _retries;
  uint32_t _lastSendTime;
  bool _sendOk;
#ifndef SENSOR_NO_SLEEP
  volatile int16_t _seconds;
#endif

  void onPacketReceived();
  void sendData();
  void sendDone();
  void sendResponse(uint32_t nonce, bool ack);
  uint32_t readLong(const uint8_t *data);
  uint16_t readShort(const uint8_t *data);
  void writeNonce(uint8_t *data, uint32_t nonce);
  uint32_t createNonce();
  void handlePacket(const uint8_t *data, uint8_t size);
};
