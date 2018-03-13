#include <Rfm69.h>

#define SEND_RETRIES 10
#define RETRY_INTERVAL 50

typedef void (*DataReceivedHandler)(const uint8_t *data, uint8_t length);

class Sensor
{
public:
  Sensor();

  void init();
  void init(uint8_t id, uint8_t gwId, const uint8_t *key, bool write = true);
  void interrupt();
  void update();
  bool send(const uint8_t *data, uint8_t size);
  bool sendAndWait(const uint8_t *data, uint8_t size);
  void onMessage(DataReceivedHandler handler);

private:
  RFM69 _radio;
  uint8_t _id, _gwId;
  RfmPacket _packet;
  uint32_t _nextSendNonce;
  DataReceivedHandler _handler;
  volatile bool _int;

  uint32_t _oldReceiveNonce, _nextReceiveNonce;
  uint8_t *_data, _size, _retries;
  uint32_t _lastSendTime;
  bool _sendOk;

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
