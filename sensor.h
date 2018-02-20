#include <Rfm69.h>

#define SEND_RETRIES 10
#define RETRY_INTERVAL 50

typedef void (*DataReceivedHandler)(const uint8_t *data, uint8_t length);

class Sensor
{
public:
  Sensor(uint8_t id, uint8_t gwId = 1);

  void interrupt();
  void update();
  bool send(const uint8_t *data, uint8_t size);
  bool sendAndWait(const uint8_t *data, uint8_t size);
  void onMessage(DataReceivedHandler handler);

private:
  RFM69 _radio;
  const uint8_t _id, _gwId;
  RfmPacket _packet;
  uint32_t _nextSendNonce;
  DataReceivedHandler _handler;

  uint32_t _oldReceiveNonce, _nextReceiveNonce;
  uint8_t *_data, _size, _retries;
  uint32_t _lastSendTime;
  bool _sendOk;

  void onPacketReceived();
  void sendData();
  void sendDone();
  void sendResponse(uint32_t nonce, bool ack);
  uint32_t readNonce(const uint8_t *data);
  void writeNonce(uint8_t *data, uint32_t nonce);
  uint32_t createNonce();
};
