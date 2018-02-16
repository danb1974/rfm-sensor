#include <Rfm69.h>

class Sensor
{
  public:
    Sensor(RFM69 &radio);

  private:
    RFM69 _radio;
};
