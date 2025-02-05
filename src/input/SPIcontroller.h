#ifndef SPICONTROLLER_H
#define SPICONTROLLER_H
/**
 * This class is used to control the SPI chip select lines
 * Sensors will each own an SPIcontroller and put all
 * calls to SPI within a capture() and a release()
 */
class SPIcontroller {
  private:
    static volatile uint8_t controllerCount;

    SPISettings settings;
    const uint8_t csBit;
    volatile uint8_t* const csReg;

    uint8_t SREG_store;

  public:
    SPIcontroller(uint8_t chipSelect, SPISettings set)
        : settings(set), csBit(digitalPinToBitMask(chipSelect)),
          csReg(portOutputRegister(digitalPinToPort(chipSelect))) {
        if (controllerCount == 0) {
            SPI.begin();
        }
        controllerCount++;

        pinMode(chipSelect, OUTPUT);
        digitalWrite(chipSelect, HIGH);
        // digitalWrite is slower, but turns off PWM if necessary for us
    }
    ~SPIcontroller() {
        controllerCount--;
        if (controllerCount == 0) {
            SPI.end();
        }
    }
    void capture() __attribute__((optimize(0))) {
        SREG_store = SREG;
        cli();
        SPI.beginTransaction(settings);
        *csReg &= ~csBit;
    }
    boolean release() __attribute__((optimize(0))) {
        *csReg |= csBit;
        SPI.endTransaction();
        SREG = SREG_store;
        return true;
    }
};
volatile uint8_t SPIcontroller::controllerCount = 0;
#endif
