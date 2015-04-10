/**
 * This class is used to control the SPI chip select lines
 * Sensors will each own an SPIcontroller and put all
 * calls to SPI within a capture() and a release()
 *
 * Note that the SPI library can do a better job of preventing interrupted
 * transactions then this can through SPI.usingInterruts(int_num);
 */
class SPIcontroller{
private:
    static uint8_t controllerCount;
    static SPIcontroller* activeController;

    uint8_t csBit;
    volatile uint8_t* csReg;
    SPISettings settings;
public:
    SPIcontroller(uint8_t chipSelect, SPISettings set):settings(set){
        if(controllerCount == 0){
            SPI.begin();
        }
        controllerCount++;

        csBit = digitalPinToBitMask(chipSelect);
        csReg = portOutputRegister(digitalPinToPort(chipSelect));
        pinMode(chipSelect, OUTPUT);
        digitalWrite(chipSelect, HIGH);
        //digitalWrite is slower, but turns off PWM if necessary for us
    }
    ~SPIcontroller(){
        controllerCount--;
        if(controllerCount == 0){
            SPI.end();
        }
    }
    void capture(){
        if(activeController != NULL) activeController->release();
        activeController = this;
        SPI.beginTransaction(settings);
        *csReg &= ~csBit;
    }
    boolean release(){
        *csReg |= csBit;
        SPI.endTransaction();
        boolean interrupted = (activeController != this);
        if(!interrupted) {
            activeController = NULL;
        }
        return interrupted;
    }
};
uint8_t        SPIcontroller::controllerCount  = 0;
SPIcontroller* SPIcontroller::activeController = NULL;
