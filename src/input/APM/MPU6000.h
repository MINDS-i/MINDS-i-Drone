//MPU6000 Accelerometer and Gyroscope on SPI
class MPU6000 : public Sensor {
protected:
    const static uint8_t APM26_CS_PIN = 53;

    uint8_t chipSelect;


public:
    MPU6000(): chipSelect(APM26_CS_PIN) {}
    MPU6000(uint8_t chip_select): chipSelect(chip_select) {}
    void init();
    void stop();
    bool status();
    void calibrate();
    void update(InertialManager& man);
    //end of sensor interface
    void readTest();
};
void
MPU6000::init(){
}
void
MPU6000::stop(){
}
bool
MPU6000::status(){
}
void
MPU6000::calibrate(){
}
void
MPU6000::update(InertialManager& man){
}
void
MPU6000::readTest(){
    uint8_t address = 0x3B | 0x80;
    byte returns[24];
    for (int i = 0; i < 24; i++) returns[i] = 0;

    tic(0);
    uint8_t oldSREG = SREG;
    cli();
    digitalWrite(chipSelect, LOW);

    SPI.transfer(address);
    for(int i=0; i<24; i++) returns[i] = SPI.transfer(address+i);

    digitalWrite(chipSelect, HIGH);
    SREG = oldSREG;
    toc(0);

    Serial.println("\nBatch read:");
    for(int i=0; i<24; i++){
        if(i%6==0) Serial.print("\n");
        Serial.print(returns[i],HEX);
        Serial.print("\t");
    }

    delay(500);

    Serial.println("\nOld style read:");
    for(int i=0; i<24; i++){
        if(i%6==0) Serial.print("\n");
        Serial.print(MPU_SPI_read(address+i),HEX);
        Serial.print("\t");
    }
}
