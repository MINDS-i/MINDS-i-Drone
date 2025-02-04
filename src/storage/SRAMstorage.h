#ifndef SRAMSTORAGE_H
#define SRAMSTORAGE_H
/*
    The storage sram implementation
    stores an array of values, with callbacks
    the callbacks will be called when attached and written
*/

template <typename T, int SIZE> class SRAMstorage : public Storage<T> {
  private:
    T data[SIZE];
    void (*callback[SIZE])(T);

  public:
    SRAMstorage() {
        for (int i = 0; i < SIZE; i++) {
            data[i] = 0;
            callback[i] = NULL;
        }
    }
    void attachCallback(uint8_t dataNum, void (*call)(T)) {
        if (dataNum >= SIZE)
            return;
        callback[dataNum] = call;
        if (callback[dataNum] != NULL)
            callback[dataNum](getRecord(dataNum));
    }
    void updateRecord(uint8_t dataNum, T value) {
        if (dataNum >= SIZE)
            return;
        data[dataNum] = value;
        if (callback[dataNum] != NULL)
            callback[dataNum](value);
    }
    T getRecord(uint8_t dataNum) {
        if (dataNum >= SIZE)
            return T();
        return data[dataNum];
    }
};

#endif
