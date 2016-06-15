#ifndef STORAGE_H
#define STORAGE_H

template<typename T>
class Storage{
protected:
	Storage(){}
public:
    /**
     * Attach a callback to location `dataNum` that will be fired every time
     * it is updated. It will be called immediately after being attached with
     * the current value
     */
	virtual void attachCallback(uint8_t dataNum, void (*call)(T))=0;
    /**
     * Update the data at location `dataNum` to `value`, firing attached
     * callbacks as necessary
     */
	virtual void updateRecord(uint8_t dataNum, T value)=0;
    /** Retreive the data at location `dataNum` */
	virtual T getRecord(uint8_t dataNum)=0;
};

#endif
