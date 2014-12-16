#ifndef STORAGE_H
#define STORAGE_H
/*
	The storage abstract base class
	stores an array of values, with callbacks
	the callbacks will be called when attached and written
*/
template<typename T>
class Storage{
protected:
	Storage(){}
public:
	virtual void attachCallback(uint8_t dataNum, void (*call)(T))=0;
	virtual void updateRecord(uint8_t dataNum, T value)=0;
	virtual T getRecord(uint8_t dataNum)=0;
};

#endif
