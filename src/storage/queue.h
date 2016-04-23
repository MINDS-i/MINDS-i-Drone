#ifndef SIMPLEQUEUE_H
#define SIMPLEQUEUE_H
#include "Arduino.h"
/*
	SimpleQueue is designed to use global data so that memory use will be more
	predictable
*/
template <typename T>
class SimpleQueue{
private:
	T* data;
	uint8_t maxSize;
	uint8_t next;
	uint8_t end;
	boolean full;
	void advance(uint8_t& index){
		index = index+1;
		if(index >= maxSize) index -= maxSize;
	}
public:
	SimpleQueue(T* array, const uint8_t &size):
			data(array), maxSize(size), next(0), end(0), full(false) {
		for(int i=0; i<maxSize; i++) data[i] = T();
	}
	boolean push(T input){
		if(full) return false; //full
		data[next] = input;
		advance(next);
		if(next == end) full = true;
		return true;
	}
	T pop(){
		if (this->isEmpty()) return T(); //empty
		T ret = data[end];
		advance(end);
		full = false;
		return ret;
	}
	boolean isFull(){
		return full;
	}
	boolean isEmpty(){
		return (!full && (next == end));
	}
};
#endif
