#ifndef SIMPLEQUEUE_H
#define SIMPLEQUEUE_H
#include "Arduino.h"
/*
    SimpleQueue is designed to use global data so that memory use will be more
    predictable
*/
/** T must have a default constructor */
template <typename T> class SimpleQueue {
  private:
    T* data;
    uint8_t maxSize;
    uint8_t next;
    uint8_t end;
    boolean full;
    void advance(uint8_t& index) {
        index = index + 1;
        if (index >= maxSize) {
            index -= maxSize;
        }
    }

  public:
    SimpleQueue(T* array, const uint8_t& size) : data(array), maxSize(size), next(0), end(0), full(false) {
        for (int i = 0; i < maxSize; i++) {
            data[i] = T();
        }
    }
    /**
     * Attempt to push `input` into the queue
     * returns true if the insertion was succesful
     * returns false if the insertion failed (queue was full)
     */
    boolean push(T input) {
        if (full) {
            return false; // full
        }
        data[next] = input;
        advance(next);
        if (next == end) {
            full = true;
        }
        return true;
    }
    /**
     * pop the top item off the queue and return it
     * returns default `T()` if empty
     */
    T pop() {
        if (this->isEmpty()) {
            return T(); // empty
        }
        T ret = data[end];
        advance(end);
        full = false;
        return ret;
    }
    /** returns true if the queue is full */
    boolean isFull() { return full; }
    /** returns true if the queue is empty */
    boolean isEmpty() { return (!full && (next == end)); }
};
#endif
