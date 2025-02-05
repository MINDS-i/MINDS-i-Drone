#ifndef CIRCBUF_H
#define CIRCBUF_H

#include "Arduino.h" //for stdint

/*
 * A circular buffer of type T that holds up to S elements
 * looping over the buffer should be done from start() to end() by incrementing
 * When the buffer is full, adding new elements will delete the oldest ones
 */

template <typename T, int S> class circBuf {
  private:
    T data[S];
    uint16_t youngest; // the next unused location
    uint16_t oldest;   // the first used location
    bool empty;

  public:
    circBuf() : youngest(0), oldest(0), empty(true) {}
    /** get the item at position `pos` */
    T get(int pos) { return data[pos % S]; }
    /** get the item at position `pos` */
    T operator[](int pos) { return get(pos); }
    /** remove the `toRemove` oldest items from the buffer */
    void remove(int toRemove) {
        if (toRemove >= size()) {
            oldest = 0;
            youngest = 0;
            empty = true;
            return;
        }
        oldest = (oldest + toRemove) % S;
    }
    /**
     * Add an item `item` to the buffer, removing the oldest item to make room
     * for the new one if necessary
     */
    void add(T item) {
        data[youngest] = item;
        if (!empty && youngest == oldest) { // delete oldest item
            oldest = (oldest + 1) % S;
        }
        youngest = (youngest + 1) % S;
        empty = false;
    }
    /** return the index of the oldest item in the buffer */
    int start() { // inclusive
        return oldest;
    }
    /**
     * Return an index that is one past the youngest item and strictly larger
     * than the index returned by `start` to support iterative indexing from
     * `start` to `end`.
     */
    int end() { // exclusive
        if (empty) {
            return oldest;
        }
        return (youngest > oldest) ? youngest : youngest + S;
    }
    /** Return the number of items in the buffer */
    int size() {
        if (empty) {
            return 0;
        }
        return end() - start();
    }
    /** return the remaining available space for new items in the buffer */
    int remaining() { return S - size(); }
};

#endif
