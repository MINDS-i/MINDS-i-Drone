#ifndef CIRCBUF_H
#define CIRCBUF_H

#include "Arduino.h" //for stdint

template<typename T, int S>
class circBuf{
private:
    T data[S];
    uint16_t youngest; //the next unused location
    uint16_t oldest; //the first used location
    bool empty;
public:
    circBuf(): youngest(0), oldest(0), empty(true) { }
    T get(int pos){
        return data[pos%S];
    }
    T operator[](int num) {
        return get(num);
    }
    void remove(int num){
        if(num >= size()) {
            oldest = 0;
            youngest = 0;
            empty = true;
            return;
        }
        oldest = (oldest+num)%S;
    }
    void add(T item){
        data[youngest] = item;
        if(!empty && youngest == oldest) //delete oldest item
            oldest = (oldest+1)%S;
        youngest = (youngest+1)%S;
        empty = false;
    }
    int start(){ //inclusive
        return oldest;
    }
    int end(){ //exclusive
        if(empty) return oldest;
        return (youngest > oldest) ? youngest : youngest+S;
    }
    int size(){
        if(empty) return 0;
        return end()-start();
    }
    int remaining(){
        return S-size();
    }
};

#endif
