#ifndef DECODER_H
#define DECODER_H

#include "Arduino.h" //for stdint
#include "util/circBuf.h"
#include "Protocol.h"
#include "Receiver.h"

class InputStream {
public:
    virtual int read() = 0;
    virtual int available() = 0;
};
struct Packet{
    int start;
    int callback;
};

/**
 * Decodes packets of the form <header+><signifier>data<checksum+><footer+>
 */

class Decoder{
public:
    Decoder(InputStream* input, Protocol* protocol, Receiver** receivers, int num_receivers):
        input(input), protocol(protocol), receivers(receivers), num_receivers(num_receivers) { }
    void update();
private:
    static const int BUF_SIZE = 64;
    InputStream* input;
    Protocol* protocol;
    Receiver** receivers;
    int num_receivers;
    circBuf<int, BUF_SIZE> buffer;
    circBuf<Packet, 4> packets;
    void receive(char);
    bool bufferMatch(int, const char *, int);
    int  findReceiver(char);
    void checkPackets(int);
};

#endif
