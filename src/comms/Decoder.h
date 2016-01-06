#ifndef DECODER_H
#define DECODER_H

#include "Arduino.h" //for stdint
#include "util/circBuf.h"

typedef char (*InputStream) (void);
class Receiver{
public:
    virtual int claim(char data) = 0;
    virtual void handle(const char* buffer, int len) = 0;
};
class Protocol{
protected:
    Protocol(const char* header, int header_len, const char* footer, int footer_len):
        header(header),
        header_len(header_len),
        footer(footer),
        footer_len(footer_len) { }
public:
    const char* header;
    const char* footer;
    int header_len;
    int footer_len;
    //return whether `char c[len]` has a valid checksum at the end
    virtual bool checksum(const char* c, int len) = 0;
};

struct Packet{
    int start;
    int callback;
};

class Decoder{
public:
    Decoder(InputStream read, Protocol* protocol, Receiver** receivers, int num_receivers):
        read(read), protocol(protocol), receivers(receivers), num_receivers(num_receivers) { }
    void update();
private:
    static const int BUF_SIZE = 64;
    InputStream read;
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
