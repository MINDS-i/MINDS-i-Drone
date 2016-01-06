#ifndef RECEIVER_H
#define RECEIVER_H

/**
 * Used to define a target for a Decoder instance to send parsed messages to
 */
class Receiver{
public:
    /**
     * The Decoder will pass the first byte of the message to the claim function
     * Should return 0 if the message does not belong to this receiver
     * Otherwise, the maximum valid length this message could be
     */
    virtual int claim(char data) = 0;
    /**
     * The Decoder will pass a pointer to a temporary buffer containing the
     * body and checksum of a claimed, valid message to this function
     */
    virtual void handle(const char* buffer, int len) = 0;
};

#endif
