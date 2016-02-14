#ifndef PROTOCOL_H
#define PROTOCOL_H

/**
 * Used to define valid message boundries for a decoder object to parse with
 */

class Protocol{
protected:
    Protocol(const char* header, int header_len, const char* footer, int footer_len):
        header(header),
        header_len(header_len),
        footer(footer),
        footer_len(footer_len) { }
public:
    const char* header;
    int header_len;
    const char* footer;
    int footer_len;
    /**return whether `char c[len]` has a valid checksum at the end */
    virtual bool checksum(const char* c, int len) = 0;
};

#endif
