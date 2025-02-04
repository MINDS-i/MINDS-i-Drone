#ifndef BYTECONV_H
#define BYTECONV_H
union byteConv {
    uint32_t l;
    float f;
    uint8_t bytes[4];
};
template <typename CTYPE> union convert {
    CTYPE data;
    uint8_t bytes[sizeof(data)];
};
#endif
