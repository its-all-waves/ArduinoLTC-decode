// the time values (in decimal) extracted from LTC in the decoder
typedef struct DecodedLTC {
    uint8_t h, m, s, f;
    uint8_t ub7, ub6, ub5, ub4, ub3, ub2, ub1, ub0;
};