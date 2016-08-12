#include "glx.h"
#include <xs1.h>

#define CT_PERIPH_WRITE 0x24
#define JUNK_RETURN_ADDRESS 0xFF

int write_periph_word(tileref tile, unsigned peripheral, unsigned addr, unsigned data)
{
    unsigned tmp[1];
    tmp[0] = data;
    return write_periph_32(tile, peripheral, addr, 1, tmp);
}

int read_periph_word(tileref tile, unsigned peripheral, unsigned addr, unsigned &data)
{
    unsigned tmp[1];
    int retval = read_periph_32(tile, peripheral, addr, 1, tmp);
    data = tmp[0];
    return retval;
}


void write_periph_word_two_part_start(int tmpchan, tileref tile, unsigned peripheral,
                                      unsigned base_address, unsigned data)
{
    asm("setd res[%0], %1" ::
        "r"(tmpchan),
        "r"((get_tile_id(tile) << 16) | (peripheral << 8) | XS1_RES_TYPE_CHANEND));

    /* Preload as much as possible, everything up to last byte of data */
    asm("outct res[%0], %1" :: "r"(tmpchan), "r"(CT_PERIPH_WRITE));
    asm("out res[%0], %1" :: "r"(tmpchan), "r"((JUNK_RETURN_ADDRESS << 8) | (base_address & 0xFF)));
    asm("outt res[%0], %1" :: "r"(tmpchan), "r"(sizeof(unsigned)));
    asm("outt res[%0], %1" :: "r"(tmpchan), "r"(data >> 24));
    asm("outt res[%0], %1" :: "r"(tmpchan), "r"(data >> 16));
    asm("outt res[%0], %1" :: "r"(tmpchan), "r"(data >> 8));
}

void write_periph_word_two_part_end(int tmpchan, unsigned data)
{
    /* Send last byte of data to bring the write to effect */
    asm("outt res[%0], %1" :: "r"(tmpchan), "r"(data));
    asm("outct res[%0], %1" :: "r"(tmpchan), "r"(XS1_CT_END));
}
