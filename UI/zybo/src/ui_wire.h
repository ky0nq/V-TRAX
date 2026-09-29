#ifndef UI_WIRE_H
#define UI_WIRE_H
#include <stdint.h>
/* All multibyte wire values are BIG ENDIAN, including RGB565 pixels. */
static inline void UiPut16(uint8_t *p, unsigned v) {
    p[0]=(uint8_t)(v>>8);p[1]=(uint8_t)v;
}
static inline void UiPut32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);
    p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;
}
static inline uint16_t UiRgb565(unsigned r,unsigned g,unsigned b) {
    return (uint16_t)(((r&0xf8U)<<8)|((g&0xfcU)<<3)|(b>>3));
}
static inline void UiVideoHeader(uint8_t *p, unsigned width, unsigned height,
    unsigned index,unsigned count,unsigned length,uint32_t frame_id,
    uint32_t total,uint32_t session) {
    p[0]='H';p[1]='U';p[2]='D';p[3]='V';p[4]=1;p[5]=3;
    UiPut16(p+6,width);UiPut16(p+8,height);UiPut16(p+10,index);
    UiPut16(p+12,count);UiPut16(p+14,length);UiPut32(p+16,frame_id);
    UiPut32(p+20,total);UiPut32(p+24,session);
}
#endif
