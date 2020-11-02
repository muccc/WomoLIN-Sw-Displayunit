#include <stdio.h>
#include <stdint.h>
//RGB332
int main(){
    for(int i = 0; i < 256; i++){
        uint8_t r = (i >> 5) & 0x07;
        uint8_t g = (i >> 2) & 0x07;
        uint8_t b = i & 0x03;
        uint32_t out = 0;
        out += (uint8_t)(r*255.0/7.0) << 16;
        out += (uint8_t)(g*255.0/7.0) << 8;
        out += (uint8_t)(b*255.0/3.0) << 0;
        printf("0x%06x, //%u,%u,%u \n",out,r,g,b);
    }
}
