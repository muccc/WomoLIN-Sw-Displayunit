#include "clut.h"

//converts RGB332 to RGB888, generated by tools/clut.c
const uint32_t clut_table[] = {
  0x000000, //0,0,0 
  0x000055, //0,0,1 
  0x0000aa, //0,0,2 
  0x0000ff, //0,0,3 
  0x002400, //0,1,0 
  0x002455, //0,1,1 
  0x0024aa, //0,1,2 
  0x0024ff, //0,1,3 
  0x004800, //0,2,0 
  0x004855, //0,2,1 
  0x0048aa, //0,2,2 
  0x0048ff, //0,2,3 
  0x006d00, //0,3,0 
  0x006d55, //0,3,1 
  0x006daa, //0,3,2 
  0x006dff, //0,3,3 
  0x009100, //0,4,0 
  0x009155, //0,4,1 
  0x0091aa, //0,4,2 
  0x0091ff, //0,4,3 
  0x00b600, //0,5,0 
  0x00b655, //0,5,1 
  0x00b6aa, //0,5,2 
  0x00b6ff, //0,5,3 
  0x00da00, //0,6,0 
  0x00da55, //0,6,1 
  0x00daaa, //0,6,2 
  0x00daff, //0,6,3 
  0x00ff00, //0,7,0 
  0x00ff55, //0,7,1 
  0x00ffaa, //0,7,2 
  0x00ffff, //0,7,3 
  0x240000, //1,0,0 
  0x240055, //1,0,1 
  0x2400aa, //1,0,2 
  0x2400ff, //1,0,3 
  0x242400, //1,1,0 
  0x242455, //1,1,1 
  0x2424aa, //1,1,2 
  0x2424ff, //1,1,3 
  0x244800, //1,2,0 
  0x244855, //1,2,1 
  0x2448aa, //1,2,2 
  0x2448ff, //1,2,3 
  0x246d00, //1,3,0 
  0x246d55, //1,3,1 
  0x246daa, //1,3,2 
  0x246dff, //1,3,3 
  0x249100, //1,4,0 
  0x249155, //1,4,1 
  0x2491aa, //1,4,2 
  0x2491ff, //1,4,3 
  0x24b600, //1,5,0 
  0x24b655, //1,5,1 
  0x24b6aa, //1,5,2 
  0x24b6ff, //1,5,3 
  0x24da00, //1,6,0 
  0x24da55, //1,6,1 
  0x24daaa, //1,6,2 
  0x24daff, //1,6,3 
  0x24ff00, //1,7,0 
  0x24ff55, //1,7,1 
  0x24ffaa, //1,7,2 
  0x24ffff, //1,7,3 
  0x480000, //2,0,0 
  0x480055, //2,0,1 
  0x4800aa, //2,0,2 
  0x4800ff, //2,0,3 
  0x482400, //2,1,0 
  0x482455, //2,1,1 
  0x4824aa, //2,1,2 
  0x4824ff, //2,1,3 
  0x484800, //2,2,0 
  0x484855, //2,2,1 
  0x4848aa, //2,2,2 
  0x4848ff, //2,2,3 
  0x486d00, //2,3,0 
  0x486d55, //2,3,1 
  0x486daa, //2,3,2 
  0x486dff, //2,3,3 
  0x489100, //2,4,0 
  0x489155, //2,4,1 
  0x4891aa, //2,4,2 
  0x4891ff, //2,4,3 
  0x48b600, //2,5,0 
  0x48b655, //2,5,1 
  0x48b6aa, //2,5,2 
  0x48b6ff, //2,5,3 
  0x48da00, //2,6,0 
  0x48da55, //2,6,1 
  0x48daaa, //2,6,2 
  0x48daff, //2,6,3 
  0x48ff00, //2,7,0 
  0x48ff55, //2,7,1 
  0x48ffaa, //2,7,2 
  0x48ffff, //2,7,3 
  0x6d0000, //3,0,0 
  0x6d0055, //3,0,1 
  0x6d00aa, //3,0,2 
  0x6d00ff, //3,0,3 
  0x6d2400, //3,1,0 
  0x6d2455, //3,1,1 
  0x6d24aa, //3,1,2 
  0x6d24ff, //3,1,3 
  0x6d4800, //3,2,0 
  0x6d4855, //3,2,1 
  0x6d48aa, //3,2,2 
  0x6d48ff, //3,2,3 
  0x6d6d00, //3,3,0 
  0x6d6d55, //3,3,1 
  0x6d6daa, //3,3,2 
  0x6d6dff, //3,3,3 
  0x6d9100, //3,4,0 
  0x6d9155, //3,4,1 
  0x6d91aa, //3,4,2 
  0x6d91ff, //3,4,3 
  0x6db600, //3,5,0 
  0x6db655, //3,5,1 
  0x6db6aa, //3,5,2 
  0x6db6ff, //3,5,3 
  0x6dda00, //3,6,0 
  0x6dda55, //3,6,1 
  0x6ddaaa, //3,6,2 
  0x6ddaff, //3,6,3 
  0x6dff00, //3,7,0 
  0x6dff55, //3,7,1 
  0x6dffaa, //3,7,2 
  0x6dffff, //3,7,3 
  0x910000, //4,0,0 
  0x910055, //4,0,1 
  0x9100aa, //4,0,2 
  0x9100ff, //4,0,3 
  0x912400, //4,1,0 
  0x912455, //4,1,1 
  0x9124aa, //4,1,2 
  0x9124ff, //4,1,3 
  0x914800, //4,2,0 
  0x914855, //4,2,1 
  0x9148aa, //4,2,2 
  0x9148ff, //4,2,3 
  0x916d00, //4,3,0 
  0x916d55, //4,3,1 
  0x916daa, //4,3,2 
  0x916dff, //4,3,3 
  0x919100, //4,4,0 
  0x919155, //4,4,1 
  0x9191aa, //4,4,2 
  0x9191ff, //4,4,3 
  0x91b600, //4,5,0 
  0x91b655, //4,5,1 
  0x91b6aa, //4,5,2 
  0x91b6ff, //4,5,3 
  0x91da00, //4,6,0 
  0x91da55, //4,6,1 
  0x91daaa, //4,6,2 
  0x91daff, //4,6,3 
  0x91ff00, //4,7,0 
  0x91ff55, //4,7,1 
  0x91ffaa, //4,7,2 
  0x91ffff, //4,7,3 
  0xb60000, //5,0,0 
  0xb60055, //5,0,1 
  0xb600aa, //5,0,2 
  0xb600ff, //5,0,3 
  0xb62400, //5,1,0 
  0xb62455, //5,1,1 
  0xb624aa, //5,1,2 
  0xb624ff, //5,1,3 
  0xb64800, //5,2,0 
  0xb64855, //5,2,1 
  0xb648aa, //5,2,2 
  0xb648ff, //5,2,3 
  0xb66d00, //5,3,0 
  0xb66d55, //5,3,1 
  0xb66daa, //5,3,2 
  0xb66dff, //5,3,3 
  0xb69100, //5,4,0 
  0xb69155, //5,4,1 
  0xb691aa, //5,4,2 
  0xb691ff, //5,4,3 
  0xb6b600, //5,5,0 
  0xb6b655, //5,5,1 
  0xb6b6aa, //5,5,2 
  0xb6b6ff, //5,5,3 
  0xb6da00, //5,6,0 
  0xb6da55, //5,6,1 
  0xb6daaa, //5,6,2 
  0xb6daff, //5,6,3 
  0xb6ff00, //5,7,0 
  0xb6ff55, //5,7,1 
  0xb6ffaa, //5,7,2 
  0xb6ffff, //5,7,3 
  0xda0000, //6,0,0 
  0xda0055, //6,0,1 
  0xda00aa, //6,0,2 
  0xda00ff, //6,0,3 
  0xda2400, //6,1,0 
  0xda2455, //6,1,1 
  0xda24aa, //6,1,2 
  0xda24ff, //6,1,3 
  0xda4800, //6,2,0 
  0xda4855, //6,2,1 
  0xda48aa, //6,2,2 
  0xda48ff, //6,2,3 
  0xda6d00, //6,3,0 
  0xda6d55, //6,3,1 
  0xda6daa, //6,3,2 
  0xda6dff, //6,3,3 
  0xda9100, //6,4,0 
  0xda9155, //6,4,1 
  0xda91aa, //6,4,2 
  0xda91ff, //6,4,3 
  0xdab600, //6,5,0 
  0xdab655, //6,5,1 
  0xdab6aa, //6,5,2 
  0xdab6ff, //6,5,3 
  0xdada00, //6,6,0 
  0xdada55, //6,6,1 
  0xdadaaa, //6,6,2 
  0xdadaff, //6,6,3 
  0xdaff00, //6,7,0 
  0xdaff55, //6,7,1 
  0xdaffaa, //6,7,2 
  0xdaffff, //6,7,3 
  0xff0000, //7,0,0 
  0xff0055, //7,0,1 
  0xff00aa, //7,0,2 
  0xff00ff, //7,0,3 
  0xff2400, //7,1,0 
  0xff2455, //7,1,1 
  0xff24aa, //7,1,2 
  0xff24ff, //7,1,3 
  0xff4800, //7,2,0 
  0xff4855, //7,2,1 
  0xff48aa, //7,2,2 
  0xff48ff, //7,2,3 
  0xff6d00, //7,3,0 
  0xff6d55, //7,3,1 
  0xff6daa, //7,3,2 
  0xff6dff, //7,3,3 
  0xff9100, //7,4,0 
  0xff9155, //7,4,1 
  0xff91aa, //7,4,2 
  0xff91ff, //7,4,3 
  0xffb600, //7,5,0 
  0xffb655, //7,5,1 
  0xffb6aa, //7,5,2 
  0xffb6ff, //7,5,3 
  0xffda00, //7,6,0 
  0xffda55, //7,6,1 
  0xffdaaa, //7,6,2 
  0xffdaff, //7,6,3 
  0xffff00, //7,7,0 
  0xffff55, //7,7,1 
  0xffffaa, //7,7,2 
  0xffffff, //7,7,3 
};
