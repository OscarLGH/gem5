#include <stdio.h>

#define PIM_VADDR_BASE 0xffff800000000000
#define PIM_DATA(x) *(unsigned long long*)(PIM_VADDR_BASE+x)

unsigned long gem5_get_tick()
{
    asm(".byte 0x0f,0x04,0x00,0x03\n");
}

void main()
{
    gem5_get_tick();
    PIM_DATA(0x30) = 0xdeadbeef;
    printf("PIM DATA:0x30 = %llx\n", PIM_DATA(0x30));
    gem5_get_tick();
}