#include "lora_frame.h"
#include <stdio.h>

#define NO_DATA "NO_FIX,r=0.0,p=0.0,y=0.0,t=-1.0,m=S,i=0,c=0"

size_t LoraFrame_Build(char *out, size_t size, const char *id, const char *status, uint32_t sequence, uint8_t victims) {
    int n = snprintf(out, size, "id=%s,%s,q=%lu,v=%u", id, status != NULL ? status : NO_DATA,
                     (unsigned long)sequence, (unsigned int)victims);
    return (n < 0 || (size_t)n >= size) ? 0U : (size_t)n;
}
