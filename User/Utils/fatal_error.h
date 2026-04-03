#ifndef __CHECK_PTR_H__
#define __CHECK_PTR_H__

#include <stddef.h>

#define CHECK_PTR(ptr) \
    do { \
        if ((ptr) == NULL) { \
            while(1); \
        } \
    } while (0);

#define INIT_ERROR() \
    do { \
        while(1); \
    } while (0);

#endif /* __CHECK_PTR_H__ */
