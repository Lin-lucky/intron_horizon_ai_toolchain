#ifndef __KEROS_H__
#define __KEROS_H__

#include "keros_lib_1_8v.h"

int hobot_keros_gen_rand(void);
int hobot_keros_authorize(int seed, int *checkv, const void *key);
int hobot_keros_read_license(int seed, int *checkv, void *buffer, int len);

#endif
