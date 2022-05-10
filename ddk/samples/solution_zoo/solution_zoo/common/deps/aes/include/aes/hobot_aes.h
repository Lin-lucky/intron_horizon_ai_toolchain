#ifndef _ENCRYPT_H_
#define _ENCRYPT_H_

#include "hobot_aes_data.h"

#ifdef __cplusplus
extern "C" {
#endif

int hobot_aes_encrypt(int file_in, int file_out);
int hobot_aes_encrypt_char2char(char* buffer_in, char* buffer_out, int size);
int hobot_aes_decrypt_int2char(int file_in, char* decrypt, int *de_file_size);
int hobot_aes_decrypt(char* buffer_in, char* decrypt, int *de_file_size);
int HobotAESEncrypt(const char* buffer_in, char* buffer_out, HobotAESEncryptType type, int ori_size); 
int HobotAESDecrypt(const char* buffer_in, char* buffer_out, HobotAESEncryptType type, int enc_size, int *dec_size);

#ifdef __cplusplus
}
#endif

#endif
