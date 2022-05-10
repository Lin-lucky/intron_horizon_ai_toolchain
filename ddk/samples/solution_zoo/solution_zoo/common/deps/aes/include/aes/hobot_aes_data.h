#ifndef _ENCRYPT_DATA_H_
#define _ENCRYPT_DATA_H_
#define HOBOT_AES_BLOCK_SIZE 16
#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
  AES_FILE = 0,  //modle params and json memory size which encrypt is bigger than origin 
  AES_FEATURE = 1, //feature memory size is same after encrypt
}HobotAESEncryptType;

#ifdef __cplusplus
}
#endif
#endif

