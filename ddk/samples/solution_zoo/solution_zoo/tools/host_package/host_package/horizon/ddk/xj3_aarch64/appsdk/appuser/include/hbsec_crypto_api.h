#ifndef __HBSEC_CRYPTO_API_H_
#define __HBSEC_CRYPTO_API_H_

#include "elpspaccmodes.h"
#include "elpspaccusr.h"

struct hbsec_ch {
	struct elp_spacc_usr io;
};

struct hbsec_ch_conf {
	int cipher_mode; int hash_mode;	// cipher and hash (see elpspaccmodes.h)
	int encrypt;					// non-zero for encrypt (sign) modes
	int icvmode;					// ICV mode (see elpspaccmodes.h)
	int icvlen;						// length of ICV, 0 for default length (algorithm dependent)
	int aad_copy;					// non-zero to copy AAD to dst buffer
	const void *ckey; int ckeylen; const void *civ; int civlen;	// cipher key and IVs
	const void *hkey; int hkeylen; const void *hiv; int hivlen;	// hash key and IVs (if any)
};

struct hbsec_ch_data {
	const void    *new_iv;		//新的初始化向量
	int            iv_offset;	//初始化向量偏移
	int            pre_aad;		//前附加认证数据
	int            post_aad;	//后附加认证数据
	int            src_offset;	//源数据偏移
	int            dst_offset;	//目的数据偏移
	unsigned char *src; unsigned long src_len;	//源数据地址，源数据长度
	unsigned char *dst; unsigned long dst_len;	//目的数据地址，目的数据长度
};

struct hbsec_ch_feature {
	unsigned project;	//Secure Engine硬件项目名
	unsigned partial;	//Secure Engine硬件局部编号
	unsigned version;	//Secure Engine硬件版本号
	unsigned ivimport;  //是否使用iv import
	unsigned qos;       //是否使能QoS
	unsigned max_msg_size; //最大消息长度
	unsigned char modes[CRYPTO_MODE_LAST]; //所有支持模式
};

#define MAX_BUF_LEN 0x8000

int hbsec_ch_init(struct hbsec_ch *handle, struct hbsec_ch_conf *conf);
int hbsec_ch_deinit(struct hbsec_ch *handle);
int hbsec_ch_process(struct hbsec_ch *handle, struct hbsec_ch_data *proc_data);
int hbsec_ch_feature(struct hbsec_ch *handle, struct hbsec_ch_feature *feature);
int hbsec_ch_isenabled(struct hbsec_ch *handle, int mode, int keysize);
int hbsec_ch_isenabled2(struct hbsec_ch_feature *feature, int mode, int keysize);
int hbsec_ch_register(struct hbsec_ch *handle);
int hbsec_ch_bind(struct hbsec_ch *handle, struct hbsec_ch *new_handle);

#define dbg(format, x...) printf("[%s:%d] " format "\n" , __func__, __LINE__, ##x)

#include <stdint.h>
static inline void hexdump(uint8_t *buf, int len)
{
	int i;
	if (buf == NULL || len <= 0) {
		printf("%s wrong params: buf:%p, len:%d\n", __func__, buf, len);
		return;
	}

	for (i = 0; i < len ; i++ ) {
		printf("%02x ", buf[i]);
		if ((i+1)%16 == 0) printf("\n");
	}
	printf("\n");
}

#endif
