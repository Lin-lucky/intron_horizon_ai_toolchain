#ifndef __KEROS_I2C_INTERFACE_H__
#define __KEROS_I2C_INTERFACE_H__

#define ERROR_CODE_TRUE			0
#define ERROR_CODE_FALSE		1
#define ERROR_CODE_WRITE_ADDR	10
#define ERROR_CODE_WRITE_DATA	20
#define ERROR_CODE_READ_ADDR	30
#define ERROR_CODE_READ_DATA	40
#define ERROR_CODE_START_BIT	50
#define ERROR_CODE_APROCESS		60
#define ERROR_CODE_DENY			70

#define I2C_M_RD  				0x0001
#define I2C_RETRIES 			0x0701
#define I2C_TIMEOUT 			0x0702
#define I2C_RDWR    			0x0707

struct keros_i2c_msg {
    unsigned short addr;
    unsigned short flags;
    unsigned short len;
    unsigned char *buf;
};

struct keros_i2c_ioctl_data {
    struct keros_i2c_msg *msgs;
    unsigned int nmsgs;
};

int keros_i2c_open(const char * dev_node);
void keros_i2c_close(int fd);
int keros_i2c_read(int fd, unsigned char dev_addr, unsigned char *sub_addr, unsigned char *buff, int n_bytes);
int keros_i2c_write(int fd, unsigned char dev_addr, unsigned char *sub_addr, unsigned char *buff, int n_bytes);

#endif
