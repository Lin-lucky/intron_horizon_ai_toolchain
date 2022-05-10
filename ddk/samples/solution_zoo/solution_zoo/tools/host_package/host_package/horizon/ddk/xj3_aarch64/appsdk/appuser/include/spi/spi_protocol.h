#ifndef _SPI_J2_MCU_H_
#define _SPI_J2_MCU_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
* spi_slave_dbus_init - open and configure spi_slave_dbus module according to specification
* @block[IN]: open spi_dbus module with block(block = 1) or unblock(block = 0) mode
* return: 0 on success; if an error occurred return a negative value
*       -1: spi device does not exist
*       -2: initialize fail
*/
//int spi_slave_init(int block);
int spi_protocol_init(int block,int index);
int spi_master_init(int block);

/*
* spi_slave_dbus_exit - close spi_slave_dbus module
* return: void
*/
//void spi_slave_exit(void);
int spi_protocol_exit(int index);
void spi_master_exit(void);

/*
* spi_dbus_slave_send_frame - send one user frame with spi_slave_dbus module
* frame[IN]: pointer to buffer which contains user frame need send
* len[IN]: user frame length want to send
* return: 0 on success; or -1 if an error occurred
*/
//int spi_slave_send_frame(const char *frame, int len);
int spi_protocol_send_frame(const char *frame, int len,int index);
int spi_master_send_frame(const char *frame, int len);

/*
* spi_slave_dbus_recv_frame - receive one user frame with spi_slave_dbus module
* buff[OUT]: pointer to buffer which contains user data from spi_slave_dbus module
* len[IN]: data length want to receive
* return: the number of byte receive on success; or -1 if an error occurred
*/
//int spi_slave_recv_frame(char *buff, int len);
int spi_protocol_recv_frame(char *buff, int len, int index);
int spi_master_recv_frame(char *buff, int len);


#ifdef __cplusplus
}
#endif

#endif


