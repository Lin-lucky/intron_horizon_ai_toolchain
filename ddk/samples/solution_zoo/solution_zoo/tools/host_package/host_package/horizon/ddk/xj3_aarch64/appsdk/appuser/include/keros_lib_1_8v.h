/*
* @brief KEROS LIB Driver
*
* @note
* Copyright(C) CHIPSBRAIN CO., Ltd., 1999 ~ 2016
 * All rights reserved.
*
* File Name 	: keros_lib_1_8v.h
* Author		: dennis lim
*
* Version	: V1.17
* Date 		: 2015.08.07
* Description : Keros LIB Header
*/
#ifndef __KEROS_LIB_1_8v_H_
#define __KEROS_LIB_1_8v_H_

#define MCU_TYPE_8BIT		0
#define MCU_TYPE_16BIT	1
#define MCU_TYPE_32BIT	2

#define MCU_TYPE	MCU_TYPE_32BIT

#ifndef CONST
#define CONST           const
#endif

#ifndef uint8_t
typedef unsigned          char uint8_t;
#endif

#ifndef uint16_t
typedef unsigned short     int uint16_t;
#endif

#ifndef uint32_t
#if (MCU_TYPE == MCU_TYPE_8BIT)
typedef unsigned           long uint32_t;		// 8BIT
#endif
#if (MCU_TYPE == MCU_TYPE_16BIT)
typedef unsigned           long uint32_t;	// 16BIT
#endif
#if (MCU_TYPE == MCU_TYPE_32BIT)
typedef unsigned           int uint32_t;		// 32BIT
#endif
#endif

#ifndef NULL
#define NULL	(void*)0
#endif

#ifndef TRUE
#define TRUE	1
#endif

#ifndef FALSE
#define FALSE	0
#endif

#ifdef __cplusplus
extern "C"
{
#endif

	typedef enum {
	    KEROS_STATUS_OK = 0,
	    KEROS_STATUS_FAILED = 1,
	    KEROS_NOT_INITIALIZE_LIB = 2,
	    KEROS_NOT_SUPPORT_MODE = 3,
	    KEROS_NOT_SUPPORT_PAGE_INDEX = 4,
	    KEROS_NOT_SUPPORT_BLOCK_INDEX = 5,
      KEROS_NOT_SUPPORT_WRITE_PAGE = 6,
      KEROS_NOT_SUPPORT_READ_PAGE = 7,
      KEROS_NOT_SUPPORT_CHANGE_PASSWORD_IN_PAGE = 8,
      KEROS_FAILED_CHANGE_PASSWORD = 9,
      KEROS_ERROR_SW_AES_ENC_DEC	= 10,
      KEROS_NOT_MACHED_AES_KEY_SIZE = 11,
      KEROS_NULL_POINT_EXCEPTION = 12,
      KEROS_NOT_SUPPORT_BLOCK = 13,
      KEROS_ERROR_DOUBLE_BLOCK_EMPTY	= 14,
      KEROS_ERROR_AUTHENTICATION	= 15,
      KEROS_OK_AUTHENTICATION	= 16,
      KEROS_NOT_SUPPORT_BLOCK_SIZE = 17,
      KEROS_FAILED_PASSWORD = 18,
	} keros_STATUS_T;

#define KEROS_SHUTDOWN				0
#define KEROS_DELAY_SHUTDOWN	1

#define AES_REQ_ENCODING				0
#define AES_REQ_DECODING				1

#define SET_AES_KEY_SIZE_128			0
#define SET_AES_KEY_SIZE_192			1
#define SET_AES_KEY_SIZE_256			2

#define REQ_AES_NONE_WRITE			0
#define REQ_AES_ENCODING_WRITE	1

#define REQ_AES_NONE_READ			0
#define REQ_AES_ENCODING_READ	1


#define MAX_EEPROM_BLOCK_NUM	16
#define EEPROM_BANK_LEN				64
#define MAX_AES_BUFFER_SIZE			16

	extern uint8_t keros_read_data( uint16_t sub_addr, int read_len, uint8_t * r_data );
	extern uint8_t keros_write_data( uint16_t sub_addr, uint8_t * w_data, int write_len );
	extern void keros_delay( uint32_t wait_time );
	extern uint8_t keros_power_on( void );

	/**
	 * name : uint8_t keros_init_1_8v(uint8_t *r_seral_data)
	 * @brief	: KEROS Initialize LIB
	 * @param	 r_seral_data	: Pointer to serial number of KEROS
	 * @return	On success returns KEROS_STATUS_OK.
	 *
	 */
	uint8_t keros_init_1_8v( uint8_t * r_seral_data );

	/**
	 * name : uint8_t get_lib_version_1_8v(uint8_t *pVer, uint8_t *len);
	 * @brief	: get information of LIB
	 * @param	 pVer : Pointer to version information of LIB
	 * @param	 len	: length to version information of LIB
	 * @return	On success returns KEROS_STATUS_OK.
	 *
	 */
	uint8_t get_lib_version_1_8v( uint8_t * pVer, uint8_t  *len );


	/**
	 * name : uint8_t keros_eeprom_read_1_8v( uint8_t page, uint8_t *r_data, uint8_t encrytion )
	 * @brief	: read EEPROM page data
	 * @param	 page : page index to read EEPROM data (0 ~ 29)
	 * @param	 r_data : pointer to read EEPROM ( mininum buffer size are 64)
	 * @param	 encrytion : read data are required aes decryption.
	 *                                      0 : require raw data
	 *                                      1 : require decrypted data.
	 * @return	On success returns KEROS_STATUS_OK.
	 *            On not initialize lib returns KEROS_NOT_INITIALIZE_LIB
	 *            On page_index greate than 29 returns KEROS_NOT_SUPPORT_PAGE_INDEX
	 *            On Support Inkjet Mode and page_index less than Range Inkjet Area than returns KEROS_NOT_SUPPORT_READ_PAGE
	 *
	 */

	uint8_t keros_eeprom_read_1_8v( uint32_t password, uint8_t page, uint8_t *r_data, uint8_t encrytion );

	/**
	 * name : uint8_t keros_eeprom_write_1_8v( uint8_t page, uint8_t *w_data, uint8_t encrytion )
	 * @brief	: write EEPROM page data
	 * @param	 uint8_t keros_eeprom_write( uint8_t page, uint8_t *w_data, uint8_t encrytion ) : page index to write EEPROM data (0 ~ 29)
	 * @param	 w_data : pointer to read EEPROM( buffer size are 64)
	 * @param	 encrytion : write data are required aes encryption.
	 *                                      0 : require raw data
	 *                                      1 : require encryption data.
	 * @return	On success returns KEROS_STATUS_OK.
	 *            On not initialize lib returns KEROS_NOT_INITIALIZE_LIB
	 *            On page_index greate than 29 returns KEROS_NOT_SUPPORT_PAGE_INDEX
	 *            On Support Inkjet Mode and page_index less than Range Inkjet Area than returns KEROS_NOT_SUPPORT_WRITE_PAGE
	 *
	 */
	uint8_t keros_eeprom_write_1_8v( uint32_t password, uint8_t page, uint8_t *w_data, uint8_t encrytion );

	/**
	 * name : keros_eeprom_pwchg_1_8v( uint8_t page, uint32_t old_password, uint32_t new_password )
	 * @brief	: change password of EEPROM Block
	 * @param	 page : index to EEPROM Block(0 ~ 29)
   * @param	 old_password : old password
	 * @param	 new_password : new password
	 * @return	On success returns KEROS_STATUS_OK.
	 *            On not initialize lib returns KEROS_NOT_INITIALIZE_LIB
	 *            On block_index greate than 14 returns KEROS_NOT_SUPPORT_BLOCK_INDEX
	 *            On Support Inkjet Mode and block_index less than Range Inkjet Area than returns KEROS_NOT_SUPPORT_CHANGE_PASSWORD_IN_PAGE
	 *
	 */
	uint8_t keros_eeprom_pwchg_1_8v( uint8_t page, uint32_t old_password, uint32_t new_password );



	/**
	 * name : uint8_t keros_req_enc_dec_1_8v(uint8_t *req_data, uint8_t *result_data, uint8_t mode);
	 * @brief	: Request Encryption and Decryption data
	 * @param	 req_data : request data for encryption or decryption (must be buffer length is 16)
	 * @param	 result_data : return result data for encryption or decryption (must be buffer length is 16)
	 * @param	 mode : 	AES_REQ_ENCODING
	 * 				AES_REQ_DECODING
	 * @return	On success returns KEROS_STATUS_OK.
	 *            On not initialize lib returns KEROS_NOT_INITIALIZE_LIB
	 *            On is_req_encryption_to_write greate than 2 returns KEROS_NOT_SUPPORT_MODE
	 *
	 */
	uint8_t keros_req_enc_dec_1_8v( uint8_t * req_data, uint8_t * result_data, uint8_t mode );

	/**
	 * name : uint8_t keros_bypass_mode_1_8v(uint8_t *req_bypass_data,  uint8_t *bypassed_data);
	 * @brief	: Request Bypass Mode
	 * @param	 req_bypass_data : request source data for Bypass (must be buffer length is 16)
	 * @param	 bypassed_data : return result data for Bypass (must be buffer length is 16)
	 * @return	On success returns KEROS_STATUS_OK.
	 *            On not initialize lib returns KEROS_NOT_INITIALIZE_LIB
	 *
	 */
	uint8_t keros_bypass_mode_1_8v( uint8_t * req_bypass_data, uint8_t * bypassed_data );

	/**
	 * name : uint8_t keros_set_aes_key_size_1_8v(uint8_t aes_key_size);
	 * @brief	: Request Set Aes Key Size
	 * @param	 aes_key_size : 	SET_AES_KEY_SIZE_128
	 * 						SET_AES_KEY_SIZE_192
	 *						SET_AES_KEY_SIZE_256
	 * @return	On success returns KEROS_STATUS_OK.
	 *            On not initialize lib returns KEROS_NOT_INITIALIZE_LIB
	 *            On mode greate than 1 returns KEROS_NOT_SUPPORT_MODE
	 *
	 */
	uint8_t keros_set_aes_key_size_1_8v( uint8_t aes_key_size );

	/**
	 * name : uint8_t keros_read_inkjet_counter_1_8v(uint16_t *counter_value);
	 * @brief	: read inkjet counter
	 * @param	 counter_value : inkjet counter
	 * @return	On success returns KEROS_STATUS_OK.
	 *            On not initialize lib returns KEROS_NOT_INITIALIZE_LIB
	 *            On not enabled Inkjet mode returns KEROS_NOT_SUPPORT_MODE
	 *
	 */
	uint8_t keros_read_inkjet_counter_1_8v( uint16_t * counter_value );

	/**
	 * name : uint8_t keros_inc_inkjet_counter_1_8v(void);
	 * @brief	: increment inkjet counter
	 * @return	On success returns KEROS_STATUS_OK.
	 *            On not initialize lib returns KEROS_NOT_INITIALIZE_LIB
	 *            On not enabled Inkjet mode returns KEROS_NOT_SUPPORT_MODE
	 *
	 */
	uint8_t keros_inc_inkjet_counter_1_8v( void );

	/**
	 * name : uint8_t keros_powersave_1_8v(uint8_t cmd, uint32_t time)
	 * @brief	: power off
	 * @param	 cmd : KEROS_SHUTDOWN, KEROS_DELAY_SHUTDOWN
	 * @param	 time : if(cmd == KEROS_SHUTDOWN) NONE
	 *                if(cmd == KEROS_DELAY_SHUTDOWN) valid(20 ~ 1400) / unit : ms
	 * @return	On success returns KEROS_STATUS_OK.
	 *
	 */
	uint8_t keros_powersave_1_8v(uint8_t cmd, uint32_t time);

	/**
	 * name : void keros_srand(uint32_t seed);
	 * @brief	: set random seed
	 * @param	 seed : random seed
	 * @return	NONE.
	 *
	 */
	void keros_srand_1_8v( uint32_t seed );

	/**
	 * name : vuint8_t keros_random_1_8v( void );
	 * @brief	: random value generation
	 * @return :	random value.
	 *
	 */
	uint8_t keros_random_1_8v( void );

	/**
	 * name : uint8_t keros_authorization_1_8v(uint8_t aes_key_size, uint32_t seed, uint8_t *raw_data);
	 * @brief	: authorization
	 * @param	 aes_key_size :
     *                      SET_AES_KEY_SIZE_128
	 * 			         SET_AES_KEY_SIZE_192
	 *			             SET_AES_KEY_SIZE_256
	 * @param	 seed : random seed
	 *                             0 : it is not used as  random SEED value.
	 *                Non-zero : it is used as random SEED value.
     * @param	 raw_data : input data(must be buffer length is 16)
     * @return	TRUE : success
     *               FALSE : Fail
	 *
	 */
   uint8_t keros_authentication_1_8v(uint8_t aes_key_size, uint32_t seed, uint8_t *indata);

#ifdef __cplusplus
}

#endif

#endif /* __KEROS_LIB_H_ */
