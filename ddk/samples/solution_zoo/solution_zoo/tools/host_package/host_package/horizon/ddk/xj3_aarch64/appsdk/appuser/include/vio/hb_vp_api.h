/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2016-2018 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef __HB_VP_H
#define __HB_VP_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

// #include <stdint.h>
#include "ion.h"
#include "hb_errno.h"
#include "hb_sys.h"

#define VP_MAX_PUB_POOLS 16 

enum HB_VP_ERR_E {
	ERR_VP_INVALID_BLOCKID = 64,
	ERR_VP_INVALID_POOLID,
	ERR_VP_NOT_PERM,
	ERR_VP_UNEXIST,
	ERR_VP_BUSY,
	ERR_SYS_BUSY,
	ERR_SYS_ILLEGAL_PARAM,
	ERR_SYS_NOMEM,
	ERR_VP_ILLEGAL_PARAM
};

#define VP_INVALID_BLOCKID	HB_DEF_ERR(HB_ID_SYS, ERR_VP_INVALID_BLOCKID)
#define VP_INVALID_POOLID	HB_DEF_ERR(HB_ID_SYS, ERR_VP_INVALID_POOLID)
#define HB_ERR_VP_NOT_PERM	HB_DEF_ERR(HB_ID_SYS, ERR_VP_NOT_PERM)
#define HB_ERR_VP_UNEXIST	HB_DEF_ERR(HB_ID_SYS, ERR_VP_UNEXIST)
#define HB_ERR_VP_BUSY		HB_DEF_ERR(HB_ID_SYS, ERR_VP_BUSY)
#define HB_ERR_SYS_BUSY		HB_DEF_ERR(HB_ID_SYS, ERR_SYS_BUSY)
#define HB_ERR_SYS_ILLEGAL_PARAM	HB_DEF_ERR(HB_ID_SYS, ERR_SYS_ILLEGAL_PARAM)
#define HB_ERR_SYS_NOMEM	HB_DEF_ERR(HB_ID_SYS, ERR_SYS_NOMEM)
#define HB_ERR_VP_ILLEGAL_PARAM	HB_DEF_ERR(HB_ID_SYS, ERR_VP_ILLEGAL_PARAM)

typedef struct HB_VP_POOL_CONFIG_S {
    uint64_t u64BlkSize;
    uint32_t u32BlkCnt;
    uint32_t cacheEnable;
} VP_POOL_CONFIG_S;

typedef struct HB_VP_CONFIG_S {
    uint32_t u32MaxPoolCnt;
    VP_POOL_CONFIG_S pubPool[VP_MAX_PUB_POOLS];
} VP_CONFIG_S;

typedef struct HB_VP_AUXILIARY_CONFIG_S {
    int u32AuxiliaryConfig;
} VP_AUXILIARY_CONFIG_S;

int HB_VP_SetConfig(VP_CONFIG_S *VpConfig);
int HB_VP_GetConfig(VP_CONFIG_S *VpConfig);
int HB_VP_Init(void);
int HB_VP_Exit(void);
uint32_t HB_VP_CreatePool(VP_POOL_CONFIG_S *VpPoolCfg);
int HB_VP_DestroyPool(uint32_t Pool);
uint32_t HB_VP_GetBlock(uint32_t Pool, uint64_t u64BlkSize);
int HB_VP_ReleaseBlock(uint32_t Block);
uint32_t HB_VP_PhysAddr2Block(uint64_t u64PhyAddr);
uint64_t HB_VP_Block2PhysAddr(uint32_t Block);
uint32_t HB_VP_Block2PoolId(uint32_t Block);
int HB_VP_MmapPool(uint32_t Pool);
int HB_VP_MunmapPool(uint32_t Pool);
int HB_VP_GetBlockVirAddr(uint32_t Pool, uint64_t u64PhyAddr,
                            void **ppVirAddr);
int HB_VP_InquireUserCnt(uint32_t Block);
int HB_VP_SetAuxiliaryConfig
                    (const VP_AUXILIARY_CONFIG_S *pstAuxiliaryConfig);
int HB_VP_DmaCopy(void *dstPaddr, void *srcPaddr, uint32_t len);
int HB_SYS_Alloc(uint64_t *pu64PhyAddr, void **ppVirAddr, uint32_t u32Len);
int HB_SYS_AllocCached(uint64_t *pu64PhyAddr, void **ppVirAddr, uint32_t u32Len);
int HB_SYS_Free(uint64_t u64PhyAddr, void *pVirAddr);
int HB_SYS_CacheInvalidate(uint64_t pu64PhyAddr, void *pVirAddr, uint32_t u32Len);
int HB_SYS_CacheFlush(uint64_t pu64PhyAddr, void *pVirAddr, uint32_t u32Len);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif // __HB_VP_H
