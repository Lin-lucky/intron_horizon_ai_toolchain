// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <iostream>

#include "dnn/hb_sys.h"

int main(int argc, char **argv) {
  std::cout << "bpu memory example start!" << std::endl;
  // Noncacheable memory
  hbSysMem mem;
  hbSysAllocMem(&mem, 1024);
  // Flush to bpu (Useless operation, nothing happened actually)
  hbSysFlushMem(&mem, HB_SYS_MEM_CACHE_INVALIDATE);
  // Flush to cpu (Useless operation, nothing happened actually)
  hbSysFlushMem(&mem, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFreeMem(&mem);

  // Cacheable memory
  hbSysMem cacheabled_mem;
  hbSysAllocCachedMem(&cacheabled_mem, 1024);
  // Flush to bpu
  hbSysFlushMem(&cacheabled_mem, HB_SYS_MEM_CACHE_INVALIDATE);
  // Flush to cpu
  hbSysFlushMem(&cacheabled_mem, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFreeMem(&cacheabled_mem);
  std::cout << "bpu memory example success!" << std::endl;
}
