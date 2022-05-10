
#ifndef SPI_SERVICE_H
#define SPI_SERVICE_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "classserver.h"
#include "classclient.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define SPI_SERVICE_MAX_LENGTH (4096u - 4u)

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/
enum {
	SERVICE_TYPE_SPI_SLAVE = 0,
	SERVICE_TYPE_SPI_MASTER = 1,
	SERVICE_TYPE_BIF_SPI = 2,
	SERVICE_TYPE_ETH = 3,
	SERVICE_TYPE_NUM
};

typedef enum {
  SPI_SERVICE_STATE_UNINIT = 0,
  SPI_SERVICE_STATE_INIT = 1,
  SPI_SERVICE_STATE_HANDSHAKE = 2,
  SPI_SERVICE_STATE_RUN = 3,
  SPI_SERVICE_STATE_DEINIT = 4,
  SPI_SERVICE_STATE_NUM
} SpiServiceType;

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

typedef struct {
  uint32_t id; /* CNA id */
  uint8_t dlc; /* CAN dlc */
  uint8_t unused[3];
  uint8_t data[8]; /* CAN data */
} CanFrame_st;

typedef struct {
  uint16_t type;            /* Message type */
  uint16_t length;          /* Data length */
  uint32_t protocol_id;
  uint8_t timestamp[8];
} MsgHeader_st;

typedef struct {
  MsgHeader_st header;
  long timestamp;
  uint8_t unused[1000];
} MsgTimestamp_st;

typedef struct {
  int length;
  uint8_t data[SPI_SERVICE_MAX_LENGTH];
} MsgOut_st;

typedef struct {
  MsgHeader_st header;
  uint8_t data[SPI_SERVICE_MAX_LENGTH];
} MsgGeneral_st;

typedef struct {
  MsgHeader_st header;
  uint16_t sub;
  uint8_t data[SPI_SERVICE_MAX_LENGTH];
} MsgSubSvc_st;

typedef int (*svcFunc)(uint8_t *, int);
typedef int (*svcSubFunc)(uint8_t *, int);

typedef struct {
  uint16_t svcType;
  svcFunc svcHandler;
} SpiServiceTab_st;

typedef struct {
  uint16_t subType;
  svcSubFunc subHandler;
} SpiServiceSubTab_st;

typedef struct {
  MsgHeader_st header;
  int64_t ts;
  uint8_t data[4000];
} MsgCANRaw_st;

typedef struct {
  int64_t time_stamp;
  uint8_t counter;
  uint8_t frame_num;
} CANHeader_st;

typedef struct {
  CANHeader_st header;
  uint8_t data[4000];
} MsgCANFrames_st;

typedef struct {
  uint16_t sub;
  uint8_t cmd[3];
  uint8_t unused[3]; 
} DiagRead_st;

typedef struct {
  MsgHeader_st header;
  DiagRead_st diag_cmd;
} MsgDiagRead_st;

typedef struct {
  MsgHeader_st header;
  uint8_t code;
  uint8_t unused[3];
} MsgHandshakeSync_st;

typedef struct {
  MsgHeader_st header;
  uint8_t sync;
  uint8_t ack;
  uint8_t unused[2];
} MsgHandshakeAck_st;

// sub system
typedef struct {
    MsgHeader_st header;
    uint8_t      answer;
    uint8_t      unused[3];
} MsgAnswer_st;

typedef struct {
    MsgHeader_st header;
    uint16_t     sub;
    uint8_t      unused[2];
    uint16_t     state;
    uint8_t      unused1[2];
} MsgMgrState_st;

// typedef struct{
//     float cpu_load;
//     int cpu_temp;
//     float  cpu_mem;
// } CPU_data_t;

typedef struct{
    float cpu_load;
    int cpu_temp;
    float cpu_mem;
    int bpu0_load;
    int bpu1_load;
    int sensor_temp;
} CPU_data_t;

typedef struct {
    MsgHeader_st header;
    uint16_t     sub;
    uint8_t     unused[2];
    CPU_data_t   cpu_data;
} MsgMgrCPUState_st;

// system head
typedef struct {
  uint16_t version;
  uint16_t type;
  uint16_t length;
} SysMgrHeader_t;

// PUB system message


typedef struct {
  SysMgrHeader_t header;
  uint8_t filter[6];
  CPU_data_t cpu_data;
} SysMgrCPUState_st;

typedef struct {
    SysMgrHeader_t header;
    uint8_t  filter[2];
    uint8_t  question;
    uint8_t  unused;
} SysMgrQA_st;

typedef struct {
    SysMgrHeader_t header;
    uint8_t  filter[2];
    uint8_t  answer;
    uint8_t  unused;
} SysMgrAns_st;

typedef struct {
    SysMgrHeader_t header;
    uint8_t  filter[6];
    uint16_t state;
} SysMgrState_st;

typedef struct {
  float camera_x;
  float camera_y;
  float camera_z;
  float pitch;
  float yaw;
  float roll;
  uint8_t update;
  uint8_t unused[3];
} CalibExtrinInfo_st;

typedef struct {
    SysMgrHeader_t header;
    uint8_t  filter[6];
    CalibExtrinInfo_st info;
} SysMgrExtrin_st;

typedef struct {
  uint32_t state;
  float param[7];
} CalibExtrin_st;

typedef struct {
  int32_t type;
  int32_t state;
  float param[7];
} SocCalibExtrin_st;

typedef struct {
  MsgHeader_st header;
  uint16_t  sub;
  uint8_t unused[2];
  CalibExtrin_st info;
} CalibExtrinMsg_st;
/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
class ServiceManager {
 public:
  ServiceManager(void) {}
  ~ServiceManager(void) {}
  virtual int service_manager_state_machine(void);
  int spi_service_set_state(SpiServiceType state);
  int spi_service_get_state(SpiServiceType * state);
  SpiServiceType spi_service_state;
};

class ServiceManagerSpiSlave: public ServiceManager {
 public:
  ServiceManagerSpiSlave(int type) {
    type_ = type;
  }
  ~ServiceManagerSpiSlave(void) {}
  virtual int service_manager_state_machine(void);
  int spi_service_handshake_process(void);
  int type_;
};

class PubCANInput: public Pub {
 public:
  PubCANInput(std::string filepath):Pub(filepath) {}
  ~PubCANInput(void) {}
  int IpcCANInputPub(uint8_t * data, int length);
};

class SubCANInput: public Sub{
 public:
  SubCANInput(std::string filepath):Sub(filepath) {
  }
  ~SubCANInput() {}
  int IpcCANInputSub(void);
};

class SubCANOutput: public Sub{
 public:
  SubCANOutput(std::string filepath):Sub(filepath) {
  }
  ~SubCANOutput() {}
  int IpcCANOutputSub(void);
  int IpcCANOutputExtraSub(void);
};

class SubCalibOutput: public Sub{
 public:
  SubCalibOutput(std::string filepath):Sub(filepath) {
  }
  ~SubCalibOutput() {}
  int IpcCalibOutputSub(void);
};

class PubDiagIn: public Pub {
 public:
  PubDiagIn(std::string filepath):Pub(filepath) {
    
  }
  ~PubDiagIn() {}
  int IpcDiagInPub(uint8_t * data, int length);
};

class PubGpsIn: public Pub {
 public:
  PubGpsIn(std::string filepath):Pub(filepath) {
  }
  ~PubGpsIn() {}
  int IpcGpsInPub(uint8_t * data, int length);
};

class PubImuIn: public Pub {
 public:
  PubImuIn(std::string filepath):Pub(filepath) {
  }
  ~PubImuIn() {}
  int IpcImuInPub(uint8_t * data, int length);
};


class SubDiagOut: public Sub{
 public:
  SubDiagOut(std::string filepath):Sub(filepath) {
  }
  ~SubDiagOut() {}
  int IpcDiagOutSub(void);
};

class PubQAIn: public Pub {
 public:
  PubQAIn(std::string filepath):Pub(filepath) {
  }
  ~PubQAIn() {}
  int IpcQAInPub(uint8_t * data, int length);
  int IpcSocStateInPub(uint8_t * data, int length);
  int IpcCalibExtrinInPub(uint8_t * data, int length);
};

class SubQAOut: public Sub{
 public:
  SubQAOut(std::string filepath):Sub(filepath) {
  }
  ~SubQAOut() {}
  int IpcQAOutSub(void);
};

class PubTimesyncIn: public Pub {
 public:
  PubTimesyncIn(std::string filepath):Pub(filepath) {
    
  }
  ~PubTimesyncIn() {}
  int IpcTimesyncInPub(uint8_t * data, int length);
};


class PubUtityIn: public Pub {
 public:
  PubUtityIn(std::string filepath):Pub(filepath) {
    
  }
  ~PubUtityIn() {}
  int IpcUtityInPub(uint8_t * data, int length);
};

class SubUtityOut: public Sub{
 public:
  SubUtityOut(std::string filepath):Sub(filepath) {
  }
  ~SubUtityOut() {}
  int IpcUtityOutSub(void);
};

/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

#endif  // SPI_SERVICE_H
