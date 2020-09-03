#ifndef __VM_INTERFACE_H__
#define __VM_INTERFACE_H__

#define SDM_COMP_SERVICE_ID 5010
#define SDM_COMP_SERVICE_VERSION 1
#define SDM_COMP_SERVICE_INSTANCE 1

enum CommandId {
  kCmdExportDemuraBuffers = 0,
  kCmdMax,
};

typedef struct {
  int cfg_mem_hdl = -1;
  int hfc_mem_hdl = -1;
} DemuraMemHandle;

typedef struct {
  int32_t id = -1;
} CommandHeader;

typedef struct {
  int32_t id = -1;
  int32_t status = -1;
} ResponseHeader;

// Command and Response structure definitions to start TUI session
typedef struct {
  union {
    DemuraMemHandle demura_mem_handle;
    uint32_t reserve[128] = { 0 };
  };
} CmdExportDemuraBuffer;

typedef struct {
  union {
    uint32_t reserve[128] = { 0 };
  };
} RspExportDemuraBuffer;

struct Command : CommandHeader {
  Command() {}
  union {
    CmdExportDemuraBuffer cmd_export_demura_buf;
  };
};

struct Response : ResponseHeader {
  Response() {}
  union {
    RspExportDemuraBuffer rsp_export_demura_buf;
  };
};

#endif // __VM_INTERFACE_H__

