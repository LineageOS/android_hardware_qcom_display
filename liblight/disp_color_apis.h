#ifndef __DISP_COLOR_APIS_H__
#define __DISP_COLOR_APIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <sys/types.h>
#include <cutils/log.h>

typedef uint64_t DISPAPI_HANDLE;

int32_t disp_api_init(DISPAPI_HANDLE *hctx, uint32_t flags);

int32_t disp_api_set_panel_brightness_level_ext(DISPAPI_HANDLE hctx, uint32_t disp_id, int32_t level,
                                          uint32_t flags);

#ifdef __cplusplus
}
#endif

#endif // __DISP_COLOR_APIS_H__