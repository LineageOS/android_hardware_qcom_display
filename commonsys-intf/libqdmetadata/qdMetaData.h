/*
 * Copyright (c) 2012-2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _QDMETADATA_H
#define _QDMETADATA_H

#include <color_metadata.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_UBWC_STATS_LENGTH 32
#define GRAPHICS_METADATA_SIZE 4096
#define CVP_METADATA_SIZE 1024

enum ColorSpace_t{
    ITU_R_601,
    ITU_R_601_FR,
    ITU_R_709,
    ITU_R_2020,
    ITU_R_2020_FR,
};

enum IGC_t {
    IGC_NotSpecified,
    IGC_sRGB,
};

struct HSICData_t {
    int32_t hue;
    float   saturation;
    int32_t intensity;
    float   contrast;
};

struct BufferDim_t {
    int32_t sliceWidth;
    int32_t sliceHeight;
};

enum UBWC_Version {
    UBWC_UNUSED      = 0,
    UBWC_1_0         = 0x1,
    UBWC_2_0         = 0x2,
    UBWC_3_0         = 0x3,
    UBWC_4_0         = 0x4,
    UBWC_MAX_VERSION = 0xFF,
};

struct UBWC_2_0_Stats {
    uint32_t nCRStatsTile32;  /**< UBWC Stats info for  32 Byte Tile */
    uint32_t nCRStatsTile64;  /**< UBWC Stats info for  64 Byte Tile */
    uint32_t nCRStatsTile96;  /**< UBWC Stats info for  96 Byte Tile */
    uint32_t nCRStatsTile128; /**< UBWC Stats info for 128 Byte Tile */
    uint32_t nCRStatsTile160; /**< UBWC Stats info for 160 Byte Tile */
    uint32_t nCRStatsTile192; /**< UBWC Stats info for 192 Byte Tile */
    uint32_t nCRStatsTile256; /**< UBWC Stats info for 256 Byte Tile */
};

struct UBWCStats {
    enum UBWC_Version version; /* Union depends on this version. */
    uint8_t bDataValid;      /* If [non-zero], CR Stats data is valid.
                               * Consumers may use stats data.
                               * If [zero], CR Stats data is invalid.
                               * Consumers *Shall* not use stats data */
    union {
        struct UBWC_2_0_Stats ubwc_stats;
        uint32_t reserved[MAX_UBWC_STATS_LENGTH]; /* This is for future */
    };
};

struct S3DGpuComp_t {
    int32_t displayId; /* on which display S3D is composed by client */
    uint32_t s3dMode; /* the S3D format of this layer to be accessed by client */
};

typedef struct GraphicsMetadata {
    uint32_t size;
    uint32_t data[GRAPHICS_METADATA_SIZE];
} GraphicsMetadata;

#define VIDEO_HISTOGRAM_STATS_SIZE (4 * 1024)
/* Frame type bit mask */
#define QD_SYNC_FRAME (0x1 << 0)
struct VideoHistogramMetadata {
    uint32_t stats_info[1024]; /* video stats payload */
    uint32_t stat_len; /* Payload size in bytes */
    uint32_t frame_type; /* bit mask to indicate frame type */
    uint32_t display_width;
    uint32_t display_height;
    uint32_t decode_width;
    uint32_t decode_height;
    uint32_t reserved[12];
};

enum CVPMetadataFlags {
    /* bit wise flags */
    CVP_METADATA_FLAG_NONE              = 0x00000000,
    CVP_METADATA_FLAG_REPEAT            = 0x00000001,
};

typedef struct CVPMetadata {
    uint32_t size; /* payload size in bytes */
    uint8_t payload[CVP_METADATA_SIZE];
    uint32_t capture_frame_rate;
    /* Frame rate in Q16 format.
            Eg: fps = 7.5, then
            capture_frame_rate = 7 << 16 --> Upper 16 bits to represent 7
            capture_frame_rate |= 5 -------> Lower 16 bits to represent 5

       If size > 0, framerate is valid
       If size = 0, invalid data, so ignore all parameters */
    uint32_t cvp_frame_rate;
    enum CVPMetadataFlags flags;
    uint32_t reserved[8];
} CVPMetadata;

struct MetaData_t {
    int32_t operation;
    int32_t interlaced;
    struct BufferDim_t bufferDim;
    float refreshrate;
    enum ColorSpace_t colorSpace;
    enum IGC_t igc;
     /* Gralloc sets PRIV_SECURE_BUFFER flag to inform that the buffers are from
      * ION_SECURE. which should not be mapped. However, for GPU post proc
      * feature, GFX needs to map this buffer, in the client context and in SF
      * context, it should not. Hence to differentiate, add this metadata field
      * for clients to set, and GPU will to read and know when to map the
      * SECURE_BUFFER(ION) */
    int32_t mapSecureBuffer;
    /* The supported formats are defined in gralloc_priv.h to
     * support legacy code*/
    uint32_t s3dFormat;
    /* VENUS output buffer is linear for UBWC Interlaced video */
    uint32_t linearFormat;
    /* Set by graphics to indicate that this buffer will be written to but not
     * swapped out */
    uint32_t isSingleBufferMode;
    /* Indicate GPU to draw S3D layer on dedicate display device */
    struct S3DGpuComp_t s3dComp;

    /* Set by camera to program the VT Timestamp */
    uint64_t vtTimeStamp;
    /* Color Aspects + HDR info */
    ColorMetaData color;
    /* Consumer should read this data as follows based on
     * Gralloc flag "interlaced" listed above.
     * [0] : If it is progressive.
     * [0] : Top field, if it is interlaced.
     * [1] : Do not read, if it is progressive.
     * [1] : Bottom field, if it is interlaced.
     */
    struct UBWCStats ubwcCRStats[2];
    /* Set by camera to indicate that this buffer will be used for a High
     * Performance Video Usecase */
    uint32_t isVideoPerfMode;
    /* Populated and used by adreno during buffer size calculation.
     * Set only for RGB formats. */
    GraphicsMetadata graphics_metadata;
    /* Video hisogram stats populated by video decoder */
    struct VideoHistogramMetadata video_histogram_stats;
    /*
     * Producer (camera) will set cvp metadata and consumer (video) will
     * use it. The format of metadata is known to producer and consumer.
     */
    CVPMetadata cvpMetadata;
};

enum DispParamType {
    SET_VT_TIMESTAMP           = 0x0001,
    COLOR_METADATA             = 0x0002,
    PP_PARAM_INTERLACED        = 0x0004,
    SET_VIDEO_PERF_MODE        = 0x0008,
    SET_GRAPHICS_METADATA      = 0x0010,
    SET_UNUSED                 = 0x0020,
    SET_UBWC_CR_STATS_INFO     = 0x0040,
    UPDATE_BUFFER_GEOMETRY     = 0x0080,
    UPDATE_REFRESH_RATE        = 0x0100,
    UPDATE_COLOR_SPACE         = 0x0200,
    MAP_SECURE_BUFFER          = 0x0400,
    S3D_FORMAT                 = 0x0800,
    LINEAR_FORMAT              = 0x1000,
    SET_IGC                    = 0x2000,
    SET_SINGLE_BUFFER_MODE     = 0x4000,
    SET_S3D_COMP               = 0x8000,
    SET_CVP_METADATA           = 0x00010000,
    SET_VIDEO_HISTOGRAM_STATS  = 0x00020000
};

enum DispFetchParamType {
    GET_VT_TIMESTAMP          = 0x0001,
    GET_COLOR_METADATA        = 0x0002,
    GET_PP_PARAM_INTERLACED   = 0x0004,
    GET_VIDEO_PERF_MODE       = 0x0008,
    GET_GRAPHICS_METADATA     = 0x0010,
    GET_UNUSED                = 0X0020,
    GET_UBWC_CR_STATS_INFO    = 0x0040,
    GET_BUFFER_GEOMETRY       = 0x0080,
    GET_REFRESH_RATE          = 0x0100,
    GET_COLOR_SPACE           = 0x0200,
    GET_MAP_SECURE_BUFFER     = 0x0400,
    GET_S3D_FORMAT            = 0x0800,
    GET_LINEAR_FORMAT         = 0x1000,
    GET_IGC                   = 0x2000,
    GET_SINGLE_BUFFER_MODE    = 0x4000,
    GET_S3D_COMP              = 0x8000,
    GET_CVP_METADATA          = 0x00010000,
    GET_VIDEO_HISTOGRAM_STATS = 0x00020000
};

struct private_handle_t;
int setMetaData(struct private_handle_t *handle, enum DispParamType paramType,
                void *param);
int setMetaDataVa(struct MetaData_t* data, enum DispParamType paramType,
                  void *param);

int getMetaData(struct private_handle_t *handle,
                enum DispFetchParamType paramType,
                void *param);
int getMetaDataVa(struct MetaData_t* data, enum DispFetchParamType paramType,
                  void *param);

int copyMetaData(struct private_handle_t *src, struct private_handle_t *dst);
int copyMetaDataVaToHandle(struct MetaData_t *src, struct private_handle_t *dst);
int copyMetaDataHandleToVa(struct private_handle_t* src, struct MetaData_t *dst);
int copyMetaDataVaToVa(struct MetaData_t *src, struct MetaData_t *dst);

int clearMetaData(struct private_handle_t *handle, enum DispParamType paramType);
int clearMetaDataVa(struct MetaData_t *data, enum DispParamType paramType);

unsigned long getMetaDataSize();

// Map, access metadata and unmap. Used by clients that do not import/free but
//  clone and delete native_handle
int setMetaDataAndUnmap(struct private_handle_t *handle, enum DispParamType paramType,
                        void *param);
int getMetaDataAndUnmap(struct private_handle_t *handle,
                        enum DispFetchParamType paramType,
                        void *param);

#ifdef __cplusplus
}
#endif

#endif /* _QDMETADATA_H */

