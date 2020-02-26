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

#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <log/log.h>
#include <cinttypes>
#include <gralloc_priv.h>
#include "qdMetaData.h"

unsigned long getMetaDataSize() {
    return static_cast<unsigned long>(ROUND_UP_PAGESIZE(sizeof(MetaData_t)));
}

static int validateAndMap(private_handle_t* handle) {
    if (private_handle_t::validate(handle)) {
        ALOGE("%s: Private handle is invalid - handle:%p", __func__, handle);
        return -1;
    }
    if (handle->fd_metadata < 0) {
        // Silently return, metadata cannot be used
        return -1;
    }

    if (!handle->base_metadata) {
        auto size = getMetaDataSize();
        void *base = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED,
                handle->fd_metadata, 0);
        if (base == reinterpret_cast<void*>(MAP_FAILED)) {
            ALOGE("%s: metadata mmap failed - handle:%p fd: %d err: %s",
                __func__, handle, handle->fd_metadata, strerror(errno));

            return -1;
        }
        handle->base_metadata = (uintptr_t) base;
    }
    return 0;
}

static void unmapAndReset(private_handle_t *handle) {
    if (private_handle_t::validate(handle) == 0 && handle->base_metadata) {
        munmap(reinterpret_cast<void *>(handle->base_metadata), getMetaDataSize());
        handle->base_metadata = 0;
    }
}

int setMetaData(private_handle_t *handle, DispParamType paramType,
                void *param) {
    auto err = validateAndMap(handle);
    if (err != 0)
        return err;
    return setMetaDataVa(reinterpret_cast<MetaData_t*>(handle->base_metadata),
                         paramType, param);
}

int setMetaDataVa(MetaData_t *data, DispParamType paramType,
                  void *param) {
    if (data == nullptr)
        return -EINVAL;
    // If parameter is NULL reset the specific MetaData Key
    if (!param) {
       data->operation &= ~paramType;
       // param unset
       return 0;
    }

    data->operation |= paramType;
    switch (paramType) {
        case PP_PARAM_INTERLACED:
            data->interlaced = *((int32_t *)param);
            break;
        case UPDATE_BUFFER_GEOMETRY:
            data->bufferDim = *((BufferDim_t *)param);
            break;
        case UPDATE_REFRESH_RATE:
            data->refreshrate = *((float *)param);
            break;
        case UPDATE_COLOR_SPACE:
            data->colorSpace = *((ColorSpace_t *)param);
            break;
        case MAP_SECURE_BUFFER:
            data->mapSecureBuffer = *((int32_t *)param);
            break;
        case S3D_FORMAT:
            data->s3dFormat = *((uint32_t *)param);
            break;
        case LINEAR_FORMAT:
            data->linearFormat = *((uint32_t *)param);
            break;
        case SET_IGC:
            data->igc = *((IGC_t *)param);
            break;
        case SET_SINGLE_BUFFER_MODE:
            data->isSingleBufferMode = *((uint32_t *)param);
            break;
        case SET_S3D_COMP:
            data->s3dComp = *((S3DGpuComp_t *)param);
            break;
        case SET_VT_TIMESTAMP:
            data->vtTimeStamp = *((uint64_t *)param);
            break;
        case COLOR_METADATA:
            data->color = *((ColorMetaData *)param);
            break;
        case SET_UBWC_CR_STATS_INFO: {
             struct UBWCStats* stats = (struct UBWCStats*)param;
             int numelems = sizeof(data->ubwcCRStats) / sizeof(struct UBWCStats);
             for (int i = 0; i < numelems; i++) {
                  data->ubwcCRStats[i] = stats[i];
             }
              break;
          }
        case SET_VIDEO_PERF_MODE:
            data->isVideoPerfMode = *((uint32_t *)param);
            break;
        case SET_GRAPHICS_METADATA: {
             GraphicsMetadata payload = *((GraphicsMetadata*)(param));
             data->graphics_metadata.size = payload.size;
             memcpy(data->graphics_metadata.data, payload.data,
                    sizeof(data->graphics_metadata.data));
             break;
        }
        case SET_CVP_METADATA: {
             struct CVPMetadata *cvpMetadata = (struct CVPMetadata *)param;
             if (cvpMetadata->size <= CVP_METADATA_SIZE) {
                 data->cvpMetadata.size = cvpMetadata->size;
                 memcpy(data->cvpMetadata.payload, cvpMetadata->payload,
                        cvpMetadata->size);
                 data->cvpMetadata.capture_frame_rate = cvpMetadata->capture_frame_rate;
                 data->cvpMetadata.cvp_frame_rate = cvpMetadata->cvp_frame_rate;
                 data->cvpMetadata.flags = cvpMetadata->flags;
                 memcpy(data->cvpMetadata.reserved, cvpMetadata->reserved,
                        (8 * sizeof(uint32_t)));
             } else {
                 data->operation &= ~(paramType);
                 ALOGE("%s: cvp metadata length %d is more than max size %d",
                     __func__, cvpMetadata->size, CVP_METADATA_SIZE);
                 return -EINVAL;
             }
             break;
        }
        case SET_VIDEO_HISTOGRAM_STATS: {
            struct VideoHistogramMetadata *vidstats = (struct VideoHistogramMetadata *)param;
            if (vidstats->stat_len <= VIDEO_HISTOGRAM_STATS_SIZE) {
                memcpy(data->video_histogram_stats.stats_info,
                    vidstats->stats_info, VIDEO_HISTOGRAM_STATS_SIZE);
                data->video_histogram_stats.stat_len = vidstats->stat_len;
                data->video_histogram_stats.frame_type = vidstats->frame_type;
                data->video_histogram_stats.display_width = vidstats->display_width;
                data->video_histogram_stats.display_height = vidstats->display_height;
                data->video_histogram_stats.decode_width = vidstats->decode_width;
                data->video_histogram_stats.decode_height = vidstats->decode_height;
            } else {
                 data->operation &= ~(paramType);
                 ALOGE("%s: video stats length %u is more than max size %u",
                     __func__, vidstats->stat_len, VIDEO_HISTOGRAM_STATS_SIZE);
                 return -EINVAL;
            }
            break;
         }
         default:
            ALOGE("Unknown paramType %d", paramType);
            break;
    }
    return 0;
}

int clearMetaData(private_handle_t *handle, DispParamType paramType) {
    auto err = validateAndMap(handle);
    if (err != 0)
        return err;
    return clearMetaDataVa(reinterpret_cast<MetaData_t *>(handle->base_metadata),
            paramType);
}

int clearMetaDataVa(MetaData_t *data, DispParamType paramType) {
    if (data == nullptr)
        return -EINVAL;
    data->operation &= ~paramType;
    switch (paramType) {
        case SET_S3D_COMP:
            data->s3dComp.displayId = -1;
            data->s3dComp.s3dMode = 0;
            break;
        case SET_VIDEO_PERF_MODE:
            data->isVideoPerfMode = 0;
            break;
        case SET_CVP_METADATA:
            data->cvpMetadata.size = 0;
            break;
        case SET_VIDEO_HISTOGRAM_STATS:
            data->video_histogram_stats.stat_len = 0;
            break;
        default:
            ALOGE("Unknown paramType %d", paramType);
            break;
    }
    return 0;
}

int getMetaData(private_handle_t *handle, DispFetchParamType paramType,
                                                    void *param) {
    int ret = validateAndMap(handle);
    if (ret != 0)
        return ret;
    return getMetaDataVa(reinterpret_cast<MetaData_t *>(handle->base_metadata),
                         paramType, param);
}

int getMetaDataVa(MetaData_t *data, DispFetchParamType paramType,
                  void *param) {
    // Make sure we send 0 only if the operation queried is present
    int ret = -EINVAL;
    if (data == nullptr)
        return ret;
    if (param == nullptr)
        return ret;

    switch (paramType) {
        case GET_PP_PARAM_INTERLACED:
            if (data->operation & PP_PARAM_INTERLACED) {
                *((int32_t *)param) = data->interlaced;
                ret = 0;
            }
            break;
        case GET_BUFFER_GEOMETRY:
            if (data->operation & UPDATE_BUFFER_GEOMETRY) {
                *((BufferDim_t *)param) = data->bufferDim;
                ret = 0;
            }
            break;
        case GET_REFRESH_RATE:
            if (data->operation & UPDATE_REFRESH_RATE) {
                *((float *)param) = data->refreshrate;
                ret = 0;
            }
            break;
        case GET_COLOR_SPACE:
            if (data->operation & UPDATE_COLOR_SPACE) {
                *((ColorSpace_t *)param) = data->colorSpace;
                ret = 0;
            }
            break;
        case GET_MAP_SECURE_BUFFER:
            if (data->operation & MAP_SECURE_BUFFER) {
                *((int32_t *)param) = data->mapSecureBuffer;
                ret = 0;
            }
            break;
        case GET_S3D_FORMAT:
            if (data->operation & S3D_FORMAT) {
                *((uint32_t *)param) = data->s3dFormat;
                ret = 0;
            }
            break;
        case GET_LINEAR_FORMAT:
            if (data->operation & LINEAR_FORMAT) {
                *((uint32_t *)param) = data->linearFormat;
                ret = 0;
            }
            break;
        case GET_IGC:
            if (data->operation & SET_IGC) {
                *((IGC_t *)param) = data->igc;
                ret = 0;
            }
            break;
        case GET_SINGLE_BUFFER_MODE:
            if (data->operation & SET_SINGLE_BUFFER_MODE) {
                *((uint32_t *)param) = data->isSingleBufferMode;
                ret = 0;
            }
            break;
        case GET_S3D_COMP:
            if (data->operation & SET_S3D_COMP) {
                *((S3DGpuComp_t *)param) = data->s3dComp;
                ret = 0;
            }
            break;
        case GET_VT_TIMESTAMP:
            if (data->operation & SET_VT_TIMESTAMP) {
                *((uint64_t *)param) = data->vtTimeStamp;
                ret = 0;
            }
            break;
        case GET_COLOR_METADATA:
            if (data->operation & COLOR_METADATA) {
                *((ColorMetaData *)param) = data->color;
                ret = 0;
            }
            break;
        case GET_UBWC_CR_STATS_INFO:
            if (data->operation & SET_UBWC_CR_STATS_INFO) {
                struct UBWCStats* stats = (struct UBWCStats*)param;
                int numelems = sizeof(data->ubwcCRStats) / sizeof(struct UBWCStats);
                for (int i = 0; i < numelems; i++) {
                    stats[i] = data->ubwcCRStats[i];
                }
                ret = 0;
            }
            break;
        case GET_VIDEO_PERF_MODE:
            if (data->operation & SET_VIDEO_PERF_MODE) {
                *((uint32_t *)param) = data->isVideoPerfMode;
                ret = 0;
            }
            break;
        case GET_GRAPHICS_METADATA:
            if (data->operation & SET_GRAPHICS_METADATA) {
                memcpy(param, data->graphics_metadata.data, sizeof(data->graphics_metadata.data));
                ret = 0;
            }
            break;
        case GET_CVP_METADATA:
            if (data->operation & SET_CVP_METADATA) {
                struct CVPMetadata *cvpMetadata = (struct CVPMetadata *)param;
                cvpMetadata->size = 0;
                if (data->cvpMetadata.size <= CVP_METADATA_SIZE) {
                    cvpMetadata->size = data->cvpMetadata.size;
                    memcpy(cvpMetadata->payload, data->cvpMetadata.payload,
                           data->cvpMetadata.size);
                    cvpMetadata->capture_frame_rate = data->cvpMetadata.capture_frame_rate;
                    cvpMetadata->cvp_frame_rate = data->cvpMetadata.cvp_frame_rate;
                    cvpMetadata->flags = data->cvpMetadata.flags;
                    memcpy(cvpMetadata->reserved, data->cvpMetadata.reserved,
                           (8 * sizeof(uint32_t)));
                    ret = 0;
                }
            }
            break;
        case GET_VIDEO_HISTOGRAM_STATS:
            if (data->operation & SET_VIDEO_HISTOGRAM_STATS) {
                struct VideoHistogramMetadata *vidstats = (struct VideoHistogramMetadata *)param;
                vidstats->stat_len = 0;
                if (data->video_histogram_stats.stat_len <= VIDEO_HISTOGRAM_STATS_SIZE) {
                    memcpy(vidstats->stats_info,
                        data->video_histogram_stats.stats_info, VIDEO_HISTOGRAM_STATS_SIZE);
                    vidstats->stat_len = data->video_histogram_stats.stat_len;
                    vidstats->frame_type = data->video_histogram_stats.frame_type;
                    vidstats->display_width = data->video_histogram_stats.display_width;
                    vidstats->display_height = data->video_histogram_stats.display_height;
                    vidstats->decode_width = data->video_histogram_stats.decode_width;
                    vidstats->decode_height = data->video_histogram_stats.decode_height;
                    ret = 0;
                }
            }
            break;
        default:
            ALOGE("Unknown paramType %d", paramType);
            break;
    }
    return ret;
}

int copyMetaData(struct private_handle_t *src, struct private_handle_t *dst) {
    auto err = validateAndMap(src);
    if (err != 0)
        return err;

    err = validateAndMap(dst);
    if (err != 0)
        return err;

    MetaData_t *src_data = reinterpret_cast <MetaData_t *>(src->base_metadata);
    MetaData_t *dst_data = reinterpret_cast <MetaData_t *>(dst->base_metadata);
    *dst_data = *src_data;
    return 0;
}

int copyMetaDataVaToHandle(MetaData_t *src_data, struct private_handle_t *dst) {
    int err = -EINVAL;
    if (src_data == nullptr)
        return err;

    err = validateAndMap(dst);
    if (err != 0)
        return err;

    MetaData_t *dst_data = reinterpret_cast <MetaData_t *>(dst->base_metadata);
    *dst_data = *src_data;
    return 0;
}

int copyMetaDataHandleToVa(struct private_handle_t *src, MetaData_t *dst_data) {
    int err = -EINVAL;
    if (dst_data == nullptr)
        return err;

    err = validateAndMap(src);
    if (err != 0)
        return err;

    MetaData_t *src_data = reinterpret_cast <MetaData_t *>(src->base_metadata);
    *dst_data = *src_data;
    return 0;
}

int copyMetaDataVaToVa(MetaData_t *src_data, MetaData_t *dst_data) {
    int err = -EINVAL;
    if (src_data == nullptr)
        return err;

    if (dst_data == nullptr)
        return err;

    *dst_data = *src_data;
    return 0;
}

int setMetaDataAndUnmap(struct private_handle_t *handle, enum DispParamType paramType,
                        void *param) {
    auto ret = setMetaData(handle, paramType, param);
    unmapAndReset(handle);
    return ret;
}

int getMetaDataAndUnmap(struct private_handle_t *handle,
                        enum DispFetchParamType paramType,
                        void *param) {
    auto ret = getMetaData(handle, paramType, param);
    unmapAndReset(handle);
    return ret;
}
