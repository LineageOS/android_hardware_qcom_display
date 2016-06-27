/*
 * Copyright (C) 2010 The Android Open Source Project
 * Copyright (c) 2011-2014,2017 The Linux Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <limits.h>
#include <unistd.h>
#include <fcntl.h>
#include <cutils/properties.h>
#include <sys/mman.h>
#include <linux/msm_ion.h>
#ifdef COMPILE_DRM
#include <drm/drm_fourcc.h>
#include <drm_master.h>
#endif
#include <qdMetaData.h>
#include <qd_utils.h>

#include <algorithm>

#include "gr.h"
#include "gpu.h"
#include "memalloc.h"
#include "alloc_controller.h"

#ifdef COMPILE_DRM
#ifndef DRM_FORMAT_MOD_QCOM_COMPRESSED
#define DRM_FORMAT_MOD_QCOM_COMPRESSED fourcc_mod_code(QCOM, 1)
#endif
#endif

using namespace gralloc;

#ifdef COMPILE_DRM
using namespace drm_utils;

static int getPlaneStrideOffset(private_handle_t *hnd, uint32_t *stride,
        uint32_t *offset, uint32_t *num_planes) {
    struct android_ycbcr yuvInfo = {};
    *num_planes = 1;

    switch (hnd->format) {
        case HAL_PIXEL_FORMAT_RGB_565:
        case HAL_PIXEL_FORMAT_BGR_565:
        case HAL_PIXEL_FORMAT_RGBA_5551:
        case HAL_PIXEL_FORMAT_RGBA_4444:
            stride[0] = hnd->width * 2;
            break;
        case HAL_PIXEL_FORMAT_RGB_888:
            stride[0] = hnd->width * 3;
            break;
        case HAL_PIXEL_FORMAT_RGBA_8888:
        case HAL_PIXEL_FORMAT_BGRA_8888:
        case HAL_PIXEL_FORMAT_RGBX_8888:
        case HAL_PIXEL_FORMAT_BGRX_8888:
        case HAL_PIXEL_FORMAT_RGBA_1010102:
        case HAL_PIXEL_FORMAT_ARGB_2101010:
        case HAL_PIXEL_FORMAT_RGBX_1010102:
        case HAL_PIXEL_FORMAT_XRGB_2101010:
        case HAL_PIXEL_FORMAT_BGRA_1010102:
        case HAL_PIXEL_FORMAT_ABGR_2101010:
        case HAL_PIXEL_FORMAT_BGRX_1010102:
        case HAL_PIXEL_FORMAT_XBGR_2101010:
            stride[0] = hnd->width * 4;
            break;
    }

    // Format is RGB
    if (stride[0]) {
        return 0;
    }

    (*num_planes)++;
    int ret = getYUVPlaneInfo(hnd, &yuvInfo);
    if (ret < 0) {
        ALOGE("%s failed", __FUNCTION__);
        return ret;
    }

    stride[0] = static_cast<uint32_t>(yuvInfo.ystride);
    offset[0] = static_cast<uint32_t>(
                    reinterpret_cast<uint64_t>(yuvInfo.y) - hnd->base);
    stride[1] = static_cast<uint32_t>(yuvInfo.cstride);
    switch (hnd->format) {
        case HAL_PIXEL_FORMAT_YCbCr_420_SP:
        case HAL_PIXEL_FORMAT_YCbCr_422_SP:
        case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
        case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
        case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
        case HAL_PIXEL_FORMAT_YCbCr_420_P010:
        case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
            offset[1] = static_cast<uint32_t>(
                    reinterpret_cast<uint64_t>(yuvInfo.cb) - hnd->base);
            break;
        case HAL_PIXEL_FORMAT_YCrCb_420_SP:
        case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
        case HAL_PIXEL_FORMAT_YCrCb_422_SP:
            offset[1] = static_cast<uint32_t>(
                    reinterpret_cast<uint64_t>(yuvInfo.cr) - hnd->base);
            break;
        case HAL_PIXEL_FORMAT_YV12:
            offset[1] = static_cast<uint32_t>(
                    reinterpret_cast<uint64_t>(yuvInfo.cr) - hnd->base);
            stride[2] = static_cast<uint32_t>(yuvInfo.cstride);
            offset[2] = static_cast<uint32_t>(
                    reinterpret_cast<uint64_t>(yuvInfo.cb) - hnd->base);
            (*num_planes)++;
            break;
        default:
            ALOGW("%s: Unsupported format %s", __FUNCTION__,
                    qdutils::GetHALPixelFormatString(hnd->format));
    }

    if (hnd->flags & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED) {
        std::fill(offset, offset + 4, 0);
    }

    return 0;
}

static void getDRMFormat(int hal_format, int flags, uint32_t *drm_format,
        uint64_t *drm_format_modifier) {

    if (flags & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED) {
        *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
    }

    switch (hal_format) {
        case HAL_PIXEL_FORMAT_RGBA_8888:
            *drm_format = DRM_FORMAT_RGBA8888;
            break;
        case HAL_PIXEL_FORMAT_RGBA_5551:
            *drm_format = DRM_FORMAT_RGBA5551;
            break;
        case HAL_PIXEL_FORMAT_RGBA_4444:
            *drm_format = DRM_FORMAT_RGBA4444;
            break;
        case HAL_PIXEL_FORMAT_BGRA_8888:
            *drm_format = DRM_FORMAT_BGRA8888;
            break;
        case HAL_PIXEL_FORMAT_RGBX_8888:
            *drm_format = DRM_FORMAT_RGBX8888;
            break;
        case HAL_PIXEL_FORMAT_BGRX_8888:
            *drm_format = DRM_FORMAT_BGRX8888;
            break;
        case HAL_PIXEL_FORMAT_RGB_888:
            *drm_format = DRM_FORMAT_RGB888;
            break;
        case HAL_PIXEL_FORMAT_RGB_565:
            *drm_format = DRM_FORMAT_RGB565;
            break;
        case HAL_PIXEL_FORMAT_BGR_565:
            *drm_format = DRM_FORMAT_BGR565;
            break;
        case HAL_PIXEL_FORMAT_RGBA_1010102:
            *drm_format = DRM_FORMAT_RGBA1010102;
            break;
        case HAL_PIXEL_FORMAT_ARGB_2101010:
            *drm_format = DRM_FORMAT_ARGB2101010;
            break;
        case HAL_PIXEL_FORMAT_RGBX_1010102:
            *drm_format = DRM_FORMAT_RGBX1010102;
            break;
        case HAL_PIXEL_FORMAT_XRGB_2101010:
            *drm_format = DRM_FORMAT_XRGB2101010;
            break;
        case HAL_PIXEL_FORMAT_BGRA_1010102:
            *drm_format = DRM_FORMAT_BGRA1010102;
            break;
        case HAL_PIXEL_FORMAT_ABGR_2101010:
            *drm_format = DRM_FORMAT_ABGR2101010;
            break;
        case HAL_PIXEL_FORMAT_BGRX_1010102:
            *drm_format = DRM_FORMAT_BGRX1010102;
            break;
        case HAL_PIXEL_FORMAT_XBGR_2101010:
            *drm_format = DRM_FORMAT_XBGR2101010;
            break;
        case HAL_PIXEL_FORMAT_YCbCr_420_SP:
            *drm_format = DRM_FORMAT_NV12;
            break;
        case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
        case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
            *drm_format = DRM_FORMAT_NV12;
            break;
        case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
            *drm_format = DRM_FORMAT_NV12;
            *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
            break;
        case HAL_PIXEL_FORMAT_YCrCb_420_SP:
            *drm_format = DRM_FORMAT_NV21;
            break;
        case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
            *drm_format = DRM_FORMAT_NV21;
            break;
        case HAL_PIXEL_FORMAT_YCbCr_420_P010:
            // TODO *drm_format = DRM_FORMAT_P010;
            break;
        case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
            // TODO *drm_format = DRM_FORMAT_P010;
            // *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED |
            //        DRM_FORMAT_MOD_QCOM_TIGHT;
            break;
        case HAL_PIXEL_FORMAT_YCbCr_422_SP:
            *drm_format = DRM_FORMAT_NV16;
            break;
        case HAL_PIXEL_FORMAT_YCrCb_422_SP:
            *drm_format = DRM_FORMAT_NV61;
            break;
        case HAL_PIXEL_FORMAT_YV12:
            *drm_format = DRM_FORMAT_YVU420;
            break;
        default:
            ALOGW("%s: Unsupported format %s", __FUNCTION__,
                    qdutils::GetHALPixelFormatString(hal_format));
    }
}
#endif

gpu_context_t::gpu_context_t(const private_module_t* module,
                             IAllocController* alloc_ctrl ) :
    mAllocCtrl(alloc_ctrl)
{
    // Zero out the alloc_device_t
    memset(static_cast<alloc_device_t*>(this), 0, sizeof(alloc_device_t));

    // Initialize the procs
    common.tag     = HARDWARE_DEVICE_TAG;
    common.version = 0;
    common.module  = const_cast<hw_module_t*>(&module->base.common);
    common.close   = gralloc_close;
    alloc          = gralloc_alloc;
    free           = gralloc_free;

}

int gpu_context_t::gralloc_alloc_buffer(unsigned int size, int usage,
                                        buffer_handle_t* pHandle, int bufferType,
                                        int format, int width, int height)
{
    int err = 0;
    int flags = 0;
    int alignedw = 0;
    int alignedh = 0;

    AdrenoMemInfo::getInstance().getAlignedWidthAndHeight(width,
            height,
            format,
            usage,
            alignedw,
            alignedh);

    size = roundUpToPageSize(size);
    alloc_data data;
    data.offset = 0;
    data.fd = -1;
    data.base = 0;
    if(format == HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED)
        data.align = 8192;
    else
        data.align = getpagesize();

    if (usage & GRALLOC_USAGE_PROTECTED) {
            if ((usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY) ||
                (usage & GRALLOC_USAGE_HW_CAMERA_MASK)) {
                /* The alignment here reflects qsee mmu V7L/V8L requirement */
                data.align = SZ_2M;
            } else {
                data.align = SECURE_ALIGN;
            }
        size = ALIGN(size, data.align);
    }

    data.size = size;
    data.pHandle = (uintptr_t) pHandle;
    err = mAllocCtrl->allocate(data, usage);

    if (!err) {
        /* allocate memory for enhancement data */
        alloc_data eData;
        eData.fd = -1;
        eData.base = 0;
        eData.offset = 0;
        eData.size = ROUND_UP_PAGESIZE(sizeof(MetaData_t));
        eData.pHandle = data.pHandle;
        eData.align = getpagesize();
        int eDataUsage = 0;
        int eDataErr = mAllocCtrl->allocate(eData, eDataUsage);
        ALOGE_IF(eDataErr, "gralloc failed for eDataErr=%s",
                                          strerror(-eDataErr));

        if (usage & GRALLOC_USAGE_PRIVATE_EXTERNAL_ONLY) {
            flags |= private_handle_t::PRIV_FLAGS_EXTERNAL_ONLY;
        }

        if (usage & GRALLOC_USAGE_PRIVATE_INTERNAL_ONLY) {
            flags |= private_handle_t::PRIV_FLAGS_INTERNAL_ONLY;
        }

        if (usage & GRALLOC_USAGE_HW_VIDEO_ENCODER ) {
            flags |= private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
        }

        if (usage & GRALLOC_USAGE_HW_CAMERA_WRITE) {
            flags |= private_handle_t::PRIV_FLAGS_CAMERA_WRITE;
        }

        if (usage & GRALLOC_USAGE_HW_CAMERA_READ) {
            flags |= private_handle_t::PRIV_FLAGS_CAMERA_READ;
        }

        if (usage & GRALLOC_USAGE_HW_COMPOSER) {
            flags |= private_handle_t::PRIV_FLAGS_HW_COMPOSER;
        }

        if (usage & GRALLOC_USAGE_HW_TEXTURE) {
            flags |= private_handle_t::PRIV_FLAGS_HW_TEXTURE;
        }

        if(usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY) {
            flags |= private_handle_t::PRIV_FLAGS_SECURE_DISPLAY;
        }

        if (isUBwcEnabled(format, usage)) {
            flags |= private_handle_t::PRIV_FLAGS_UBWC_ALIGNED;
        }

        if(usage & (GRALLOC_USAGE_SW_READ_MASK | GRALLOC_USAGE_SW_WRITE_MASK)) {
            flags |= private_handle_t::PRIV_FLAGS_CPU_RENDERED;
        }

        if (usage & (GRALLOC_USAGE_HW_VIDEO_ENCODER |
                GRALLOC_USAGE_HW_CAMERA_WRITE |
                GRALLOC_USAGE_HW_RENDER |
                GRALLOC_USAGE_HW_FB)) {
            flags |= private_handle_t::PRIV_FLAGS_NON_CPU_WRITER;
        }

        if(usage & GRALLOC_USAGE_HW_COMPOSER) {
            flags |= private_handle_t::PRIV_FLAGS_DISP_CONSUMER;
        }

        if(false == data.uncached) {
            flags |= private_handle_t::PRIV_FLAGS_CACHED;
        }

        flags |= data.allocType;
        uint64_t eBaseAddr = (uint64_t)(eData.base) + eData.offset;
        private_handle_t *hnd = new private_handle_t(data.fd, size, flags,
                bufferType, format, alignedw, alignedh,
                eData.fd, eData.offset, eBaseAddr, width, height);

        hnd->offset = data.offset;
        hnd->base = (uint64_t)(data.base) + data.offset;
        hnd->gpuaddr = 0;
        ColorSpace_t colorSpace = ITU_R_601;
        setMetaData(hnd, UPDATE_COLOR_SPACE, (void*) &colorSpace);

#ifdef COMPILE_DRM
        if (qdutils::getDriverType() == qdutils::DriverType::DRM &&
                usage & GRALLOC_USAGE_HW_COMPOSER) {
            DRMBuffer buf = {};
            int ret = getPlaneStrideOffset(hnd, buf.stride, buf.offset,
                    &buf.num_planes);
            if (ret < 0) {
                ALOGE("%s failed", __FUNCTION__);
                return ret;
            }

            buf.fd = hnd->fd;
            buf.width = hnd->width;
            buf.height = hnd->height;
            getDRMFormat(hnd->format, flags, &buf.drm_format,
                    &buf.drm_format_modifier);

            DRMMaster *master = nullptr;
            ret = DRMMaster::GetInstance(&master);
            if (ret < 0) {
                ALOGE("%s Failed to acquire DRMMaster instance", __FUNCTION__);
                return ret;
            }

            ret = master->CreateFbId(buf, &hnd->gem_handle, &hnd->fb_id);
            if (ret < 0) {
                ALOGE("%s: CreateFbId failed. width %d, height %d, " \
                        "format: %s, stride %u, error %d", __FUNCTION__,
                        buf.width, buf.height,
                        qdutils::GetHALPixelFormatString(hnd->format),
                        buf.stride[0], errno);
                return ret;
            }
        }
#endif

        *pHandle = hnd;
    }

    ALOGE_IF(err, "gralloc failed err=%s", strerror(-err));
    return err;
}

void gpu_context_t::getGrallocInformationFromFormat(int inputFormat,
                                                    int *bufferType)
{
    *bufferType = BUFFER_TYPE_VIDEO;

    if (isUncompressedRgbFormat(inputFormat) == TRUE) {
        // RGB formats
        *bufferType = BUFFER_TYPE_UI;
    }
}

int gpu_context_t::gralloc_alloc_framebuffer_locked(int usage,
                                                    buffer_handle_t* pHandle)
{
    private_module_t* m = reinterpret_cast<private_module_t*>(common.module);

    // This allocation will only happen when gralloc is in fb mode

    if (m->framebuffer == NULL) {
        ALOGE("%s: Invalid framebuffer", __FUNCTION__);
        return -EINVAL;
    }

    const unsigned int bufferMask = m->bufferMask;
    const uint32_t numBuffers = m->numBuffers;
    unsigned int bufferSize = m->finfo.line_length * m->info.yres;

    //adreno needs FB size to be page aligned
    bufferSize = roundUpToPageSize(bufferSize);

    if (numBuffers == 1) {
        // If we have only one buffer, we never use page-flipping. Instead,
        // we return a regular buffer which will be memcpy'ed to the main
        // screen when post is called.
        int newUsage = (usage & ~GRALLOC_USAGE_HW_FB) | GRALLOC_USAGE_HW_2D;
        return gralloc_alloc_buffer(bufferSize, newUsage, pHandle, BUFFER_TYPE_UI,
                                    m->fbFormat, m->info.xres, m->info.yres);
    }

    if (bufferMask >= ((1LU<<numBuffers)-1)) {
        // We ran out of buffers.
        return -ENOMEM;
    }

    // create a "fake" handle for it
    uint64_t vaddr = uint64_t(m->framebuffer->base);
    // As GPU needs ION FD, the private handle is created
    // using ION fd and ION flags are set
    private_handle_t* hnd = new private_handle_t(
        dup(m->framebuffer->fd), bufferSize,
        private_handle_t::PRIV_FLAGS_USES_ION |
        private_handle_t::PRIV_FLAGS_FRAMEBUFFER,
        BUFFER_TYPE_UI, m->fbFormat, m->info.xres,
        m->info.yres);

    // find a free slot
    for (uint32_t i=0 ; i<numBuffers ; i++) {
        if ((bufferMask & (1LU<<i)) == 0) {
            m->bufferMask |= (uint32_t)(1LU<<i);
            break;
        }
        vaddr += bufferSize;
    }
    hnd->base = vaddr;
    hnd->offset = (unsigned int)(vaddr - m->framebuffer->base);
    *pHandle = hnd;
    return 0;
}


int gpu_context_t::gralloc_alloc_framebuffer(int usage,
                                             buffer_handle_t* pHandle)
{
    private_module_t* m = reinterpret_cast<private_module_t*>(common.module);
    pthread_mutex_lock(&m->lock);
    int err = gralloc_alloc_framebuffer_locked(usage, pHandle);
    pthread_mutex_unlock(&m->lock);
    return err;
}

int gpu_context_t::alloc_impl(int w, int h, int format, int usage,
                              buffer_handle_t* pHandle, int* pStride,
                              unsigned int bufferSize) {
    if (!pHandle || !pStride)
        return -EINVAL;

    unsigned int size;
    int alignedw, alignedh;
    int grallocFormat = format;
    int bufferType;

    //If input format is HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED then based on
    //the usage bits, gralloc assigns a format.
    if(format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED ||
       format == HAL_PIXEL_FORMAT_YCbCr_420_888) {
        if (usage & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC)
            grallocFormat = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC;
        else if(usage & GRALLOC_USAGE_HW_VIDEO_ENCODER) {
            if(MDPCapabilityInfo::getInstance().isWBUBWCSupportedByMDP() &&
               !IAllocController::getInstance()->isDisableUBWCForEncoder() &&
               usage & GRALLOC_USAGE_HW_COMPOSER)
              grallocFormat = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC;
            else
              grallocFormat = HAL_PIXEL_FORMAT_NV12_ENCODEABLE; //NV12
        } else if((usage & GRALLOC_USAGE_HW_CAMERA_MASK)
                == GRALLOC_USAGE_HW_CAMERA_ZSL)
            grallocFormat = HAL_PIXEL_FORMAT_NV21_ZSL; //NV21 ZSL
        else if(usage & GRALLOC_USAGE_HW_CAMERA_READ)
            grallocFormat = HAL_PIXEL_FORMAT_YCrCb_420_SP; //NV21
        else if(usage & GRALLOC_USAGE_HW_CAMERA_WRITE) {
           if (format == HAL_PIXEL_FORMAT_YCbCr_420_888) {
               grallocFormat = HAL_PIXEL_FORMAT_NV21_ZSL; //NV21
           } else {
               grallocFormat = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS; //NV12 preview
           }
        } else if(usage & GRALLOC_USAGE_HW_COMPOSER)
            //XXX: If we still haven't set a format, default to RGBA8888
            grallocFormat = HAL_PIXEL_FORMAT_RGBA_8888;
        else if(format == HAL_PIXEL_FORMAT_YCbCr_420_888) {
            //If no other usage flags are detected, default the
            //flexible YUV format to NV21_ZSL
            grallocFormat = HAL_PIXEL_FORMAT_NV21_ZSL;
        }
    }

    bool useFbMem = false;
    char property[PROPERTY_VALUE_MAX];
    char isUBWC[PROPERTY_VALUE_MAX];
    if (usage & GRALLOC_USAGE_HW_FB) {
        if ((property_get("debug.gralloc.map_fb_memory", property, NULL) > 0) &&
            (!strncmp(property, "1", PROPERTY_VALUE_MAX ) ||
            (!strncasecmp(property,"true", PROPERTY_VALUE_MAX )))) {
            useFbMem = true;
        } else {
            usage &= ~GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
            if (property_get("debug.gralloc.enable_fb_ubwc", isUBWC, NULL) > 0){
                if ((!strncmp(isUBWC, "1", PROPERTY_VALUE_MAX)) ||
                    (!strncasecmp(isUBWC, "true", PROPERTY_VALUE_MAX))) {
                    // Allocate UBWC aligned framebuffer
                    usage |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
                }
            }
        }
    }

    getGrallocInformationFromFormat(grallocFormat, &bufferType);
    size = getBufferSizeAndDimensions(w, h, grallocFormat, usage, alignedw,
                   alignedh);

    if ((unsigned int)size <= 0)
        return -EINVAL;
    size = (bufferSize >= size)? bufferSize : size;

    int err = 0;
    if(useFbMem) {
        err = gralloc_alloc_framebuffer(usage, pHandle);
    } else {
        err = gralloc_alloc_buffer(size, usage, pHandle, bufferType,
                                   grallocFormat, w, h);
    }

    if (err < 0) {
        return err;
    }

    *pStride = alignedw;
    return 0;
}

int gpu_context_t::free_impl(private_handle_t const* hnd) {
    private_module_t* m = reinterpret_cast<private_module_t*>(common.module);
    if (hnd->flags & private_handle_t::PRIV_FLAGS_FRAMEBUFFER) {
        const unsigned int bufferSize = m->finfo.line_length * m->info.yres;
        unsigned int index = (unsigned int) ((hnd->base - m->framebuffer->base)
                / bufferSize);
        m->bufferMask &= (uint32_t)~(1LU<<index);
    } else {

        terminateBuffer(&m->base, const_cast<private_handle_t*>(hnd));
        IMemAlloc* memalloc = mAllocCtrl->getAllocator(hnd->flags);
        int err = memalloc->free_buffer((void*)hnd->base, hnd->size,
                                        hnd->offset, hnd->fd);
        if(err)
            return err;
        // free the metadata space
        unsigned int size = ROUND_UP_PAGESIZE(sizeof(MetaData_t));
        err = memalloc->free_buffer((void*)hnd->base_metadata,
                                    size, hnd->offset_metadata,
                                    hnd->fd_metadata);
        if (err)
            return err;
    }

#ifdef COMPILE_DRM
    if (hnd->fb_id) {
        DRMMaster *master = nullptr;
        int ret = DRMMaster::GetInstance(&master);
        if (ret < 0) {
            ALOGE("%s Failed to acquire DRMMaster instance", __FUNCTION__);
            return ret;
        }
        ret = master->RemoveFbId(hnd->gem_handle, hnd->fb_id);
        if (ret < 0) {
            ALOGE("%s: Removing fb_id %d failed with error %d", __FUNCTION__,
                    hnd->fb_id, errno);
        }
    }
#endif

    delete hnd;
    return 0;
}

int gpu_context_t::gralloc_alloc(alloc_device_t* dev, int w, int h, int format,
                                 int usage, buffer_handle_t* pHandle,
                                 int* pStride)
{
    if (!dev) {
        return -EINVAL;
    }
    gpu_context_t* gpu = reinterpret_cast<gpu_context_t*>(dev);
    return gpu->alloc_impl(w, h, format, usage, pHandle, pStride, 0);
}
int gpu_context_t::gralloc_alloc_size(alloc_device_t* dev, int w, int h,
                                      int format, int usage,
                                      buffer_handle_t* pHandle, int* pStride,
                                      int bufferSize)
{
    if (!dev) {
        return -EINVAL;
    }
    gpu_context_t* gpu = reinterpret_cast<gpu_context_t*>(dev);
    return gpu->alloc_impl(w, h, format, usage, pHandle, pStride, bufferSize);
}


int gpu_context_t::gralloc_free(alloc_device_t* dev,
                                buffer_handle_t handle)
{
    if (private_handle_t::validate(handle) < 0)
        return -EINVAL;

    private_handle_t const* hnd = reinterpret_cast<private_handle_t const*>(handle);
    gpu_context_t* gpu = reinterpret_cast<gpu_context_t*>(dev);
    return gpu->free_impl(hnd);
}

/*****************************************************************************/

int gpu_context_t::gralloc_close(struct hw_device_t *dev)
{
    gpu_context_t* ctx = reinterpret_cast<gpu_context_t*>(dev);
    if (ctx) {
        /* TODO: keep a list of all buffer_handle_t created, and free them
         * all here.
         */
        delete ctx;
    }
    return 0;
}

