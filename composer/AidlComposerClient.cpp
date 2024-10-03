/*
 * Copyright 2022 The Android Open Source Project
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

/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "AidlComposerClient.h"
#include "android/binder_auto_utils.h"
#include <android/binder_ibinder_platform.h>
namespace aidl {
namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace composer3 {

ComposerHandleImporter mHandleImporter;

BufferCacheEntry::BufferCacheEntry() : mHandle(nullptr) {}

BufferCacheEntry::BufferCacheEntry(BufferCacheEntry &&other) {
  mHandle = other.mHandle;
  other.mHandle = nullptr;
}

BufferCacheEntry &BufferCacheEntry::operator=(buffer_handle_t handle) {
  clear();
  mHandle = handle;
  return *this;
}

BufferCacheEntry::~BufferCacheEntry() {
  clear();
}

void BufferCacheEntry::clear() {
  if (mHandle) {
    mHandleImporter.freeBuffer(mHandle);
  }
}

bool AidlComposerClient::init() {
  hwc_session_ = HWCSession::GetInstance();

  mCommandEngine = std::make_unique<CommandEngine>(*this);
  if (mCommandEngine == nullptr) {
    return false;
  }

  if (!mCommandEngine->init()) {
    mCommandEngine = nullptr;
    return false;
  }

  return true;
}

AidlComposerClient::~AidlComposerClient() {
  ALOGW("%s: Destroying composer client", __FUNCTION__);

  EnableCallback(false);

  // no need to grab the mutex as any in-flight hwbinder call would have
  // kept the client alive
  for (const auto &dpy : mDisplayData) {
    ALOGW("%s: Destroying client resources for display %" PRIu64, __FUNCTION__, dpy.first);

    for (const auto &ly : dpy.second.Layers) {
      hwc_session_->DestroyLayer(dpy.first, ly.first);
    }

    if (dpy.second.IsVirtual) {
      destroyVirtualDisplay(dpy.first);
    } else {
      ALOGW("%s: Performing a final presentDisplay", __FUNCTION__);

      mCommandEngine->validateDisplay(dpy.first);

      hwc_session_->AcceptDisplayChanges(dpy.first);

      shared_ptr<Fence> presentFence = nullptr;
      mCommandEngine->presentDisplay(dpy.first, &presentFence);
    }
  }

  mDisplayData.clear();

  mHandleImporter.cleanup();

  if (mOnClientDestroyed) {
    mOnClientDestroyed();
  }

  ALOGW("%s: Removed composer client", __FUNCTION__);
}

void AidlComposerClient::EnableCallback(bool enable) {
  if (enable) {
    hotplug_callback_ = &OnHotplug;
    refresh_callback_ = &OnRefresh;
    vsync_callback_ = &OnVsync;
    vsync_changed_callback_ = &OnVsyncPeriodTimingChanged;
    seamless_possible_callback_ = &OnSeamlessPossible;
    vsync_idle_callback_ = &OnVsyncIdle;
    hwc_session_->RegisterCallback(sdm::CALLBACK_HOTPLUG, this,
                                   reinterpret_cast<void *>(&hotplug_callback_));
    hwc_session_->RegisterCallback(sdm::CALLBACK_REFRESH, this,
                                   reinterpret_cast<void *>(&refresh_callback_));
    hwc_session_->RegisterCallback(sdm::CALLBACK_VSYNC, this,
                                   reinterpret_cast<void *>(&vsync_callback_));
    hwc_session_->RegisterCallback(sdm::CALLBACK_VSYNC_PERIOD_TIMING_CHANGED, this,
                                   reinterpret_cast<void *>(&vsync_changed_callback_));
    hwc_session_->RegisterCallback(sdm::CALLBACK_SEAMLESS_POSSIBLE, this,
                                   reinterpret_cast<void *>(&seamless_possible_callback_));
    hwc_session_->RegisterCallback(sdm::CALLBACK_VSYNC_IDLE, this,
                                   reinterpret_cast<void *>(&vsync_idle_callback_));
  } else {
    hwc_session_->RegisterCallback(sdm::CALLBACK_HOTPLUG, this, nullptr);
    hwc_session_->RegisterCallback(sdm::CALLBACK_REFRESH, this, nullptr);
    hwc_session_->RegisterCallback(sdm::CALLBACK_VSYNC, this, nullptr);
    hwc_session_->RegisterCallback(sdm::CALLBACK_VSYNC_PERIOD_TIMING_CHANGED, this, nullptr);
    hwc_session_->RegisterCallback(sdm::CALLBACK_SEAMLESS_POSSIBLE, this, nullptr);
    hwc_session_->RegisterCallback(sdm::CALLBACK_VSYNC_IDLE, this, nullptr);
  }
}

ScopedAStatus AidlComposerClient::createLayer(int64_t in_display, int32_t in_buffer_slot_count,
                                              int64_t *aidl_return) {
  sdm::LayerId layer = 0;
  auto error = hwc_session_->CreateLayer(in_display, &layer);
  if (error == Error::None) {
    *aidl_return = static_cast<int64_t>(layer);
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    auto dpy = mDisplayData.find(in_display);
    // The display entry may have already been removed by onHotplug.
    if (dpy != mDisplayData.end()) {
      auto ly = dpy->second.Layers.emplace(layer, LayerBuffers()).first;
      ly->second.Buffers.resize(in_buffer_slot_count);
    } else {
      error = Error::BadDisplay;
      // Note: We do not destroy the layer on this error as the hotplug
      // disconnect invalidates the display id. The implementation should
      // ensure all layers for the display are destroyed.
    }
  }
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::createVirtualDisplay(int32_t in_width, int32_t in_height,
                                                       PixelFormat in_format_hint,
                                                       int32_t in_output_buffer_slot_count,
                                                       VirtualDisplay *aidl_return) {
  int32_t format = static_cast<int32_t>(in_format_hint);
  uint64_t display;
  auto error = hwc_session_->CreateVirtualDisplay(in_width, in_height, &format, &display);

  if (error == Error::None) {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);

    auto dpy = mDisplayData.emplace(static_cast<sdm::Display>(display), DisplayData(true)).first;
    dpy->second.OutputBuffers.resize(in_output_buffer_slot_count);

    aidl_return->display = display;
    aidl_return->format = in_format_hint;
  }
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::destroyLayer(int64_t in_display, int64_t in_layer) {
  auto error = hwc_session_->DestroyLayer(in_display, in_layer);
  if (error == Error::None) {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);

    auto dpy = mDisplayData.find(in_display);
    // The display entry may have already been removed by onHotplug.
    if (dpy != mDisplayData.end()) {
      dpy->second.Layers.erase(in_layer);
    }
  }
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::destroyVirtualDisplay(int64_t in_display) {
  auto error = hwc_session_->DestroyVirtualDisplay(in_display);
  if (error == Error::None) {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);

    mDisplayData.erase(in_display);
  }

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::executeCommands(const std::vector<DisplayCommand> &in_commands,
                                                  std::vector<CommandResultPayload> *aidl_return) {
  std::lock_guard<std::mutex> lock(m_command_mutex_);

  std::lock_guard<std::mutex> hwc_lock(hwc_session_->command_seq_mutex_);

  Error error = mCommandEngine->execute(in_commands, aidl_return);

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::executeQtiCommands(
    const std::vector<QtiDisplayCommand> &in_commands,
    std::vector<CommandResultPayload> *aidl_return) {
  std::lock_guard<std::mutex> lock(m_command_mutex_);

  std::lock_guard<std::mutex> hwc_lock(hwc_session_->command_seq_mutex_);

  Error error = mCommandEngine->qtiExecute(in_commands, aidl_return);

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getActiveConfig(int64_t in_display, int32_t *aidl_return) {
  uint32_t config = 0;
  auto error = hwc_session_->GetActiveConfig(in_display, &config);

  *aidl_return = config;
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getColorModes(int64_t in_display,
                                                std::vector<ColorMode> *aidl_return) {
  uint32_t count = 0;

  auto error = hwc_session_->GetColorModes(in_display, &count, nullptr);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  aidl_return->resize(count);
  error = hwc_session_->GetColorModes(
      in_display, &count,
      reinterpret_cast<std::underlying_type<ColorMode>::type *>(aidl_return->data()));
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getDataspaceSaturationMatrix(Dataspace in_dataspace,
                                                               std::vector<float> *aidl_return) {
  if (in_dataspace != Dataspace::SRGB_LINEAR) {
    return TO_BINDER_STATUS(INT32(Error::BadParameter));
  }

  aidl_return->resize(sdm::kDataspaceSaturationMatrixCount);
  Error error = Error::Unsupported;
  error = hwc_session_->GetDataspaceSaturationMatrix(static_cast<int32_t>(in_dataspace),
                                                     aidl_return->data());
  if (error != Error::None) {
    *aidl_return = {
        1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    };
  }
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getDisplayAttribute(int64_t in_display, int32_t in_config,
                                                      DisplayAttribute in_attribute,
                                                      int32_t *aidl_return) {
  auto error = hwc_session_->GetDisplayAttribute(in_display, in_config, in_attribute, aidl_return);
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getDisplayConfigurations(
    int64_t in_display, int32_t maxFrameIntervalNs,
    std::vector<DisplayConfiguration> *out_configs) {
  auto error = hwc_session_->GetDisplayConfigurations(in_display, out_configs);
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::notifyExpectedPresent(
    int64_t displayId, const ClockMonotonicTimestamp &expectedPresentTime,
    int32_t frameIntervalNs) {
  Error error = Error::Unsupported;
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getDisplayCapabilities(
    int64_t in_display, std::vector<DisplayCapability> *aidl_return) {
  // Client queries per display capabilities which gets populated here

  std::lock_guard<std::mutex> lock(m_display_data_mutex_);
  if (mDisplayData.find(in_display) == mDisplayData.end()) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }

  HwcDisplayConnectionType display_conn_type = HwcDisplayConnectionType::INTERNAL;
  auto ret = hwc_session_->GetDisplayConnectionType(in_display, &display_conn_type);

  if (Error::None != ret) {
    return TO_BINDER_STATUS(INT32(ret));
  }

  if (HwcDisplayConnectionType::INTERNAL == display_conn_type) {
    // SKIP_CLIENT_COLOR_TRANSFORM is used to prevent client from applying color transform on the
    // client composed layers. Since DSPP would apply color transform on the final composed output,
    // this is needed to prevent applying color transform twice. When client queries per display
    // capabilities, the global Capability::SKIP_CLIENT_COLOR_TRANSFORM is ignored. We need to push
    // DisplayCapability::SKIP_CLIENT_COLOR_TRANSFORM here to maintain support.
    aidl_return->push_back(DisplayCapability::SKIP_CLIENT_COLOR_TRANSFORM);
    int32_t has_doze_support = 0;
    hwc_session_->GetDozeSupport(in_display, &has_doze_support);
    if (has_doze_support) {
      aidl_return->push_back(DisplayCapability::DOZE);
      aidl_return->push_back(DisplayCapability::SUSPEND);
      aidl_return->push_back(DisplayCapability::BRIGHTNESS);
    } else {
      aidl_return->push_back(DisplayCapability::BRIGHTNESS);
    }
  }

  return ScopedAStatus::ok();
}

ScopedAStatus AidlComposerClient::getDisplayConfigs(int64_t in_display,
                                                    std::vector<int32_t> *aidl_return) {
  uint32_t count = 0;

  auto error = hwc_session_->GetDisplayConfigs(in_display, &count, nullptr);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  aidl_return->resize(count);
  error = hwc_session_->GetDisplayConfigs(in_display, &count,
                                          reinterpret_cast<uint32_t *>(aidl_return->data()));
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getDisplayConnectionType(int64_t in_display,
                                                           DisplayConnectionType *aidl_return) {
  auto error = hwc_session_->GetDisplayConnectionType(in_display, aidl_return);
  return TO_BINDER_STATUS(INT32(error));
}
ScopedAStatus AidlComposerClient::getDisplayIdentificationData(int64_t in_display,
                                                               DisplayIdentification *aidl_return) {
  uint8_t port = 0;
  uint32_t size = 0;

  auto error = hwc_session_->GetDisplayIdentificationData(in_display, &port, &size, nullptr);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  aidl_return->data.resize(size);
  error = hwc_session_->GetDisplayIdentificationData(in_display, &port, &size,
                                                     aidl_return->data.data());
  aidl_return->port = port;

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getDisplayName(int64_t in_display, std::string *aidl_return) {
  uint32_t count = 0;
  // std::vector<char> name;

  auto error = hwc_session_->GetDisplayName(in_display, &count, nullptr);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  aidl_return->resize(count + 1);
  error = hwc_session_->GetDisplayName(in_display, &count, aidl_return->data());
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  aidl_return->at(count) = '\0';
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getDisplayVsyncPeriod(int64_t in_display, int32_t *aidl_return) {
  VsyncPeriodNanos vsync_period;
  auto error = hwc_session_->GetDisplayVsyncPeriod(in_display, &vsync_period);
  *aidl_return = INT32(vsync_period);
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getDisplayedContentSample(int64_t in_display,
                                                            int64_t in_max_frames,
                                                            int64_t in_timestamp,
                                                            DisplayContentSample *aidl_return) {
  // getDisplayedContentSample is not supported
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::getDisplayedContentSamplingAttributes(
    int64_t in_display, DisplayContentSamplingAttributes *aidl_return) {
  // getDisplayedContentSamplingAttributes is not supported
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::getDisplayPhysicalOrientation(int64_t in_display,
                                                                Transform *aidl_return) {
  std::lock_guard<std::mutex> lock(m_display_data_mutex_);
  if (mDisplayData.find(in_display) == mDisplayData.end()) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }
  if (!aidl_return)
    return TO_BINDER_STATUS(INT32(Error::BadParameter));

  // TODO: Add getDisplayPhysicalOrientation support in hwc_session
  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getHdrCapabilities(int64_t in_display,
                                                     HdrCapabilities *aidl_return) {
  uint32_t count = 0;

  auto error = hwc_session_->GetHdrCapabilities(
      in_display, &count, nullptr, &aidl_return->maxLuminance, &aidl_return->maxAverageLuminance,
      &aidl_return->minLuminance);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  aidl_return->types.resize(count);
  error = hwc_session_->GetHdrCapabilities(
      in_display, &count,
      reinterpret_cast<std::underlying_type<Hdr>::type *>(aidl_return->types.data()),
      &aidl_return->maxLuminance, &aidl_return->maxAverageLuminance, &aidl_return->minLuminance);
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getMaxVirtualDisplayCount(int32_t *aidl_return) {
  *aidl_return = hwc_session_->GetMaxVirtualDisplayCount();

  return ScopedAStatus::ok();
}

ScopedAStatus AidlComposerClient::getOverlaySupport(OverlayProperties *aidl_return) {
  auto error = hwc_session_->GetOverlaySupport(aidl_return);
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getHdrConversionCapabilities(
    std::vector<HdrConversionCapability> *_aidl_return) {
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::setHdrConversionStrategy(
    const HdrConversionStrategy &in_conversionStrategy, Hdr *_aidl_return) {
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::setRefreshRateChangedCallbackDebugEnabled(int64_t in_display,
                                                                            bool in_enabled) {
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::getPerFrameMetadataKeys(
    int64_t in_display, std::vector<PerFrameMetadataKey> *aidl_return) {
  uint32_t count = 0;

  auto error = hwc_session_->GetPerFrameMetadataKeys(in_display, &count, nullptr);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  aidl_return->resize(count);
  error = hwc_session_->GetPerFrameMetadataKeys(
      in_display, &count,
      reinterpret_cast<std::underlying_type<PerFrameMetadataKey>::type *>(aidl_return->data()));

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getReadbackBufferAttributes(
    int64_t in_display, ReadbackBufferAttributes *aidl_return) {
  int32_t format = -1;
  int32_t dataspace = -1;

  auto error = hwc_session_->GetReadbackBufferAttributes(in_display, &format, &dataspace);

  if (error != Error::None) {
    format = -1;
    dataspace = -1;
  }

  aidl_return->format = static_cast<PixelFormat>(format);
  aidl_return->dataspace = static_cast<Dataspace>(dataspace);

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getReadbackBufferFence(int64_t in_display,
                                                         ::ndk::ScopedFileDescriptor *aidl_return) {
  shared_ptr<Fence> fence = nullptr;
  auto error = hwc_session_->GetReadbackBufferFence(in_display, &fence);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  *aidl_return = ::ndk::ScopedFileDescriptor(Fence::Dup(fence));

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getRenderIntents(int64_t in_display, ColorMode in_mode,
                                                   std::vector<RenderIntent> *aidl_return) {
  uint32_t count = 0;

  auto error = hwc_session_->GetRenderIntents(in_display, int32_t(in_mode), &count, nullptr);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  std::lock_guard<std::mutex> lock(m_display_data_mutex_);
  if (mDisplayData.find(in_display) == mDisplayData.end()) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }

  aidl_return->resize(count);
  error = hwc_session_->GetRenderIntents(
      in_display, int32_t(in_mode), &count,
      reinterpret_cast<std::underlying_type<RenderIntent>::type *>(aidl_return->data()));

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::getSupportedContentTypes(int64_t in_display,
                                                           std::vector<ContentType> *aidl_return) {
  std::lock_guard<std::mutex> lock(m_display_data_mutex_);
  if (mDisplayData.find(in_display) == mDisplayData.end()) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }
  return ScopedAStatus::ok();
}

ScopedAStatus AidlComposerClient::getDisplayDecorationSupport(
    int64_t in_display, std::optional<DisplayDecorationSupport> *aidl_return) {
  PixelFormat_V3 format;
  AlphaInterpretation alpha;
  auto error = hwc_session_->getDisplayDecorationSupport(in_display, &format, &alpha);
  if (error == Error::None) {
    aidl_return->emplace();
    aidl_return->value().alphaInterpretation = alpha;
    aidl_return->value().format = format;
  }

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::registerCallback(
    const std::shared_ptr<IComposerCallback> &in_callback) {
  callback_ = in_callback;
  EnableCallback(in_callback != nullptr);
  return ScopedAStatus::ok();
}

ScopedAStatus AidlComposerClient::setActiveConfig(int64_t in_display, int32_t in_config) {
  auto error = hwc_session_->SetActiveConfig(in_display, in_config);

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::setActiveConfigWithConstraints(
    int64_t in_display, int32_t in_config,
    const VsyncPeriodChangeConstraints &in_vsync_period_change_constraints,
    VsyncPeriodChangeTimeline *aidl_return) {
  VsyncPeriodChangeTimeline timeline;
  timeline.newVsyncAppliedTimeNanos = systemTime();
  timeline.refreshRequired = false;
  timeline.refreshTimeNanos = 0;

  auto error = hwc_session_->SetActiveConfigWithConstraints(
      in_display, in_config, &in_vsync_period_change_constraints, aidl_return);
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::setBootDisplayConfig(int64_t in_display, int32_t in_config) {
  // TODO: Add support in hwc_session
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::clearBootDisplayConfig(int64_t in_display) {
  // TODO: Add support in hwc_session
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::getPreferredBootDisplayConfig(int64_t in_display,
                                                                int32_t *aidl_return) {
  // TODO: Add support in hwc_session
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::setAutoLowLatencyMode(int64_t in_display, bool in_on) {
  std::lock_guard<std::mutex> lock(m_display_data_mutex_);
  if (mDisplayData.find(in_display) == mDisplayData.end()) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::setClientTargetSlotCount(int64_t in_display,
                                                           int32_t in_client_target_slot_count) {
  std::lock_guard<std::mutex> lock(m_display_data_mutex_);

  auto dpy = mDisplayData.find(in_display);
  if (dpy == mDisplayData.end()) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }
  dpy->second.ClientTargets.resize(in_client_target_slot_count);
  return ScopedAStatus::ok();
}

ScopedAStatus AidlComposerClient::setColorMode(int64_t in_display, ColorMode in_mode,
                                               RenderIntent in_intent) {
  auto error = hwc_session_->SetColorModeWithRenderIntent(in_display, static_cast<int32_t>(in_mode),
                                                          static_cast<int32_t>(in_intent));

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::setContentType(int64_t in_display, ContentType in_type) {
  std::lock_guard<std::mutex> lock(m_display_data_mutex_);
  if (mDisplayData.find(in_display) == mDisplayData.end()) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }
  if (in_type == ContentType::NONE) {
    return TO_BINDER_STATUS(INT32(Error::None));
  }
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::setDisplayedContentSamplingEnabled(
    int64_t in_display, bool in_enable, FormatColorComponent in_component_mask,
    int64_t in_max_frames) {
  // setDisplayedContentSamplingEnabled is not supported
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

ScopedAStatus AidlComposerClient::setPowerMode(int64_t in_display, PowerMode in_mode) {
  auto error = hwc_session_->SetPowerMode(in_display, static_cast<int32_t>(in_mode));

  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::setReadbackBuffer(
    int64_t in_display, const NativeHandle &in_buffer,
    const ::ndk::ScopedFileDescriptor &in_release_fence) {
  shared_ptr<Fence> fence = nullptr;
  buffer_handle_t buffer = ::android::makeFromAidl(in_buffer);
  auto &sfd = const_cast<::ndk::ScopedFileDescriptor &>(in_release_fence);
  auto fd = sfd.get();
  *sfd.getR() = -1;

  fence = Fence::Create(fd, "read_back");

  {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    if (mDisplayData.find(in_display) == mDisplayData.end()) {
      return TO_BINDER_STATUS(INT32(Error::BadDisplay));
    }
  }

  const native_handle_t *readback_buffer = nullptr;
  auto error = getDisplayReadbackBuffer(in_display, buffer, &readback_buffer);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  // Cleanup orginally cloned handle from in_buffer
  native_handle_delete(const_cast<native_handle_t *>(buffer));

  error = hwc_session_->SetReadbackBuffer(in_display, readback_buffer, fence);
  return TO_BINDER_STATUS(INT32(error));
}

ScopedAStatus AidlComposerClient::setVsyncEnabled(int64_t in_display, bool in_enabled) {
  auto error = hwc_session_->SetVsyncEnabled(in_display, static_cast<int32_t>(in_enabled));

  return TO_BINDER_STATUS(INT32(error));
}
ScopedAStatus AidlComposerClient::setIdleTimerEnabled(int64_t in_display, int32_t in_timeoutMs) {
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

void AidlComposerClient::OnHotplug(void *callbackData, int64_t in_display, bool in_connected) {
  auto client = reinterpret_cast<AidlComposerClient *>(callbackData);
  if (!client->callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }
  if (in_connected) {
    std::lock_guard<std::mutex> lock_d(client->m_display_data_mutex_);
    client->mDisplayData.emplace(in_display, DisplayData(false));
  }

  client->callback_->onHotplug(in_display, in_connected);

  if (!in_connected) {
    // Trigger refresh to make sure disconnect event received/updated properly by SurfaceFlinger.
    client->hwc_session_->Refresh(HWC_DISPLAY_PRIMARY);
    // Wait for sufficient time to ensure sufficient resources are available to process connection.
    uint32_t vsync_period = 0;
    client->hwc_session_->GetVsyncPeriod(HWC_DISPLAY_PRIMARY, &vsync_period);
    usleep(vsync_period * 2 / 1000);

    // Wait for the input command message queue to process before destroying the local display data.
    std::lock_guard<std::mutex> lock(client->m_command_mutex_);
    std::lock_guard<std::mutex> lock_d(client->m_display_data_mutex_);
    client->mDisplayData.erase(in_display);
  }
}
void AidlComposerClient::OnRefresh(void *callback_data, int64_t in_display) {
  auto client = reinterpret_cast<AidlComposerClient *>(callback_data);
  if (!client->callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }
  client->callback_->onRefresh(in_display);
  // hwc2_callback_data_t used here originally with a callback ret status log
}
void AidlComposerClient::OnSeamlessPossible(void *callback_data, int64_t in_display) {
  auto client = reinterpret_cast<AidlComposerClient *>(callback_data);
  if (!client->callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }

  client->callback_->onSeamlessPossible(in_display);
}
void AidlComposerClient::OnVsync(void *callback_data, int64_t in_display, int64_t in_timestamp,
                                 int32_t in_vsync_period_nanos) {
  auto client = reinterpret_cast<AidlComposerClient *>(callback_data);
  if (!client->callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }
  client->callback_->onVsync(in_display, in_timestamp, in_vsync_period_nanos);
  // hwc2_callback_data_t used here originally with a callback ret status log
}
void AidlComposerClient::OnVsyncPeriodTimingChanged(
    void *callback_data, int64_t in_display, const VsyncPeriodChangeTimeline &in_updated_timeline) {
  auto client = reinterpret_cast<AidlComposerClient *>(callback_data);
  if (!client->callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }
  client->callback_->onVsyncPeriodTimingChanged(in_display, in_updated_timeline);
  // hwc2_callback_data_t used here originally with a callback ret status log
}
void AidlComposerClient::OnVsyncIdle(void *callback_data, int64_t in_display) {
  auto client = reinterpret_cast<AidlComposerClient *>(callback_data);
  if (!client->callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }
  client->callback_->onVsyncIdle(in_display);
}

Error AidlComposerClient::getDisplayReadbackBuffer(int64_t display,
                                                   const native_handle_t *rawHandle,
                                                   const native_handle_t **outHandle) {
  // TODO(user): revisit for caching and freeBuffer in success case.
  if (!mHandleImporter.importBuffer(rawHandle)) {
    ALOGE("%s: ImportBuffer failed.", __FUNCTION__);
    return Error::NoResources;
  }

  std::lock_guard<std::mutex> lock(m_display_data_mutex_);
  auto iter = mDisplayData.find(display);
  if (iter == mDisplayData.end()) {
    mHandleImporter.freeBuffer(rawHandle);
    return Error::BadDisplay;
  }

  *outHandle = rawHandle;
  return Error::None;
}

bool AidlComposerClient::CommandEngine::init() {
  mWriter = std::make_unique<ComposerServiceWriter>();
  return (mWriter != nullptr);
}

Error AidlComposerClient::CommandEngine::execute(const std::vector<DisplayCommand> &commands,
                                                 std::vector<CommandResultPayload> *result) {
  // std::set<int64_t> displaysPendingBrightnessChange;
  mCommandIndex = 0;

  for (const auto &displayCmd : commands) {
    ExecuteCommand(displayCmd.brightness, &CommandEngine::executeSetDisplayBrightness,
                   displayCmd.display, *displayCmd.brightness);
    for (const auto &layerCmd : displayCmd.layers) {
      ExecuteCommand(layerCmd.cursorPosition, &CommandEngine::executeSetLayerCursorPosition,
                     displayCmd.display, layerCmd.layer, *layerCmd.cursorPosition);
      ExecuteCommand(layerCmd.buffer, &CommandEngine::executeSetLayerBuffer, displayCmd.display,
                     layerCmd.layer, *layerCmd.buffer);
      ExecuteCommand(layerCmd.damage, &CommandEngine::executeSetLayerSurfaceDamage,
                     displayCmd.display, layerCmd.layer, *layerCmd.damage);
      ExecuteCommand(layerCmd.blendMode, &CommandEngine::executeSetLayerBlendMode,
                     displayCmd.display, layerCmd.layer, *layerCmd.blendMode);
      ExecuteCommand(layerCmd.composition, &CommandEngine::executeSetLayerComposition,
                     displayCmd.display, layerCmd.layer, *layerCmd.composition);
      // AIDL definiton of LayerCommand Color which calls into executeSetLayerColor:
      // Sets the color of the given layer. If the composition type of the layer is not
      // Composition.SOLID_COLOR, this call must succeed and have no other effect.
      // Since the function depends on composition type to be set, executeSetLayerColor
      // has to be called after executeSetLayerComposition
      ExecuteCommand(layerCmd.color, &CommandEngine::executeSetLayerColor, displayCmd.display,
                     layerCmd.layer, *layerCmd.color);
      ExecuteCommand(layerCmd.dataspace, &CommandEngine::executeSetLayerDataspace,
                     displayCmd.display, layerCmd.layer, *layerCmd.dataspace);
      ExecuteCommand(layerCmd.displayFrame, &CommandEngine::executeSetLayerDisplayFrame,
                     displayCmd.display, layerCmd.layer, *layerCmd.displayFrame);
      ExecuteCommand(layerCmd.planeAlpha, &CommandEngine::executeSetLayerPlaneAlpha,
                     displayCmd.display, layerCmd.layer, *layerCmd.planeAlpha);
      ExecuteCommand(layerCmd.sidebandStream, &CommandEngine::executeSetLayerSidebandStream,
                     displayCmd.display, layerCmd.layer, *layerCmd.sidebandStream);
      ExecuteCommand(layerCmd.sourceCrop, &CommandEngine::executeSetLayerSourceCrop,
                     displayCmd.display, layerCmd.layer, *layerCmd.sourceCrop);
      ExecuteCommand(layerCmd.visibleRegion, &CommandEngine::executeSetLayerVisibleRegion,
                     displayCmd.display, layerCmd.layer, *layerCmd.visibleRegion);
      ExecuteCommand(layerCmd.transform, &CommandEngine::executeSetLayerTransform,
                     displayCmd.display, layerCmd.layer, *layerCmd.transform);
      ExecuteCommand(layerCmd.z, &CommandEngine::executeSetLayerZOrder, displayCmd.display,
                     layerCmd.layer, *layerCmd.z);
      ExecuteCommand(layerCmd.brightness, &CommandEngine::executeSetLayerBrightness,
                     displayCmd.display, layerCmd.layer, *layerCmd.brightness);
      ExecuteCommand(layerCmd.perFrameMetadata, &CommandEngine::executeSetLayerPerFrameMetadata,
                     displayCmd.display, layerCmd.layer, *layerCmd.perFrameMetadata);
      ExecuteCommand(layerCmd.perFrameMetadataBlob,
                     &CommandEngine::executeSetLayerPerFrameMetadataBlobs, displayCmd.display,
                     layerCmd.layer, *layerCmd.perFrameMetadataBlob);
      ExecuteCommand(layerCmd.blockingRegion, &CommandEngine::executeSetLayerBlockingRegion,
                     displayCmd.display, layerCmd.layer, *layerCmd.blockingRegion);
    }
    ExecuteCommand(displayCmd.colorTransformMatrix, &CommandEngine::executeSetColorTransform,
                   displayCmd.display, *displayCmd.colorTransformMatrix);
    ExecuteCommand(displayCmd.clientTarget, &CommandEngine::executeSetClientTarget,
                   displayCmd.display, *displayCmd.clientTarget);
    ExecuteCommand(displayCmd.virtualDisplayOutputBuffer, &CommandEngine::executeSetOutputBuffer,
                   displayCmd.display, *displayCmd.virtualDisplayOutputBuffer);
    ExecuteCommand(displayCmd.validateDisplay, &CommandEngine::executeValidateDisplay,
                   displayCmd.display, displayCmd.expectedPresentTime);
    ExecuteCommand(displayCmd.acceptDisplayChanges, &CommandEngine::executeAcceptDisplayChanges,
                   displayCmd.display);
    ExecuteCommand(displayCmd.presentDisplay, &CommandEngine::executePresentDisplay,
                   displayCmd.display);
    ExecuteCommand(displayCmd.presentOrValidateDisplay,
                   &CommandEngine::executePresentOrValidateDisplay, displayCmd.display,
                   displayCmd.expectedPresentTime);

    ++mCommandIndex;

    // TODO: Process brightness change on presentDisplay if both commands come in?????
    // if (displayCmd.validateDisplay || displayCmd.presentDisplay ||
    //     displayCmd.presentOrValidateDisplay) {
    //   displaysPendingBrightnessChange.erase(displayCmd.display);
    // } else if (DisplayCmd.brightness) {
    //   displaysPendingBrightnessChange.insert(displayCmd.display);
    // }
  }

  if (!mCommandIndex) {
    ALOGW("%s: No command found", __FUNCTION__);
  }

  *result = mWriter->getPendingCommandResults();
  reset();

  return (mCommandIndex) ? Error::None : Error::BadParameter;
}

Error AidlComposerClient::CommandEngine::qtiExecute(const std::vector<QtiDisplayCommand> &commands,
                                                    std::vector<CommandResultPayload> *result) {
  for (const auto &displayCmd : commands) {
    for (const auto &layerCmd : displayCmd.qtiLayers) {
      ExecuteCommand(layerCmd.qtiLayerType, &CommandEngine::executeSetLayerType, displayCmd.display,
                     layerCmd.layer, layerCmd.qtiLayerType);
      ExecuteCommand(layerCmd.qtiLayerFlags, &CommandEngine::executeSetLayerFlag,
                     displayCmd.display, layerCmd.layer, layerCmd.qtiLayerFlags);
    }
    ExecuteCommand(displayCmd.clientTarget_3_1, &CommandEngine::executeSetClientTarget_3_1,
                   displayCmd.display, *displayCmd.clientTarget_3_1);
    ExecuteCommand(displayCmd.time, &CommandEngine::executeSetDisplayElapseTime, displayCmd.display,
                   displayCmd.time);

    ++mCommandIndex;
  }

  if (!mCommandIndex) {
    ALOGW("%s: No command found", __FUNCTION__);
  }

  *result = mWriter->getPendingCommandResults();
  reset();

  return (mCommandIndex) ? Error::None : Error::BadParameter;
}

void AidlComposerClient::CommandEngine::executeSetColorTransform(int64_t display,
                                                                 const std::vector<float> &matrix) {
  auto err = mClient.hwc_session_->SetColorTransform(display, matrix);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetClientTarget(int64_t display,
                                                               const ClientTarget &command) {
  bool useCache = !command.buffer.handle;
  buffer_handle_t clientTarget =
      useCache ? nullptr : ::android::makeFromAidl(*command.buffer.handle);
  native_handle_t *clientTargetClone = const_cast<native_handle_t *>(clientTarget);
  shared_ptr<Fence> fence = nullptr;
  auto &sfd = const_cast<::ndk::ScopedFileDescriptor &>(command.buffer.fence);
  auto fd = sfd.get();
  *sfd.getR() = -1;

  fence = Fence::Create(fd, "fbt");
  if (fence == nullptr) {
    ALOGV("%s: Failed to dup fence %d", __FUNCTION__, fd);
    sync_wait(fd, -1);
  }

  sdm::Region region = {command.damage.size(),
                        reinterpret_cast<Rect const *>(command.damage.data())};
  auto err = lookupBuffer(display, -1, BufferCache::CLIENT_TARGETS, command.buffer.slot, useCache,
                          clientTarget, &clientTarget);
  if (err == Error::None) {
    err = mClient.hwc_session_->SetClientTarget(display, clientTarget, fence,
                                                INT32(command.dataspace), region);
    auto updateBufErr = updateBuffer(display, -1, BufferCache::CLIENT_TARGETS, command.buffer.slot,
                                     useCache, clientTarget);
    if (err == Error::None) {
      err = updateBufErr;
    }
  }

  // Cleanup orginally cloned handle from the input
  native_handle_delete(clientTargetClone);

  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetDisplayBrightness(
    uint64_t display, const DisplayBrightness &command) {
  if (std::isnan(command.brightness) || command.brightness > 1.0f ||
      (command.brightness < 0.0f && command.brightness != -1.0f)) {
    writeError(__FUNCTION__, Error::BadParameter);
    return;
  }

  auto err = mClient.hwc_session_->SetDisplayBrightness(display, command.brightness);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}
void AidlComposerClient::CommandEngine::executeSetOutputBuffer(uint64_t display,
                                                               const Buffer &buffer) {
  bool useCache = !buffer.handle;
  buffer_handle_t outputBuffer = useCache ? nullptr : ::android::makeFromAidl(*buffer.handle);
  native_handle_t *outputBufferClone = const_cast<native_handle_t *>(outputBuffer);
  shared_ptr<Fence> fence = nullptr;
  auto &sfd = const_cast<::ndk::ScopedFileDescriptor &>(buffer.fence);
  auto fd = sfd.get();
  *sfd.getR() = -1;

  fence = Fence::Create(fd, "outbuf");
  if (fence == nullptr) {
    ALOGV("%s: Failed to dup fence %d", __FUNCTION__, fd);
    sync_wait(fd, -1);
  }

  auto err = lookupBuffer(display, -1, BufferCache::OUTPUT_BUFFERS, buffer.slot, useCache,
                          outputBuffer, &outputBuffer);
  if (err == Error::None) {
    err = mClient.hwc_session_->SetOutputBuffer(display, outputBuffer, fence);
    auto updateBufErr =
        updateBuffer(display, -1, BufferCache::OUTPUT_BUFFERS, buffer.slot, useCache, outputBuffer);
    if (err == Error::None) {
      err = updateBufErr;
    }
  }

  // Cleanup orginally cloned handle from the input
  native_handle_delete(outputBufferClone);

  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeValidateDisplay(
    int64_t display, const std::optional<ClockMonotonicTimestamp> expectedPresentTime) {
  executeSetExpectedPresentTimeInternal(display, expectedPresentTime);

  auto err = validateDisplay(display);

  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executePresentOrValidateDisplay(
    int64_t display, const std::optional<ClockMonotonicTimestamp> expectedPresentTime) {
  executeSetExpectedPresentTimeInternal(display, expectedPresentTime);

  // Handle unified commit.
  bool needsCommit = false;
  shared_ptr<Fence> presentFence = nullptr;
  uint32_t typesCount = 0;
  uint32_t reqsCount = 0;
  bool validate_only = false;
  auto status = mClient.hwc_session_->CommitOrPrepare(display, validate_only, &presentFence,
                                                      &typesCount, &reqsCount, &needsCommit);
  if (needsCommit) {
    if (status != Error::None && status != Error::HasChanges) {
      ALOGE("%s: CommitOrPrepare failed %d", __FUNCTION__, status);
    }
    // Implement post validation. Getcomptypes etc;
    postValidateDisplay(display, typesCount, reqsCount);
    mWriter->setPresentOrValidateResult(display, PresentOrValidate::Result::Validated);
  } else {
    if (status == Error::HasChanges) {
      // Perform post validate.
      auto error = postValidateDisplay(display, typesCount, reqsCount);
      if (error == Error::None) {
        mClient.hwc_session_->AcceptDisplayChanges(display);
      }
      // Set result to validated, has comp changes
      mWriter->setPresentOrValidateResult(display, static_cast<PresentOrValidate::Result>(2));
    } else {
      // Set result to Presented.
      mWriter->setPresentOrValidateResult(display, PresentOrValidate::Result::Presented);
    }
    // perform post present display.
    postPresentDisplay(display, &presentFence);
  }
}

void AidlComposerClient::CommandEngine::executeAcceptDisplayChanges(int64_t display) {
  auto err = mClient.hwc_session_->AcceptDisplayChanges(display);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

Error AidlComposerClient::CommandEngine::presentDisplay(int64_t display,
                                                        shared_ptr<Fence> *presentFence) {
  auto err = mClient.hwc_session_->PresentDisplay(display, presentFence);
  if (err != Error::None) {
    return err;
  }

  return postPresentDisplay(display, presentFence);
}

void AidlComposerClient::CommandEngine::executePresentDisplay(int64_t display) {
  shared_ptr<Fence> presentFence = nullptr;

  auto err = presentDisplay(display, &presentFence);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerCursorPosition(int64_t display,
                                                                      int64_t layer,
                                                                      const Point &cursorPosition) {
  auto err =
      mClient.hwc_session_->SetCursorPosition(display, layer, cursorPosition.x, cursorPosition.y);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerBuffer(int64_t display, int64_t layer,
                                                              const Buffer &buffer) {
  bool useCache = !buffer.handle;
  buffer_handle_t layerBuffer = useCache ? nullptr : ::android::makeFromAidl(*buffer.handle);
  native_handle_t *layerBufferClone = const_cast<native_handle_t *>(layerBuffer);
  shared_ptr<Fence> fence = nullptr;
  auto &sfd = const_cast<::ndk::ScopedFileDescriptor &>(buffer.fence);
  auto fd = sfd.get();
  *sfd.getR() = -1;

  fence = Fence::Create(fd, "layer");
  if (fence == nullptr) {
    ALOGV("%s: Failed to dup fence %d", __FUNCTION__, fd);
    sync_wait(fd, -1);
  }

  auto error = lookupBuffer(display, layer, BufferCache::LAYER_BUFFERS, buffer.slot, useCache,
                            layerBuffer, &layerBuffer);
  if (error == Error::None) {
    error = mClient.hwc_session_->SetLayerBuffer(display, layer, layerBuffer, fence);
    auto updateBufErr = updateBuffer(display, layer, BufferCache::LAYER_BUFFERS, buffer.slot,
                                     useCache, layerBuffer);
    if (static_cast<Error>(error) == Error::None) {
      error = updateBufErr;
    }
  }

  // Cleanup orginally cloned handle from the input
  native_handle_delete(layerBufferClone);

  if (error != Error::None) {
    writeError(__FUNCTION__, error);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerSurfaceDamage(
    int64_t display, int64_t layer, const std::vector<std::optional<Rect>> &damage) {
  // N rectangles
  sdm::Region region = {damage.size(), reinterpret_cast<Rect const *>(damage.data())};
  auto err = mClient.hwc_session_->SetLayerSurfaceDamage(display, layer, region);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerBlendMode(
    int64_t display, int64_t layer, const ParcelableBlendMode &blendMode) {
  auto err = mClient.hwc_session_->SetLayerBlendMode(display, layer, INT32(blendMode.blendMode));
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerColor(int64_t display, int64_t layer,
                                                             const FColor &color) {
  const auto floatColorToUint8Clamped = [](float val) -> uint8_t {
    const auto intVal = static_cast<uint64_t>(std::round(255.0f * val));
    const auto minVal = static_cast<uint64_t>(0);
    const auto maxVal = static_cast<uint64_t>(255);
    return std::clamp(intVal, minVal, maxVal);
  };

  sdm::Color int_color{floatColorToUint8Clamped(color.r), floatColorToUint8Clamped(color.g),
                       floatColorToUint8Clamped(color.b), floatColorToUint8Clamped(color.a)};
  auto err = mClient.hwc_session_->SetLayerColor(display, layer, int_color);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerComposition(
    int64_t display, int64_t layer, const ParcelableComposition &composition) {
  auto err =
      mClient.hwc_session_->SetLayerCompositionType(display, layer, INT32(composition.composition));
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerDataspace(
    int64_t display, int64_t layer, const ParcelableDataspace &dataspace) {
  auto err = mClient.hwc_session_->SetLayerDataspace(display, layer, INT32(dataspace.dataspace));
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerDisplayFrame(int64_t display, int64_t layer,
                                                                    const Rect &rect) {
  auto err = mClient.hwc_session_->SetLayerDisplayFrame(display, layer, rect);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerPlaneAlpha(int64_t display, int64_t layer,
                                                                  const PlaneAlpha &planeAlpha) {
  auto err = mClient.hwc_session_->SetLayerPlaneAlpha(display, layer, planeAlpha.alpha);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerSidebandStream(
    int64_t display, int64_t layer, const NativeHandle &sidebandStream) {
  // Sideband stream is not supported
}

void AidlComposerClient::CommandEngine::executeSetLayerSourceCrop(int64_t display, int64_t layer,
                                                                  const FRect &sourceCrop) {
  auto err = mClient.hwc_session_->SetLayerSourceCrop(display, layer, sourceCrop);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerTransform(
    int64_t display, int64_t layer, const ParcelableTransform &transform) {
  // TODO: Remove this catch block for invalid rotation hint after a fix is found
  Transform layer_transform = transform.transform;
  if (INT32(layer_transform) == 128)
    layer_transform = Transform::NONE;

  auto err = mClient.hwc_session_->SetLayerTransform(display, layer, layer_transform);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerVisibleRegion(
    int64_t display, int64_t layer, const std::vector<std::optional<Rect>> &visibleRegion) {
  sdm::Region region = {visibleRegion.size(), reinterpret_cast<Rect const *>(visibleRegion.data())};
  auto err = mClient.hwc_session_->SetLayerVisibleRegion(display, layer, region);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerZOrder(int64_t display, int64_t layer,
                                                              const ZOrder &zOrder) {
  auto err = mClient.hwc_session_->SetLayerZOrder(display, layer, zOrder.z);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerPerFrameMetadata(
    int64_t display, int64_t layer,
    const std::vector<std::optional<PerFrameMetadata>> &perFrameMetadata) {
  std::vector<int32_t> keys;
  std::vector<float> values;

  for (const auto &m : perFrameMetadata) {
    keys.push_back(INT32(m->key));
    values.push_back(static_cast<float>(m->value));
  }

  auto err = mClient.hwc_session_->SetLayerPerFrameMetadata(display, layer, perFrameMetadata.size(),
                                                            keys.data(), values.data());
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerColorTransform(
    int64_t display, int64_t layer, const std::vector<float> &colorTransform) {
  auto err = mClient.hwc_session_->SetLayerColorTransform(display, layer, colorTransform.data());
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerPerFrameMetadataBlobs(
    int64_t display, int64_t layer,
    const std::vector<std::optional<PerFrameMetadataBlob>> &perFrameMetadataBlob) {
  std::vector<int32_t> keys;
  std::vector<uint32_t> sizes_of_metablob_;
  std::vector<uint8_t> blob_of_data_;

  for (const auto &m : perFrameMetadataBlob) {
    keys.push_back(INT32(m->key));
    sizes_of_metablob_.push_back(UINT32(m->blob.size()));
    blob_of_data_.insert(blob_of_data_.end(), m->blob.begin(), m->blob.end());
  }

  auto err = mClient.hwc_session_->SetLayerPerFrameMetadataBlobs(
      display, layer, perFrameMetadataBlob.size(), keys.data(), sizes_of_metablob_.data(),
      blob_of_data_.data());
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerBrightness(
    int64_t display, int64_t layer, const LayerBrightness &brightness) {
  auto err = mClient.hwc_session_->SetLayerBrightness(display, layer, brightness.brightness);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetExpectedPresentTimeInternal(
    int64_t display, const std::optional<ClockMonotonicTimestamp> expectedPresentTime) {
  if (!expectedPresentTime.has_value()) {
    return;
  }

  uint64_t expectedPresentTimestamp = 0;
  if (expectedPresentTime->timestampNanos > 0) {
    expectedPresentTimestamp = static_cast<uint64_t>(expectedPresentTime->timestampNanos);
  }

  auto err = mClient.hwc_session_->SetExpectedPresentTime(display, expectedPresentTimestamp);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerBlockingRegion(
    int64_t display, int64_t layer, const std::vector<std::optional<Rect>> &blockingRegion) {
  // TODO: Add impl here and in hwc_session / hwc_display
  //   auto err = mClient.hwc_session_->SetLayerBlockingRegion(display, blockingRegion);
  //   if (err != Error::None) {
  //     writeError(__FUNCTION__, err);
  //   }
  // writeError(__FUNCTION__, Error::Unsupported);
}

Error AidlComposerClient::CommandEngine::validateDisplay(int64_t display) {
  bool validate_only = true;
  bool needsCommit = false;
  uint32_t types_count = 0;
  uint32_t reqs_count = 0;
  shared_ptr<Fence> presentFence = nullptr;

  auto err = mClient.hwc_session_->CommitOrPrepare(display, validate_only, &presentFence,
                                                   &types_count, &reqs_count, &needsCommit);
  if (err != Error::None && err != Error::HasChanges) {
    return err;
  }

  return postValidateDisplay(display, types_count, reqs_count);
}

Error AidlComposerClient::CommandEngine::postPresentDisplay(int64_t display,
                                                            shared_ptr<Fence> *presentFence) {
  uint32_t count = 0;
  auto err = mClient.hwc_session_->GetReleaseFences(display, &count, nullptr, nullptr);
  if (err != Error::None) {
    ALOGW("%s: Failed to get release fences", __FUNCTION__);
    return Error::None;
  }

  std::vector<sdm::LayerId> layers;
  std::vector<shared_ptr<Fence>> releaseFences;
  std::vector<::ndk::ScopedFileDescriptor> aidlReleaseFences;
  layers.resize(count);
  releaseFences.resize(count);
  err = mClient.hwc_session_->GetReleaseFences(display, &count, layers.data(), &releaseFences);
  if (err != Error::None) {
    ALOGW("%s: Failed to get release fences", __FUNCTION__);
    layers.clear();
    releaseFences.clear();
    return Error::None;
  }

  // Convert from Fence to ScopedFileDescriptor
  for (auto const &fd : releaseFences) {
    aidlReleaseFences.emplace_back(::ndk::ScopedFileDescriptor(Fence::Dup(fd)));
  }

  mWriter->setPresentFence(display,
                           std::move(::ndk::ScopedFileDescriptor(Fence::Dup(*presentFence))));
  mWriter->setReleaseFences(display, layers, std::move(aidlReleaseFences));

  return Error::None;
}

Error AidlComposerClient::CommandEngine::postValidateDisplay(int64_t display, uint32_t &types_count,
                                                             uint32_t &reqs_count) {
  std::vector<sdm::LayerId> changedLayers;
  std::vector<Composition> compositionTypes;
  std::vector<sdm::LayerId> requestedLayers;
  std::vector<int32_t> requestMasks;
  ClientTargetProperty clientTargetProperty;
  changedLayers.resize(types_count);
  compositionTypes.resize(types_count);
  auto err =
      mClient.hwc_session_->GetChangedCompositionTypes(display, &types_count, nullptr, nullptr);
  if (err != Error::None) {
    return err;
  }

  err = mClient.hwc_session_->GetChangedCompositionTypes(
      display, &types_count, changedLayers.data(),
      reinterpret_cast<std::underlying_type<Composition>::type *>(compositionTypes.data()));

  if (err != Error::None) {
    changedLayers.clear();
    compositionTypes.clear();
    return static_cast<Error>(err);
  }

  int32_t display_reqs = 0;
  err = mClient.hwc_session_->GetDisplayRequests(display, &display_reqs, &reqs_count, nullptr,
                                                 nullptr);
  if (err != Error::None) {
    changedLayers.clear();
    compositionTypes.clear();
    return err;
  }

  requestedLayers.resize(reqs_count);
  requestMasks.resize(reqs_count);
  err = mClient.hwc_session_->GetDisplayRequests(display, &display_reqs, &reqs_count,
                                                 requestedLayers.data(), requestMasks.data());
  if (err != Error::None) {
    changedLayers.clear();
    compositionTypes.clear();

    requestedLayers.clear();
    requestMasks.clear();
  }

  err = mClient.hwc_session_->GetClientTargetProperty(display, &clientTargetProperty);
  if (err != Error::None) {
    // todo: reset to default values
    return err;
  }

  mWriter->setChangedCompositionTypes(display, static_cast<std::vector<int64_t>>(changedLayers),
                                      compositionTypes);
  mWriter->setDisplayRequests(display, display_reqs,
                              static_cast<std::vector<int64_t>>(requestedLayers), requestMasks);
  static constexpr float kBrightness = 1.f;
  DimmingStage dimmingStage = DimmingStage::NONE;
  mWriter->setClientTargetProperty(display, clientTargetProperty, kBrightness, dimmingStage);

  return err;
}

// TODO: Re-add extensions API
void AidlComposerClient::CommandEngine::executeSetClientTarget_3_1(int64_t display,
                                                                   const ClientTarget &command) {
  bool useCache = true;
  buffer_handle_t clientTarget = nullptr;
  shared_ptr<Fence> fence = nullptr;
  auto &sfd = const_cast<::ndk::ScopedFileDescriptor &>(command.buffer.fence);
  auto fd = sfd.get();
  *sfd.getR() = -1;

  fence = Fence::Create(fd, "fbt");
  if (fence == nullptr) {
    ALOGW("%s: Failed to dup fence %d", __FUNCTION__, fd);
    sync_wait(fd, -1);
  }

  sdm::Region region = {};
  auto err = lookupBuffer(display, -1, BufferCache::CLIENT_TARGETS, command.buffer.slot, useCache,
                          clientTarget, &clientTarget);
  if (err == Error::None) {
    err = mClient.hwc_session_->SetClientTarget_3_1(display, clientTarget, fence,
                                                    INT32(command.dataspace), region);
    auto updateBufErr = updateBuffer(display, -1, BufferCache::CLIENT_TARGETS, command.buffer.slot,
                                     useCache, clientTarget);
    if (err == Error::None) {
      err = updateBufErr;
    }
  }
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetDisplayElapseTime(int64_t display,
                                                                    uint64_t time) {
  auto err = mClient.hwc_session_->SetDisplayElapseTime(display, time);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerType(int64_t display, int64_t layer,
                                                            sdm::LayerType type) {
  auto err = mClient.hwc_session_->SetLayerType(display, layer, type);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerFlag(int64_t display, int64_t layer,
                                                            sdm::LayerFlag flag) {
  auto err = mClient.hwc_session_->SetLayerFlag(display, layer, flag);
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

Error AidlComposerClient::CommandEngine::lookupBufferCacheEntryLocked(
    int64_t display, int64_t layer, BufferCache cache, uint32_t slot, BufferCacheEntry **outEntry) {
  auto dpy = mClient.mDisplayData.find(display);
  if (dpy == mClient.mDisplayData.end()) {
    return Error::BadDisplay;
  }

  BufferCacheEntry *entry = nullptr;
  switch (cache) {
    case BufferCache::CLIENT_TARGETS:
      if (slot < dpy->second.ClientTargets.size()) {
        entry = &dpy->second.ClientTargets[slot];
      }
      break;
    case BufferCache::OUTPUT_BUFFERS:
      if (slot < dpy->second.OutputBuffers.size()) {
        entry = &dpy->second.OutputBuffers[slot];
      }
      break;
    case BufferCache::LAYER_BUFFERS: {
      auto ly = dpy->second.Layers.find(layer);
      if (ly == dpy->second.Layers.end()) {
        return Error::BadLayer;
      }
      if (slot < ly->second.Buffers.size()) {
        entry = &ly->second.Buffers[slot];
      }
    } break;
    case BufferCache::LAYER_SIDEBAND_STREAMS: {
      auto ly = dpy->second.Layers.find(layer);
      if (ly == dpy->second.Layers.end()) {
        return Error::BadLayer;
      }
      if (slot == 0) {
        entry = &ly->second.SidebandStream;
      }
    } break;
    default:
      break;
  }

  if (!entry) {
    ALOGW("%s: Invalid buffer slot %" PRIu32, __FUNCTION__, slot);
    return Error::BadParameter;
  }

  *outEntry = entry;

  return Error::None;
}

Error AidlComposerClient::CommandEngine::lookupBuffer(int64_t display, int64_t layer,
                                                      BufferCache cache, uint32_t slot,
                                                      bool useCache, buffer_handle_t handle,
                                                      buffer_handle_t *outHandle) {
  if (useCache) {
    std::lock_guard<std::mutex> lock(mClient.m_display_data_mutex_);

    BufferCacheEntry *entry;
    Error error = lookupBufferCacheEntryLocked(display, layer, cache, slot, &entry);
    if (error != Error::None) {
      return error;
    }

    // input handle is ignored
    *outHandle = entry->getHandle();
  } else if (cache == BufferCache::LAYER_SIDEBAND_STREAMS) {
    if (handle) {
      *outHandle = native_handle_clone(handle);
      if (*outHandle == nullptr) {
        return Error::NoResources;
      }
    }
  } else {
    if (!mHandleImporter.importBuffer(handle)) {
      return Error::NoResources;
    }

    *outHandle = handle;
  }

  return Error::None;
}

Error AidlComposerClient::CommandEngine::updateBuffer(int64_t display, int64_t layer,
                                                      BufferCache cache, uint32_t slot,
                                                      bool useCache, buffer_handle_t handle) {
  // handle was looked up from cache
  if (useCache) {
    return Error::None;
  }

  std::lock_guard<std::mutex> lock(mClient.m_display_data_mutex_);

  BufferCacheEntry *entry = nullptr;
  Error error = lookupBufferCacheEntryLocked(display, layer, cache, slot, &entry);
  if (error != Error::None) {
    return error;
  }

  *entry = handle;
  return Error::None;
}

SpAIBinder AidlComposerClient::createBinder() {
  auto binder = BnComposerClient::createBinder();
  AIBinder_setInheritRt(binder.get(), true);
  return binder;
}

}  // namespace composer3
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
}  // namespace aidl
