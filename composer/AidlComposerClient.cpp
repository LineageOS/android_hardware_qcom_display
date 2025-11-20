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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "AidlComposerClient.h"
#include "android/binder_auto_utils.h"
#include <android/binder_ibinder_platform.h>

#include <sync/sync.h>

#include "hwc_parcel.h"
#include <fcntl.h>

namespace aidl {
namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace composer3 {

using MetadataType = vendor_qti_hardware_display_common_MetadataType;
using sdm::HWCParcel;

ComposerHandleImporter mHandleImporter;

BufferCacheEntry::BufferCacheEntry() : mHandle(nullptr) {}

BufferCacheEntry::BufferCacheEntry(BufferCacheEntry &&other) {
  mHandle = other.mHandle;
  other.mHandle = nullptr;
}

BufferCacheEntry &BufferCacheEntry::operator=(SnapHandle *handle) {
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

bool AidlComposerClient::init(std::shared_ptr<SDMDisplayCapsIntf> caps,
                              std::shared_ptr<SDMDisplaySettingsIntf> settings,
                              std::shared_ptr<SDMDisplayLifeCycleIntf> lifecycle,
                              std::shared_ptr<SDMDisplayDrawCycleIntf> drawcycle,
                              std::shared_ptr<SDMDisplayLayerBuilderIntf> layers,
                              std::shared_ptr<SDMDisplaySideBandIntf> sideband) {
  if (!caps || !settings || !lifecycle || !drawcycle || !layers) {
    ALOGE("AidlComposerClient::%s: interfaces not ready", __FUNCTION__);
    return false;
  }

  caps_ = caps;
  settings_ = settings;
  lifecycle_ = lifecycle;
  drawcycle_ = drawcycle;
  layer_builder_ = layers;
  sideband_ = sideband;

  qservice_ = QServiceBackend::GetInstance();

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

::android::status_t AidlComposerClient::notifyCallback(uint32_t command,
                                                       const ::android::Parcel *input_parcel,
                                                       ::android::Parcel *output_parcel) {
  // TODO (user) should be part of debug
  HWCParcel in(input_parcel);
  HWCParcel out(output_parcel);

  auto ret = sideband_->NotifyCallback(command, &in, &out);
  if (ret != sdm::kErrorNone) {
    return ::android::FAILED_TRANSACTION;
  }

  return ::android::NO_ERROR;
}

AidlComposerClient::~AidlComposerClient() {
  ALOGW("%s: Destroying composer client", __FUNCTION__);

  lifecycle_->RegisterCompositorCallback(nullptr, false);

  // no need to grab the mutex as any in-flight hwbinder call would have
  // kept the client alive
  for (const auto &dpy : mDisplayData) {
    ALOGW("%s: Destroying client resources for display %" PRIu64, __FUNCTION__, dpy.first);

    for (const auto &ly : dpy.second.Layers) {
      layer_builder_->DestroyLayer(dpy.first, ly.first);
    }

    if (dpy.second.IsVirtual) {
      destroyVirtualDisplay(dpy.first);
    } else {
      ALOGW("%s: Performing a final presentDisplay", __FUNCTION__);

      mCommandEngine->validateDisplay(dpy.first);

      drawcycle_->AcceptDisplayChanges(dpy.first);

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

ScopedAStatus AidlComposerClient::createLayer(int64_t in_display, int32_t in_buffer_slot_count,
                                              int64_t *aidl_return) {
  auto ret = Error::None;

  if (aidl_return && in_display >= 0 && in_buffer_slot_count >= 0) {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    auto dpy = mDisplayData.find(in_display);
    // The display entry may have already been removed by onHotplug.
    if (dpy != mDisplayData.end()) {
      sdm::LayerId layer = 0;
      auto error = layer_builder_->CreateLayer(in_display, &layer);
      if (error == sdm::kErrorNone) {
        *aidl_return = static_cast<int64_t>(layer);
        drawcycle_->LayerStackUpdated(in_display);

        auto ly = dpy->second.Layers.emplace(layer, LayerBuffers()).first;
        ly->second.Buffers.resize(in_buffer_slot_count);
      } else {
        ret = Error::BadLayer;
      }
    } else {
      ret = Error::BadDisplay;
      // Note: We do not destroy the layer on this error as the hotplug
      // disconnect invalidates the display id. The implementation should
      // ensure all layers for the display are destroyed.
    }
  } else {
    ret = Error::BadParameter;
  }

  return TO_BINDER_STATUS(INT32(ret));
}

ScopedAStatus AidlComposerClient::createVirtualDisplay(int32_t in_width, int32_t in_height,
                                                       PixelFormat in_format_hint,
                                                       int32_t in_output_buffer_slot_count,
                                                       VirtualDisplay *aidl_return) {
  int32_t format = static_cast<int32_t>(in_format_hint);
  uint64_t display;
  auto error = lifecycle_->CreateDisplay(sdm::kVirtual, in_width, in_height, &format, &display);
  auto ret = Error::None;

  if (error == sdm::kErrorNone) {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);

    auto dpy = mDisplayData.emplace(static_cast<sdm::Display>(display), DisplayData(true)).first;
    dpy->second.OutputBuffers.resize(in_output_buffer_slot_count);

    aidl_return->display = display;
    aidl_return->format = in_format_hint;
  } else {
    ret = Error::BadDisplay;
  }

  return TO_BINDER_STATUS(INT32(ret));
}

ScopedAStatus AidlComposerClient::destroyLayer(int64_t in_display, int64_t in_layer) {
  {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    if (mDisplayData.find(in_display) == mDisplayData.end()) {
      return TO_BINDER_STATUS(INT32(Error::BadDisplay));
    }
  }

  drawcycle_->WaitForDrawCycleToComplete(in_display);
  auto error = layer_builder_->DestroyLayer(in_display, in_layer);
  drawcycle_->LayerStackUpdated(in_display);

  auto ret = Error::None;

  if (error == sdm::kErrorNone) {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);

    auto dpy = mDisplayData.find(in_display);
    // The display entry may have already been removed by onHotplug.
    if (dpy != mDisplayData.end()) {
      dpy->second.Layers.erase(in_layer);
    }
  } else {
    ret = Error::BadLayer;
  }

  return TO_BINDER_STATUS(INT32(ret));
}

ScopedAStatus AidlComposerClient::destroyVirtualDisplay(int64_t in_display) {
  auto error = lifecycle_->DestroyDisplay(in_display);
  auto ret = Error::None;

  if (error == sdm::kErrorNone) {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);

    mDisplayData.erase(in_display);
  } else {
    ret = Error::BadDisplay;
  }

  return TO_BINDER_STATUS(INT32(ret));
}

ScopedAStatus AidlComposerClient::executeCommands(const std::vector<DisplayCommand> &in_commands,
                                                  std::vector<CommandResultPayload> *aidl_return) {
  std::lock_guard<std::mutex> lock(m_command_mutex_);

  lifecycle_->CompositorSync(sdm::CompositorSyncTypeAcquire);
  Error error = mCommandEngine->execute(in_commands, aidl_return);
  lifecycle_->CompositorSync(sdm::CompositorSyncTypeRelease);

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::executeQtiCommands(
    const std::vector<QtiDisplayCommand> &in_commands,
    std::vector<CommandResultPayload> *aidl_return) {
  std::lock_guard<std::mutex> lock(m_command_mutex_);

  lifecycle_->CompositorSync(sdm::CompositorSyncTypeAcquire);
  Error error = mCommandEngine->qtiExecute(in_commands, aidl_return);
  lifecycle_->CompositorSync(sdm::CompositorSyncTypeRelease);

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getActiveConfig(int64_t in_display, int32_t *aidl_return) {
  {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    if (mDisplayData.find(in_display) == mDisplayData.end()) {
      return TO_BINDER_STATUS(INT32(Error::BadDisplay));
    }
  }

  auto error = settings_->GetActiveConfig(in_display, (sdm::Config *)aidl_return);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getColorModes(int64_t in_display,
                                                std::vector<ColorMode> *aidl_return) {
  uint32_t count = 0;

  {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    if (mDisplayData.find(in_display) == mDisplayData.end()) {
      return TO_BINDER_STATUS(INT32(Error::BadDisplay));
    }
  }

  auto error = settings_->GetColorModes(in_display, &count, nullptr);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  aidl_return->resize(count);
  error = settings_->GetColorModes(
      in_display, &count,
      reinterpret_cast<std::underlying_type<ColorMode>::type *>(aidl_return->data()));
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getDataspaceSaturationMatrix(Dataspace in_dataspace,
                                                               std::vector<float> *aidl_return) {
  if (in_dataspace != Dataspace::SRGB_LINEAR) {
    return TO_BINDER_STATUS(INT32(Error::BadParameter));
  }

  aidl_return->resize(sdm::kDataspaceSaturationMatrixCount);
  auto error = settings_->GetDataspaceSaturationMatrix(static_cast<int32_t>(in_dataspace),
                                                       aidl_return->data());
  if (error != sdm::kErrorNone) {
    *aidl_return = {
        1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    };

    return TO_BINDER_STATUS(INT32(Error::Unsupported));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getDisplayAttribute(int64_t in_display, int32_t in_config,
                                                      DisplayAttribute in_attribute,
                                                      int32_t *aidl_return) {
  DisplayConfigVariableInfo attributes{};
  auto error = settings_->GetDisplayAttributes(in_display, in_config, &attributes);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }

  switch (in_attribute) {
    case DisplayAttribute::VSYNC_PERIOD:
      *aidl_return = INT32(attributes.vsync_period_ns);
      break;
    case DisplayAttribute::WIDTH:
      *aidl_return = INT32(attributes.x_pixels);
      break;
    case DisplayAttribute::HEIGHT:
      *aidl_return = INT32(attributes.y_pixels);
      break;
    case DisplayAttribute::DPI_X:
      *aidl_return = INT32(attributes.x_dpi * 1000.0f);
      break;
    case DisplayAttribute::DPI_Y:
      *aidl_return = INT32(attributes.y_dpi * 1000.0f);
      break;
    case DisplayAttribute::CONFIG_GROUP:
      *aidl_return = settings_->GetDisplayConfigGroup(in_display, attributes);
      break;
    default:
      ALOGW("Spurious attribute type");
      *aidl_return = -1;
      return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

#ifdef COMPOSER3_V3
ScopedAStatus AidlComposerClient::getDisplayConfigurations(
    int64_t in_display, int32_t maxFrameIntervalNs,
    std::vector<DisplayConfiguration> *out_configs) {
  std::map<uint32_t, sdm::DisplayConfigVariableInfo> info;

  bool enable_vrr = settings_->SetupVRRConfig(in_display) == sdm::kErrorNone;
  auto error = settings_->GetAllDisplayAttributes(in_display, &info);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }

  out_configs->clear();
  out_configs->reserve(info.size());

  for (const auto &[config_id, variable_config] : info) {
    DisplayConfiguration display_configuration;
    display_configuration.configId = config_id;
    display_configuration.width = variable_config.x_pixels;
    display_configuration.height = variable_config.y_pixels;
    display_configuration.dpi = {static_cast<float>(variable_config.x_dpi),
                                 static_cast<float>(variable_config.y_dpi)};
    display_configuration.vsyncPeriod = variable_config.vsync_period_ns;
    display_configuration.configGroup =
        settings_->GetDisplayConfigGroup(in_display, variable_config);

    ALOGI("GetDisplayConfigurations ConfigId[%d] vsyncPeriod= %d, configGroup= %d, fps= %d",
          config_id, variable_config.vsync_period_ns, display_configuration.configGroup,
          variable_config.fps);

    if (enable_vrr && variable_config.avr_step > 0) {
      display_configuration.vrrConfig = {
          static_cast<int32_t>((1000.f / static_cast<float>(variable_config.fps)) * 1000000),
          {},
          {}};
      int notify_ept_threshold_value = settings_->GetNotifyEptConfig(in_display);
      if (variable_config.early_ept_timeout > 0 && notify_ept_threshold_value > 0) {
        int notify_ept_heads_up =
            static_cast<int32_t>((1000.f / static_cast<float>(variable_config.fps)) * 1000000 *
                                 notify_ept_threshold_value);
        display_configuration.vrrConfig->notifyExpectedPresentConfig = {
            notify_ept_heads_up, static_cast<int32_t>(variable_config.early_ept_timeout)};
        ALOGI("NotifyEPT config early_ept_timeout= %d ns, notify_ept_heads_up= %d ns",
              variable_config.early_ept_timeout, notify_ept_heads_up);
      }
    }

    out_configs->emplace_back(display_configuration);
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::notifyExpectedPresent(
    int64_t in_display, const ClockMonotonicTimestamp &expected_present_time,
    int32_t frame_interval_ns) {
  if (frame_interval_ns <= 0 || expected_present_time.timestampNanos <= 0) {
    return TO_BINDER_STATUS(INT32(Error::BadParameter));
  }

  uint64_t ept = static_cast<uint64_t>(expected_present_time.timestampNanos);
  uint32_t fi_ns = (uint32_t)frame_interval_ns;

  auto error = drawcycle_->NotifyExpectedPresent(in_display, ept, fi_ns);
  return TO_BINDER_STATUS(INT32(error));
}
#endif

ScopedAStatus AidlComposerClient::getDisplayCapabilities(
    int64_t in_display, std::vector<DisplayCapability> *aidl_return) {
  // Client queries per display capabilities which gets populated here

  std::lock_guard<std::mutex> lock(m_display_data_mutex_);
  if (mDisplayData.find(in_display) == mDisplayData.end()) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }

  HwcDisplayConnectionType display_conn_type = HwcDisplayConnectionType::INTERNAL;
  sdm::DisplayClass display_class;
  auto ret = caps_->GetDisplayConnectionType(in_display, &display_class);
  if (sdm::kErrorNone != ret) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }

  display_conn_type = HwcDisplayConnectionType::EXTERNAL;
  if (display_class == sdm::DISPLAY_CLASS_BUILTIN) {
    display_conn_type = HwcDisplayConnectionType::INTERNAL;
  }

  if (HwcDisplayConnectionType::INTERNAL == display_conn_type) {
    // SKIP_CLIENT_COLOR_TRANSFORM is used to prevent client from applying color transform on the
    // client composed layers. Since DSPP would apply color transform on the final composed output,
    // this is needed to prevent applying color transform twice. When client queries per display
    // capabilities, the global Capability::SKIP_CLIENT_COLOR_TRANSFORM is ignored. We need to push
    // DisplayCapability::SKIP_CLIENT_COLOR_TRANSFORM here to maintain support.
    aidl_return->push_back(DisplayCapability::SKIP_CLIENT_COLOR_TRANSFORM);
    int32_t has_doze_support = 0;
    caps_->GetDozeSupport(in_display, &has_doze_support);
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
  auto error = caps_->GetDisplayConfigs(in_display, aidl_return);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getDisplayConnectionType(int64_t in_display,
                                                           DisplayConnectionType *aidl_return) {
  sdm::DisplayClass display_class;
  auto ret = caps_->GetDisplayConnectionType(in_display, &display_class);
  if (sdm::kErrorNone != ret) {
    return TO_BINDER_STATUS(INT32(Error::BadDisplay));
  }

  *aidl_return = HwcDisplayConnectionType::EXTERNAL;
  if (display_class == sdm::DISPLAY_CLASS_BUILTIN) {
    *aidl_return = HwcDisplayConnectionType::INTERNAL;
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}
ScopedAStatus AidlComposerClient::getDisplayIdentificationData(int64_t in_display,
                                                               DisplayIdentification *aidl_return) {
  uint8_t port = 0;
  uint32_t size = 0;

  auto error = caps_->GetDisplayIdentificationData(in_display, &port, &size, nullptr);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  aidl_return->data.resize(size);
  error = caps_->GetDisplayIdentificationData(in_display, &port, &size, aidl_return->data.data());
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }
  aidl_return->port = port;

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getDisplayName(int64_t in_display, std::string *aidl_return) {
  uint32_t count = 0;
  // std::vector<char> name;

  auto error = settings_->GetDisplayName(in_display, &count, nullptr);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  aidl_return->resize(count + 1);
  error = settings_->GetDisplayName(in_display, &count, aidl_return->data());
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  aidl_return->at(count) = '\0';
  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getDisplayVsyncPeriod(int64_t in_display, int32_t *aidl_return) {
  {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    if (mDisplayData.find(in_display) == mDisplayData.end()) {
      return TO_BINDER_STATUS(INT32(Error::BadDisplay));
    }
  }

  sdm::VsyncPeriodNanos vsync_period;
  auto error = settings_->GetDisplayVsyncPeriod(in_display, &vsync_period);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  *aidl_return = INT32(vsync_period);
  return TO_BINDER_STATUS(INT32(Error::None));
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

  auto error =
      caps_->GetHdrCapabilities(in_display, &count, nullptr, &aidl_return->maxLuminance,
                                &aidl_return->maxAverageLuminance, &aidl_return->minLuminance);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  aidl_return->types.resize(count);
  error = caps_->GetHdrCapabilities(
      in_display, &count,
      reinterpret_cast<std::underlying_type<Hdr>::type *>(aidl_return->types.data()),
      &aidl_return->maxLuminance, &aidl_return->maxAverageLuminance, &aidl_return->minLuminance);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getMaxVirtualDisplayCount(int32_t *aidl_return) {
  *aidl_return = caps_->GetMaxVirtualDisplayCount();

  return ScopedAStatus::ok();
}

ScopedAStatus AidlComposerClient::getOverlaySupport(OverlayProperties *aidl_return) {
  // All individually supported properties by hardware
  static std::vector<PixelFormat> pixel_formats{
      PixelFormat::RGBA_8888,    PixelFormat::RGBX_8888,    PixelFormat::RGB_888,
      PixelFormat::RGB_565,      PixelFormat::BGRA_8888,    PixelFormat::YV12,
      PixelFormat::YCRCB_420_SP, PixelFormat::RGBA_1010102, PixelFormat::RGBA_FP16};
  static std::vector<Dataspace> dataspace_standards{
      Dataspace::STANDARD_BT709,  Dataspace::STANDARD_BT601_625, Dataspace::STANDARD_BT601_525,
      Dataspace::STANDARD_BT2020, Dataspace::STANDARD_ADOBE_RGB, Dataspace::STANDARD_DCI_P3};
  static std::vector<Dataspace> dataspace_transfers{
      Dataspace::TRANSFER_SRGB, Dataspace::TRANSFER_GAMMA2_2, Dataspace::TRANSFER_SMPTE_170M,
      Dataspace::TRANSFER_LINEAR};
  static std::vector<Dataspace> dataspace_ranges{Dataspace::RANGE_FULL, Dataspace::RANGE_LIMITED,
                                                 Dataspace::RANGE_EXTENDED};
  static bool mixed_colorspaces_support = true;

  OverlayProperties::SupportedBufferCombinations supported_combination;

  // Combination 1 - All support pixel formats work for all supported colorspaces
  // Since all pixel formats work for all colorspaces only 1 entry is required
  supported_combination.pixelFormats = std::move(pixel_formats);
  supported_combination.standards = std::move(dataspace_standards);
  supported_combination.transfers = std::move(dataspace_transfers);
  supported_combination.ranges = std::move(dataspace_ranges);

  aidl_return->combinations.emplace_back(supported_combination);
  aidl_return->supportMixedColorSpaces = mixed_colorspaces_support;

  return TO_BINDER_STATUS(INT32(Error::None));
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

  sdm::DisplayConfigFixedInfo fixed_info{};
  auto error = caps_->GetFixedConfig(in_display, &fixed_info);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  uint32_t num_keys = 0;
  if (fixed_info.hdr_plus_supported) {
    num_keys = UINT32(PerFrameMetadataKey::HDR10_PLUS_SEI) + 1;
  } else {
    num_keys = UINT32(PerFrameMetadataKey::MAX_FRAME_AVERAGE_LIGHT_LEVEL) + 1;
  }

  aidl_return->resize(num_keys);

  for (int32_t i = 0; i < num_keys; i++) {
    (*aidl_return)[i] = static_cast<PerFrameMetadataKey>(i);
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getReadbackBufferAttributes(
    int64_t in_display, ReadbackBufferAttributes *aidl_return) {
  int32_t format = -1;
  int32_t dataspace = -1;

  auto error = settings_->GetReadbackBufferAttributes(in_display, &format, &dataspace);
  if (error != sdm::kErrorNone) {
    format = -1;
    dataspace = -1;
  }

  aidl_return->format = static_cast<PixelFormat>(format);
  aidl_return->dataspace = static_cast<Dataspace>(dataspace);

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getReadbackBufferFence(int64_t in_display,
                                                         ::ndk::ScopedFileDescriptor *aidl_return) {
  shared_ptr<Fence> fence = nullptr;
  auto error = settings_->GetReadbackBufferFence(in_display, &fence);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::Unsupported));
  }

  *aidl_return = ::ndk::ScopedFileDescriptor(Fence::Dup(fence));

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::getRenderIntents(int64_t in_display, ColorMode in_mode,
                                                   std::vector<RenderIntent> *aidl_return) {
  uint32_t count = 0;

  {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    if (mDisplayData.find(in_display) == mDisplayData.end()) {
      return TO_BINDER_STATUS(INT32(Error::BadDisplay));
    }
  }

  auto error = settings_->GetRenderIntents(in_display, int32_t(in_mode), &count, nullptr);
  if (error != sdm::kErrorNone) {
    if (error == sdm::kErrorParameters) {
      return TO_BINDER_STATUS(INT32(Error::BadParameter));
    } else {
      return TO_BINDER_STATUS(INT32(Error::Unsupported));
    }
  }

  aidl_return->resize(count);
  error = settings_->GetRenderIntents(
      in_display, int32_t(in_mode), &count,
      reinterpret_cast<std::underlying_type<RenderIntent>::type *>(aidl_return->data()));
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
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
  uint32_t format;
  uint32_t alpha;
  auto error = settings_->getDisplayDecorationSupport(in_display, &format, &alpha);
  if (error != sdm::kErrorNone)
    return TO_BINDER_STATUS(INT32(Error::Unsupported));

  aidl_return->emplace();
  aidl_return->value().alphaInterpretation = static_cast<AlphaInterpretation>(alpha);
  aidl_return->value().format = static_cast<PixelFormat>(format);

  return ScopedAStatus::ok();
}

ScopedAStatus AidlComposerClient::registerCallback(
    const std::shared_ptr<IComposerCallback> &in_callback) {
  callback_ = in_callback;
  lifecycle_->RegisterCompositorCallback(this, in_callback != nullptr);
  return ScopedAStatus::ok();
}

ScopedAStatus AidlComposerClient::setActiveConfig(int64_t in_display, int32_t in_config) {
  auto error = settings_->SetActiveConfig(in_display, in_config);
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::setActiveConfigWithConstraints(
    int64_t in_display, int32_t in_config,
    const VsyncPeriodChangeConstraints &in_vsync_period_change_constraints,
    VsyncPeriodChangeTimeline *aidl_return) {
  {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    if (mDisplayData.find(in_display) == mDisplayData.end()) {
      return TO_BINDER_STATUS(INT32(Error::BadDisplay));
    }
  }

  sdm::SDMVsyncPeriodChangeTimeline timeline{};
  sdm::SDMVsyncPeriodChangeConstraints constraints{};

  constraints.desiredTimeNanos = in_vsync_period_change_constraints.desiredTimeNanos;
  constraints.seamlessRequired = in_vsync_period_change_constraints.seamlessRequired;

  auto error =
      settings_->SetActiveConfigWithConstraints(in_display, in_config, &constraints, &timeline);
  if (error != sdm::kErrorNone) {
    auto ret_err = Error::BadConfig;
    if (error == sdm::kSeamlessNotAllowed) {
      ret_err = Error::SeamlessNotAllowed;
    }
    return TO_BINDER_STATUS(INT32(ret_err));
  }

  aidl_return->newVsyncAppliedTimeNanos = timeline.newVsyncAppliedTimeNanos;
  aidl_return->refreshRequired = timeline.refreshRequired;
  aidl_return->refreshTimeNanos = timeline.refreshTimeNanos;

  return TO_BINDER_STATUS(INT32(Error::None));
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
  {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    if (mDisplayData.find(in_display) == mDisplayData.end()) {
      return TO_BINDER_STATUS(INT32(Error::BadDisplay));
    }
  }

  auto error = settings_->SetColorModeWithRenderIntent(in_display, static_cast<int32_t>(in_mode),
                                                       static_cast<int32_t>(in_intent));
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadParameter));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
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
  {
    std::lock_guard<std::mutex> lock(m_display_data_mutex_);
    if (mDisplayData.find(in_display) == mDisplayData.end()) {
      return TO_BINDER_STATUS(INT32(Error::BadDisplay));
    }
  }

  auto error = lifecycle_->SetPowerMode(in_display, static_cast<int32_t>(in_mode));
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadParameter));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::setReadbackBuffer(
    int64_t in_display, const NativeHandle &in_buffer,
    const ::ndk::ScopedFileDescriptor &in_release_fence) {
  shared_ptr<Fence> fence = nullptr;
  const SnapHandle *buffer = sdm::ConvertToSnapHandle(in_buffer);
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

  auto error = getDisplayReadbackBuffer(in_display, buffer);
  if (error != Error::None) {
    return TO_BINDER_STATUS(INT32(error));
  }

  auto err = settings_->SetReadbackBuffer(in_display, (void *)buffer, fence);
  if (err != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadParameter));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}

ScopedAStatus AidlComposerClient::setVsyncEnabled(int64_t in_display, bool in_enabled) {
  auto error = drawcycle_->SetVsyncEnabled(in_display, static_cast<int32_t>(in_enabled));
  if (error != sdm::kErrorNone) {
    return TO_BINDER_STATUS(INT32(Error::BadConfig));
  }

  return TO_BINDER_STATUS(INT32(Error::None));
}
ScopedAStatus AidlComposerClient::setIdleTimerEnabled(int64_t in_display, int32_t in_timeoutMs) {
  return TO_BINDER_STATUS(INT32(Error::Unsupported));
}

void AidlComposerClient::OnHotplug(uint64_t in_display, bool in_connected) {
  if (!callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }
  if (in_connected) {
    std::lock_guard<std::mutex> lock_d(m_display_data_mutex_);
    mDisplayData.emplace(in_display, DisplayData(false));
  }

  callback_->onHotplug(in_display, in_connected);

  if (!in_connected) {
    // Trigger refresh to make sure disconnect event received/updated properly by SurfaceFlinger.
    drawcycle_->Refresh(sdm::HWC_DISPLAY_PRIMARY);
    // Wait for sufficient time to ensure sufficient resources are available to process connection.
    uint32_t vsync_period = 0;
    drawcycle_->GetVsyncPeriod(sdm::HWC_DISPLAY_PRIMARY, &vsync_period);
    usleep(vsync_period * 2 / 1000);

    // Wait for the input command message queue to process before destroying the local display data.
    std::lock_guard<std::mutex> lock(m_command_mutex_);
    std::lock_guard<std::mutex> lock_d(m_display_data_mutex_);
    mDisplayData.erase(in_display);
  }
}

void AidlComposerClient::OnRefresh(uint64_t in_display) {
  if (!callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }
  callback_->onRefresh(in_display);
  // hwc2_callback_data_t used here originally with a callback ret status log
}

void AidlComposerClient::OnSeamlessPossible(uint64_t in_display) {
  if (!callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }

  callback_->onSeamlessPossible(in_display);
}

void AidlComposerClient::OnVsync(uint64_t in_display, int64_t in_timestamp,
                                 int32_t in_vsync_period_nanos) {
  if (!callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }
  callback_->onVsync(in_display, in_timestamp, in_vsync_period_nanos);
  // hwc2_callback_data_t used here originally with a callback ret status log
}

void AidlComposerClient::OnVsyncPeriodTimingChanged(
    uint64_t in_display, const SDMVsyncPeriodChangeTimeline &in_updated_timeline) {
  if (!callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);

    return;
  }

  VsyncPeriodChangeTimeline timeline{};
  timeline.newVsyncAppliedTimeNanos = in_updated_timeline.newVsyncAppliedTimeNanos;
  timeline.refreshRequired = in_updated_timeline.refreshRequired;
  timeline.refreshTimeNanos = in_updated_timeline.refreshTimeNanos;

  callback_->onVsyncPeriodTimingChanged(in_display, timeline);
  // hwc2_callback_data_t used here originally with a callback ret status log
}

void AidlComposerClient::OnVsyncIdle(uint64_t in_display) {
  if (!callback_) {
    ALOGW("%s: Callback not registered or SF is unavailable.", __FUNCTION__);
    return;
  }
  callback_->onVsyncIdle(in_display);
}

Error AidlComposerClient::getDisplayReadbackBuffer(int64_t display,
                                                   const SnapHandle *rawHandle) {
  // TODO(user): revisit for caching and freeBuffer in success case.
  if (!mHandleImporter.importBuffer(rawHandle)) {
    ALOGE("%s: Snapmapper retain failed.", __FUNCTION__);
    return Error::NoResources;
  }

  std::lock_guard<std::mutex> lock(m_display_data_mutex_);
  auto iter = mDisplayData.find(display);
  if (iter == mDisplayData.end()) {
    mHandleImporter.freeBuffer(rawHandle);
    return Error::BadDisplay;
  }

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
    ExecuteCommand(displayCmd.acceptDisplayChanges, &CommandEngine::executeAcceptDisplayChanges,
                   displayCmd.display);
    ExecuteCommand(displayCmd.presentDisplay, &CommandEngine::executePresentDisplay,
                   displayCmd.display);
#ifdef COMPOSER3_V3
    ExecuteCommand(displayCmd.validateDisplay, &CommandEngine::executeValidateDisplay,
                   displayCmd.display, displayCmd.expectedPresentTime, displayCmd.frameIntervalNs);
    ExecuteCommand(displayCmd.presentOrValidateDisplay,
                   &CommandEngine::executePresentOrValidateDisplay, displayCmd.display,
                   displayCmd.expectedPresentTime, displayCmd.frameIntervalNs);
#endif

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
  auto err = mClient.settings_->SetColorTransform(display, matrix);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetClientTarget(int64_t display,
                                                               const ClientTarget &command) {
  bool useCache = !command.buffer.handle;
  SnapHandle *clientTarget = useCache ? nullptr : sdm::ConvertToSnapHandle(*command.buffer.handle);
  shared_ptr<Fence> fence = nullptr;
  auto &sfd = const_cast<::ndk::ScopedFileDescriptor &>(command.buffer.fence);
  auto fd = sfd.get();
  *sfd.getR() = -1;

  fence = Fence::Create(fd, "fbt");
  if (fence == nullptr) {
    ALOGV("%s: Failed to dup fence %d", __FUNCTION__, fd);
    sync_wait(fd, -1);
  }

  uint32_t size = command.damage.size();
  const Rect *rect = reinterpret_cast<const Rect *>(command.damage.data());

  sdm::SDMRegion region = {size, std::vector<sdm::SDMRect>()};
  GetSDMRectFromRect(rect, &region);

  auto err = lookupBuffer(display, -1, BufferCache::CLIENT_TARGETS, command.buffer.slot, useCache,
                          &clientTarget);
  if (err == Error::None) {
    auto error = mClient.drawcycle_->SetClientTarget(display, clientTarget, fence,
                                                     INT32(command.dataspace), region, 0);
    auto updateBufErr = updateBuffer(display, -1, BufferCache::CLIENT_TARGETS, command.buffer.slot,
                                     useCache, clientTarget);
    if (error == sdm::kErrorNone) {
      err = updateBufErr;
    }
  }

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

  auto err = mClient.settings_->SetDisplayBrightness(display, command.brightness, false);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}
void AidlComposerClient::CommandEngine::executeSetOutputBuffer(uint64_t display,
                                                               const Buffer &buffer) {
  bool useCache = !buffer.handle;
  SnapHandle *outputBuffer = useCache ? nullptr : sdm::ConvertToSnapHandle(*buffer.handle);
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
                          &outputBuffer);
  if (err == Error::None) {
    mClient.drawcycle_->SetOutputBuffer(display, outputBuffer, fence);
    auto updateBufErr =
        updateBuffer(display, -1, BufferCache::OUTPUT_BUFFERS, buffer.slot, useCache, outputBuffer);
    if (err == Error::None) {
      err = updateBufErr;
    }
  }

  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeValidateDisplay(
    int64_t display, const std::optional<ClockMonotonicTimestamp> expectedPresentTime,
    int32_t frameIntervalNs) {
  executeSetExpectedPresentTimeInternal(display, expectedPresentTime);
  executeSetFrameIntervalNsInternal(display, frameIntervalNs);

  auto err = validateDisplay(display);

  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executePresentOrValidateDisplay(
    int64_t display, const std::optional<ClockMonotonicTimestamp> expectedPresentTime,
    int32_t frameIntervalNs) {
  executeSetExpectedPresentTimeInternal(display, expectedPresentTime);
  executeSetFrameIntervalNsInternal(display, frameIntervalNs);

  // Handle unified commit.
  bool needsCommit = false;
  shared_ptr<Fence> presentFence = nullptr;
  uint32_t typesCount = 0;
  uint32_t reqsCount = 0;
  bool validate_only = false;
  auto status = mClient.drawcycle_->CommitOrPrepare(display, validate_only, &presentFence,
                                                    &typesCount, &reqsCount, &needsCommit);
  if (needsCommit) {
    if (status != sdm::kErrorNone && status != sdm::kErrorNeedsCommit) {
      ALOGE("%s: CommitOrPrepare failed %d", __FUNCTION__, status);
    }
    // Implement post validation. Getcomptypes etc;
    postValidateDisplay(display, typesCount, reqsCount);
    mWriter->setPresentOrValidateResult(display, PresentOrValidate::Result::Validated);
  } else {
    if (status == sdm::kErrorNeedsCommit) {
      // Perform post validate.
      auto error = postValidateDisplay(display, typesCount, reqsCount);
      if (error == Error::None) {
        mClient.drawcycle_->AcceptDisplayChanges(display);
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
  auto err = mClient.drawcycle_->AcceptDisplayChanges(display);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

Error AidlComposerClient::CommandEngine::presentDisplay(int64_t display,
                                                        shared_ptr<Fence> *presentFence) {
  auto err = mClient.drawcycle_->PresentDisplay(display, presentFence);
  if (err != sdm::kErrorNone) {
    return Error::BadConfig;
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
  if (mClient.layer_builder_->GetDeviceSelectedCompositionType(display, layer) !=
      sdm::SDMCompositionType::COMP_CURSOR) {
    writeError(__FUNCTION__, Error::BadConfig);
    return;
  }

  auto err =
      mClient.settings_->SetCursorPosition(display, layer, cursorPosition.x, cursorPosition.y);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
    return;
  }

  mClient.layer_builder_->SetCursorPosition(display, layer, cursorPosition.x, cursorPosition.y);
}

void AidlComposerClient::CommandEngine::executeSetLayerBuffer(int64_t display, int64_t layer,
                                                              const Buffer &buffer) {
  bool useCache = !buffer.handle;
  SnapHandle *layerBuffer = useCache ? nullptr : sdm::ConvertToSnapHandle(*buffer.handle);
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
                            &layerBuffer);

  if (error == Error::None) {
    auto err = mClient.layer_builder_->SetLayerBuffer(display, layer, layerBuffer, fence);
    auto updateBufErr = updateBuffer(display, layer, BufferCache::LAYER_BUFFERS, buffer.slot,
                                     useCache, layerBuffer);
    if (static_cast<Error>(error) == Error::None) {
      error = updateBufErr;
    }
  }

  if (error != Error::None) {
    writeError(__FUNCTION__, error);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerSurfaceDamage(
    int64_t display, int64_t layer, const std::vector<std::optional<Rect>> &damage) {
  // N rectangles
  const Rect *rect = reinterpret_cast<const Rect *>(damage.data());

  sdm::SDMRegion region = {damage.size(), std::vector<sdm::SDMRect>()};
  GetSDMRectFromRect(rect, &region);

  auto err = mClient.layer_builder_->SetLayerSurfaceDamage(display, layer, region);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerBlendMode(
    int64_t display, int64_t layer, const ParcelableBlendMode &blendMode) {
  int32_t blending = sdm::kBlendingPremultiplied;
  auto mode = static_cast<BlendMode>(blendMode.blendMode);
  switch (mode) {
    case BlendMode::NONE:
      blending = sdm::kBlendingOpaque;
      break;
    case BlendMode::PREMULTIPLIED:
      blending = sdm::kBlendingPremultiplied;
      break;
    case BlendMode::COVERAGE:
      blending = sdm::kBlendingCoverage;
      break;
    case BlendMode::INVALID:
    default:
      writeError(__FUNCTION__, Error::BadConfig);
      return;
  }

  auto err = mClient.layer_builder_->SetLayerBlendMode(display, layer, blending);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
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

  sdm::SDMColor int_color{floatColorToUint8Clamped(color.r), floatColorToUint8Clamped(color.g),
                          floatColorToUint8Clamped(color.b), floatColorToUint8Clamped(color.a)};
  auto err = mClient.layer_builder_->SetLayerColor(display, layer, int_color);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerComposition(
    int64_t display, int64_t layer, const ParcelableComposition &composition) {
  auto err = mClient.layer_builder_->SetLayerCompositionType(display, layer,
                                                             INT32(composition.composition));
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerDataspace(
    int64_t display, int64_t layer, const ParcelableDataspace &dataspace) {
  auto err = mClient.layer_builder_->SetLayerDataspace(display, layer, INT32(dataspace.dataspace));
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerDisplayFrame(int64_t display, int64_t layer,
                                                                    const Rect &rect) {
  uint32_t size = 1;

  sdm::SDMRegion region = {size, std::vector<sdm::SDMRect>()};
  GetSDMRectFromRect(&rect, &region);
  auto err = mClient.layer_builder_->SetLayerDisplayFrame(display, layer, region.rects[0]);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerPlaneAlpha(int64_t display, int64_t layer,
                                                                  const PlaneAlpha &planeAlpha) {
  auto err = mClient.layer_builder_->SetLayerPlaneAlpha(display, layer, planeAlpha.alpha);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerSidebandStream(
    int64_t display, int64_t layer, const NativeHandle &sidebandStream) {
  // Sideband stream is not supported
}

void AidlComposerClient::CommandEngine::executeSetLayerSourceCrop(int64_t display, int64_t layer,
                                                                  const FRect &sourceCrop) {
  sdm::SDMRect rect{};
  rect.left = sourceCrop.left;
  rect.right = sourceCrop.right;
  rect.top = sourceCrop.top;
  rect.bottom = sourceCrop.bottom;

  auto err = mClient.layer_builder_->SetLayerSourceCrop(display, layer, rect);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerTransform(
    int64_t display, int64_t layer, const ParcelableTransform &transform) {
  // TODO: Remove this catch block for invalid rotation hint after a fix is found
  Transform layer_transform = transform.transform;
  if (INT32(layer_transform) == 128)
    layer_transform = Transform::NONE;

  auto err = mClient.layer_builder_->SetLayerTransform(
      display, layer, static_cast<sdm::SDMTransform>(layer_transform));
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerVisibleRegion(
    int64_t display, int64_t layer, const std::vector<std::optional<Rect>> &visibleRegion) {
  uint32_t size = visibleRegion.size();
  const Rect *rect = reinterpret_cast<const Rect *>(visibleRegion.data());

  sdm::SDMRegion region = {size, std::vector<sdm::SDMRect>()};
  GetSDMRectFromRect(rect, &region);

  auto err = mClient.layer_builder_->SetLayerVisibleRegion(display, layer, region);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerZOrder(int64_t display, int64_t layer,
                                                              const ZOrder &zOrder) {
  auto err = mClient.layer_builder_->SetLayerZOrder(display, layer, zOrder.z);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
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

  auto err = mClient.layer_builder_->SetLayerPerFrameMetadata(
      display, layer, perFrameMetadata.size(), keys.data(), values.data());
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerColorTransform(
    int64_t display, int64_t layer, const std::vector<float> &colorTransform) {
  auto err = mClient.layer_builder_->SetLayerColorTransform(display, layer, colorTransform.data());
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
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

  auto err = mClient.layer_builder_->SetLayerPerFrameMetadataBlobs(
      display, layer, perFrameMetadataBlob.size(), keys.data(), sizes_of_metablob_.data(),
      blob_of_data_.data());
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerBrightness(
    int64_t display, int64_t layer, const LayerBrightness &brightness) {
  auto err = mClient.layer_builder_->SetLayerBrightness(display, layer, brightness.brightness);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadParameter);
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

  auto err = mClient.drawcycle_->SetExpectedPresentTime(display, expectedPresentTimestamp);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetFrameIntervalNsInternal(int64_t display,
                                                                          int32_t frameIntervalNs) {
  if (frameIntervalNs <= 0) {
    return;
  }
  uint32_t fi_ns = (uint32_t)frameIntervalNs;
  auto err = mClient.drawcycle_->SetFrameIntervalNs(display, fi_ns);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerBlockingRegion(
    int64_t display, int64_t layer, const std::vector<std::optional<Rect>> &blockingRegion) {
  // TODO: Add impl here and in hwc_session / hwc_display
  //   auto err = mClient.layer_builder_->SetLayerBlockingRegion(display, blockingRegion);
  //   if (err != Error::None) {
  //     writeError(__FUNCTION__, Error::BadConfig);
  //   }
  // writeError(__FUNCTION__, Error::Unsupported);
}

Error AidlComposerClient::CommandEngine::validateDisplay(int64_t display) {
  bool validate_only = true;
  bool needsCommit = false;
  uint32_t types_count = 0;
  uint32_t reqs_count = 0;
  shared_ptr<Fence> presentFence = nullptr;

  auto err = mClient.drawcycle_->CommitOrPrepare(display, validate_only, &presentFence,
                                                 &types_count, &reqs_count, &needsCommit);
  if (err != sdm::kErrorNone && err != sdm::kErrorNeedsCommit) {
    return Error::BadConfig;
  }

  return postValidateDisplay(display, types_count, reqs_count);
}

Error AidlComposerClient::CommandEngine::postPresentDisplay(int64_t display,
                                                            shared_ptr<Fence> *presentFence) {
  uint32_t count = 0;
  auto err = mClient.drawcycle_->GetReleaseFences(display, &count, nullptr, nullptr);
  if (err != sdm::kErrorNone) {
    ALOGW("%s: Failed to get release fences", __FUNCTION__);
    return Error::None;
  }

  std::vector<sdm::LayerId> layers;
  std::vector<shared_ptr<Fence>> releaseFences;
  std::vector<::ndk::ScopedFileDescriptor> aidlReleaseFences;
  layers.resize(count);
  releaseFences.resize(count);
  err = mClient.drawcycle_->GetReleaseFences(display, &count, layers.data(), &releaseFences);
  if (err != sdm::kErrorNone) {
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
      mClient.drawcycle_->GetChangedCompositionTypes(display, &types_count, nullptr, nullptr);
  if (err != sdm::kErrorNone) {
    return Error::BadConfig;
  }

  err = mClient.drawcycle_->GetChangedCompositionTypes(
      display, &types_count, changedLayers.data(),
      reinterpret_cast<std::underlying_type<Composition>::type *>(compositionTypes.data()));

  if (err != sdm::kErrorNone) {
    changedLayers.clear();
    compositionTypes.clear();
    return static_cast<Error>(err);
  }

  int32_t display_reqs = 0;
  err =
      mClient.drawcycle_->GetDisplayRequests(display, &display_reqs, &reqs_count, nullptr, nullptr);
  if (err != sdm::kErrorNone) {
    changedLayers.clear();
    compositionTypes.clear();
    return Error::BadConfig;
  }

  requestedLayers.resize(reqs_count);
  requestMasks.resize(reqs_count);
  err = mClient.drawcycle_->GetDisplayRequests(display, &display_reqs, &reqs_count,
                                               requestedLayers.data(), requestMasks.data());
  if (err != sdm::kErrorNone) {
    changedLayers.clear();
    compositionTypes.clear();

    requestedLayers.clear();
    requestMasks.clear();
  }

  sdm::SDMClientTargetProperty client_property{};
  err = mClient.settings_->GetClientTargetProperty(display, &client_property);
  if (err != sdm::kErrorNone) {
    // todo: reset to default values
    return Error::BadConfig;
  }

  clientTargetProperty.dataspace = static_cast<Dataspace>(client_property.dataspace);
  clientTargetProperty.pixelFormat = static_cast<PixelFormat>(client_property.pixel_format);

  mWriter->setChangedCompositionTypes(display, static_cast<std::vector<int64_t>>(changedLayers),
                                      compositionTypes);
  mWriter->setDisplayRequests(display, display_reqs,
                              static_cast<std::vector<int64_t>>(requestedLayers), requestMasks);
  static constexpr float kBrightness = 1.f;
  DimmingStage dimmingStage = DimmingStage::NONE;
  mWriter->setClientTargetProperty(display, clientTargetProperty, kBrightness, dimmingStage);

  return Error::None;
}

// TODO: Re-add extensions API
void AidlComposerClient::CommandEngine::executeSetClientTarget_3_1(int64_t display,
                                                                   const ClientTarget &command) {
  bool useCache = true;
  SnapHandle *clientTarget = nullptr;
  shared_ptr<Fence> fence = nullptr;
  auto &sfd = const_cast<::ndk::ScopedFileDescriptor &>(command.buffer.fence);
  auto fd = sfd.get();
  *sfd.getR() = -1;

  fence = Fence::Create(fd, "fbt");
  if (fence == nullptr) {
    ALOGW("%s: Failed to dup fence %d", __FUNCTION__, fd);
    sync_wait(fd, -1);
  }

  sdm::SDMRegion region = {};
  auto err = lookupBuffer(display, -1, BufferCache::CLIENT_TARGETS, command.buffer.slot, useCache,
                          &clientTarget);
  if (err == Error::None) {
    auto error = mClient.drawcycle_->SetClientTarget(
        display, clientTarget, fence, INT32(command.dataspace), region, 3 /* version*/);
    auto updateBufErr = updateBuffer(display, -1, BufferCache::CLIENT_TARGETS, command.buffer.slot,
                                     useCache, clientTarget);
    if (error == sdm::kErrorNone) {
      err = updateBufErr;
    }
  }
  if (err != Error::None) {
    writeError(__FUNCTION__, err);
  }
}

void AidlComposerClient::CommandEngine::executeSetDisplayElapseTime(int64_t display,
                                                                    uint64_t time) {
  auto err = mClient.drawcycle_->SetDisplayElapseTime(display, time);
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerType(int64_t display, int64_t layer,
                                                            sdm::LayerType type) {
  auto err =
      mClient.layer_builder_->SetLayerType(display, layer, static_cast<sdm::SDMLayerTypes>(type));
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
  }
}

void AidlComposerClient::CommandEngine::executeSetLayerFlag(int64_t display, int64_t layer,
                                                            sdm::LayerFlag flag) {
  auto err =
      mClient.layer_builder_->SetLayerFlag(display, layer, static_cast<sdm::SDMLayerFlag>(flag));
  if (err != sdm::kErrorNone) {
    writeError(__FUNCTION__, Error::BadConfig);
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
                                                      bool useCache, SnapHandle **handle) {
  if (useCache) {
    std::lock_guard<std::mutex> lock(mClient.m_display_data_mutex_);

    BufferCacheEntry *entry;
    Error error = lookupBufferCacheEntryLocked(display, layer, cache, slot, &entry);
    if (error != Error::None) {
      return error;
    }

    // assign cached handle to given SnapHandle *
    *handle = entry->getHandle();
  } else {
    if (!mHandleImporter.importBuffer(*handle)) {
      return Error::NoResources;
    }
  }

  return Error::None;
}

Error AidlComposerClient::CommandEngine::updateBuffer(int64_t display, int64_t layer,
                                                      BufferCache cache, uint32_t slot,
                                                      bool useCache, SnapHandle *handle) {
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
