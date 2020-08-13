/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright (C) 2017 The Android Open Source Project
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

#include <vector>
#include <string>

#include "QtiQmaaComposerClient.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace composer {
namespace V3_0 {
namespace implementation {

ComposerHandleImporter mHandleImporter;
using ::android::hardware::graphics::composer::V2_1::Error;

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

QtiComposerClient::QtiComposerClient() : mWriter(kWriterInitialSize), mReader(*this) {
  mHandleImporter.initialize();

  default_variable_config_.vsync_period_ns = 16600000;
  default_variable_config_.x_pixels = 1080;
  default_variable_config_.y_pixels = 1920;
  default_variable_config_.x_dpi = 300;
  default_variable_config_.y_dpi = 300;
  default_variable_config_.fps = 60;
  default_variable_config_.is_yuv = false;
}

QtiComposerClient::~QtiComposerClient() {}

void QtiComposerClient::onHotplug(hwc2_callback_data_t callbackData, hwc2_display_t display,
                                  int32_t connected) {
  auto client = reinterpret_cast<QtiComposerClient *>(callbackData);
  auto connect = static_cast<composer_V2_4::IComposerCallback::Connection>(connected);
  if (connect == composer_V2_4::IComposerCallback::Connection::CONNECTED) {
    std::lock_guard<std::mutex> lock_d(client->mDisplayDataMutex);
    client->mDisplayData.emplace(display, DisplayData(false));
  }

  auto ret = client->callback_->onHotplug(display, connect);
  ALOGW_IF(!ret.isOk(), "failed to send onHotplug: %s. SF likely unavailable.",
           ret.description().c_str());

  if (connect == composer_V2_4::IComposerCallback::Connection::DISCONNECTED) {
    // Trigger refresh to make sure disconnect event received/updated properly by SurfaceFlinger.
    // Wait for sufficient time to ensure sufficient resources are available to process connection.
    uint32_t vsync_period = 0;
    usleep(vsync_period * 2 / 1000);

    // Wait for the input command message queue to process before destroying the local display data.
    std::lock_guard<std::mutex> lock(client->mCommandMutex);
    std::lock_guard<std::mutex> lock_d(client->mDisplayDataMutex);
    client->mDisplayData.erase(display);
  }
}

void implementation::QtiComposerClient::onRefresh(hwc2_callback_data_t callbackData,
                                                  hwc2_display_t display) {
  auto client = reinterpret_cast<QtiComposerClient *>(callbackData);
  auto ret = client->callback_->onRefresh(display);
  ALOGW_IF(!ret.isOk(), "failed to send onRefresh: %s. SF likely unavailable.",
           ret.description().c_str());
}

void implementation::QtiComposerClient::onVsync(hwc2_callback_data_t callbackData,
                                                hwc2_display_t display, int64_t timestamp) {
  auto client = reinterpret_cast<QtiComposerClient *>(callbackData);
  auto ret = client->callback_->onVsync(display, timestamp);
  ALOGW_IF(!ret.isOk(), "failed to send onVsync: %s. SF likely unavailable.",
           ret.description().c_str());
}

// Enable these functions and see if stability improves or worsens.
void implementation::QtiComposerClient::onVsync_2_4(hwc2_callback_data_t callbackData,
                                                    hwc2_display_t display, int64_t timestamp,
                                                    VsyncPeriodNanos vsyncPeriodNanos) {
  return;
}

void implementation::QtiComposerClient::onVsyncPeriodTimingChanged(
    hwc2_callback_data_t callbackData, hwc2_display_t display,
    hwc_vsync_period_change_timeline_t *updatedTimeline) {
  VsyncPeriodChangeTimeline timeline = {updatedTimeline->newVsyncAppliedTimeNanos,
                                        static_cast<bool>(updatedTimeline->refreshRequired),
                                        updatedTimeline->refreshTimeNanos};

  auto client = reinterpret_cast<QtiComposerClient *>(callbackData);
  auto ret = client->callback24_->onVsyncPeriodTimingChanged(display, timeline);
  ALOGW_IF(!ret.isOk(), "failed to send onVsyncPeriodTimingChanged: %s. SF likely unavailable.",
           ret.description().c_str());
}

void QtiComposerClient::onSeamlessPossible(hwc2_callback_data_t callbackData,
                                           hwc2_display_t display) {
  auto client = reinterpret_cast<QtiComposerClient *>(callbackData);
  auto ret = client->callback24_->onSeamlessPossible(display);
  ALOGW_IF(!ret.isOk(), "failed to send onSeamlessPossible: %s. SF likely unavailable.",
           ret.description().c_str());
}

// convert fenceFd to or from hidl_handle
// Handle would still own original fence. Hence create a Fence object on duped fd.
Error QtiComposerClient::getFence(const hidl_handle &fenceHandle, shared_ptr<int32_t> *outFence,
                                  const string &name) {
  auto handle = fenceHandle.getNativeHandle();
  if (handle && handle->numFds > 1) {
    ALOGE("invalid fence handle with %d fds", handle->numFds);
    return Error::BAD_PARAMETER;
  }

  return Error::NONE;
}

// Handle would own fence hereafter. Hence provide a dupped fd.
hidl_handle QtiComposerClient::getFenceHandle(const shared_ptr<int32_t> &fence,
                                              char *handleStorage) {
  native_handle_t *handle = nullptr;
  if (fence) {
    handle = native_handle_init(handleStorage, 1, 0);
  }
  return hidl_handle(handle);
}

Error QtiComposerClient::getDisplayReadbackBuffer(Display display, const native_handle_t *rawHandle,
                                                  const native_handle_t **outHandle) {
  // TODO(user): revisit for caching and freeBuffer in success case.
  if (!mHandleImporter.importBuffer(rawHandle)) {
    ALOGE("%s: importBuffer failed: ", __FUNCTION__);
    return Error::NO_RESOURCES;
  }

  std::lock_guard<std::mutex> lock(mDisplayDataMutex);
  auto iter = mDisplayData.find(display);
  if (iter == mDisplayData.end()) {
    mHandleImporter.freeBuffer(rawHandle);
    return Error::BAD_DISPLAY;
  }

  *outHandle = rawHandle;
  return Error::NONE;
}

void implementation::QtiComposerClient::getCapabilities() {
  uint32_t count = 2;
  int32_t *outCapabilities;
  std::vector<int32_t> composer_caps(count);

  outCapabilities = composer_caps.data();
  outCapabilities[0] = HWC2_CAPABILITY_SKIP_CLIENT_COLOR_TRANSFORM;
  outCapabilities[1] = HWC2_CAPABILITY_SKIP_VALIDATE;
  composer_caps.resize(count);

  mCapabilities.reserve(count);
  for (auto cap : composer_caps) {
    mCapabilities.insert(static_cast<hwc2_capability_t>(cap));
  }
}

void implementation::QtiComposerClient::enableCallback(bool enable) {
  return;
}

// Methods from ::android::hardware::graphics::composer::V2_1::IComposerClient follow.
implementation::Return<void> implementation::QtiComposerClient::registerCallback(
    const sp<composer_V2_1::IComposerCallback> &callback) {
  callback_ = callback;

  onHotplug(this, HWC_DISPLAY_PRIMARY, (int32_t)(HWC2::Connection::Connected));

  return Void();
}

implementation::Return<uint32_t> implementation::QtiComposerClient::getMaxVirtualDisplayCount() {
  return -1;
}

implementation::Return<void> implementation::QtiComposerClient::createVirtualDisplay(
    uint32_t width, uint32_t height, common_V1_0::PixelFormat formatHint,
    uint32_t outputBufferSlotCount, createVirtualDisplay_cb _hidl_cb) {
  return Void();
}

implementation::Return<implementation::composer_V2_1::Error>
implementation::QtiComposerClient::destroyVirtualDisplay(uint64_t display) {
  return Error::NONE;
}

implementation::Return<void> implementation::QtiComposerClient::createLayer(
    uint64_t display, uint32_t bufferSlotCount, createLayer_cb _hidl_cb) {
  composer_V2_1::Layer layer = ++layer_count_;
  auto error = HWC2_ERROR_NONE;
  Error err = static_cast<Error>(error);

  _hidl_cb(err, layer);

  return Void();
}

implementation::Return<implementation::Error> implementation::QtiComposerClient::destroyLayer(
    uint64_t display, uint64_t layer) {
  return Error::NONE;
}

implementation::Return<void> implementation::QtiComposerClient::getActiveConfig(
    uint64_t display, getActiveConfig_cb _hidl_cb) {
  uint32_t config = 0;
  auto error = HWC2_ERROR_NONE;

  _hidl_cb(static_cast<Error>(error), config);

  return Void();
}

implementation::Return<implementation::Error>
implementation::QtiComposerClient::getClientTargetSupport(uint64_t display, uint32_t width,
                                                          uint32_t height,
                                                          common_V1_0::PixelFormat format,
                                                          common_V1_0::Dataspace dataspace) {
  return Error::NONE;
}

implementation::Return<void> implementation::QtiComposerClient::getColorModes(
    uint64_t display, getColorModes_cb _hidl_cb) {
  hidl_vec<common_V1_0::ColorMode> modes;
  uint32_t count = 1;

  modes.resize(count);
  auto error = HWC2_ERROR_NONE;
  auto out_modes = reinterpret_cast<ColorMode *>(
      reinterpret_cast<std::underlying_type<common_V1_0::ColorMode>::type *>(modes.data()));
  out_modes[0] = ColorMode::NATIVE;

  _hidl_cb(static_cast<Error>(error), modes);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayAttribute(
    uint64_t display, uint32_t config, composer_V2_1::IComposerClient::Attribute attribute,
    getDisplayAttribute_cb _hidl_cb) {
  int32_t value = 0;
  auto error = HWC2_ERROR_NONE;

  switch (static_cast<composer_V2_4::IComposerClient::Attribute>(attribute)) {
    case HwcAttribute::VSYNC_PERIOD:
      value = (int32_t)(default_variable_config_.vsync_period_ns);
      break;
    case HwcAttribute::WIDTH:
      value = (int32_t)(default_variable_config_.x_pixels);
      break;
    case HwcAttribute::HEIGHT:
      value = (int32_t)(default_variable_config_.y_pixels);
      break;
    case HwcAttribute::DPI_X:
      value = (int32_t)(default_variable_config_.x_dpi * 1000.0f);
      break;
    case HwcAttribute::DPI_Y:
      value = (int32_t)(default_variable_config_.y_dpi * 1000.0f);
      break;
    default:
      value = -1;
      error = HWC2_ERROR_UNSUPPORTED;
  }

  _hidl_cb(static_cast<Error>(error), value);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayConfigs(
    uint64_t display, getDisplayConfigs_cb _hidl_cb) {
  hidl_vec<uint32_t> configs;
  uint32_t count = 1;
  auto error = HWC2_ERROR_NONE;

  configs.resize(count);
  _hidl_cb(static_cast<Error>(error), configs);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayName(
    uint64_t display, getDisplayName_cb _hidl_cb) {
  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayType(
    uint64_t display, getDisplayType_cb _hidl_cb) {
  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDozeSupport(
    uint64_t display, getDozeSupport_cb _hidl_cb) {
  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getHdrCapabilities(
    uint64_t display, getHdrCapabilities_cb _hidl_cb) {
  return Void();
}

implementation::Return<implementation::Error>
implementation::QtiComposerClient::setClientTargetSlotCount(uint64_t display,
                                                            uint32_t clientTargetSlotCount) {
  std::lock_guard<std::mutex> lock(mDisplayDataMutex);

  auto dpy = mDisplayData.find(display);
  if (dpy == mDisplayData.end()) {
    return Error::BAD_DISPLAY;
  }
  dpy->second.ClientTargets.resize(clientTargetSlotCount);

  return Error::NONE;
}

implementation::Return<implementation::Error> implementation::QtiComposerClient::setActiveConfig(
    uint64_t display, uint32_t config) {
  return Error::NONE;
}

implementation::Return<implementation::Error> implementation::QtiComposerClient::setColorMode(
    uint64_t display, common_V1_0::ColorMode mode) {
  return Error::NONE;
}

implementation::Return<implementation::Error> implementation::QtiComposerClient::setPowerMode(
    uint64_t display, composer_V2_1::IComposerClient::PowerMode mode) {
  return Error::NONE;
}

implementation::Return<implementation::Error> implementation::QtiComposerClient::setVsyncEnabled(
    uint64_t display, composer_V2_1::IComposerClient::Vsync enabled) {
  return Error::NONE;
}

implementation::Return<implementation::Error>
implementation::QtiComposerClient::setInputCommandQueue(
    const MQDescriptorSync<uint32_t> &descriptor) {
  std::lock_guard<std::mutex> lock(mCommandMutex);
  return mReader.setMQDescriptor(descriptor) ? Error::NONE : Error::NO_RESOURCES;

  return Error::NONE;
}

implementation::Return<void> implementation::QtiComposerClient::getOutputCommandQueue(
    getOutputCommandQueue_cb _hidl_cb) {
  // no locking as we require this function to be called inside
  // executeCommands_cb

  auto outDescriptor = mWriter.getMQDescriptor();
  if (outDescriptor) {
    _hidl_cb(Error::NONE, *outDescriptor);
  } else {
    _hidl_cb(Error::NO_RESOURCES, MQDescriptorSync<uint32_t>());
  }

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::executeCommands(
    uint32_t inLength, const hidl_vec<hidl_handle> &inHandles, executeCommands_cb _hidl_cb) {
  std::lock_guard<std::mutex> lock(mCommandMutex);
  bool outChanged = false;
  uint32_t outLength = 0;
  hidl_vec<hidl_handle> outHandles;

  if (!mReader.readQueue(inLength, inHandles)) {
    _hidl_cb(Error::BAD_PARAMETER, outChanged, outLength, outHandles);
    return Void();
  }

  Error err = mReader.parse();
  if (err == Error::NONE && !mWriter.writeQueue(outChanged, outLength, outHandles)) {
    err = Error::NO_RESOURCES;
  }

  _hidl_cb(Error::NONE, outChanged, outLength, outHandles);

  mReader.reset();
  mWriter.reset();

  return Void();
}

// Methods from ::android::hardware::graphics::composer::V2_2::IComposerClient follow.
implementation::Return<void> implementation::QtiComposerClient::getPerFrameMetadataKeys(
    uint64_t display, getPerFrameMetadataKeys_cb _hidl_cb) {
  std::vector<PerFrameMetadataKey_V2> keys;
  uint32_t count = (uint32_t)(PerFrameMetadataKey::MAX_FRAME_AVERAGE_LIGHT_LEVEL) + 1;
  auto error = HWC2_ERROR_NONE;

  keys.resize(count);

  _hidl_cb(static_cast<Error>(error), keys);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getReadbackBufferAttributes(
    uint64_t display, getReadbackBufferAttributes_cb _hidl_cb) {
  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getReadbackBufferFence(
    uint64_t display, getReadbackBufferFence_cb _hidl_cb) {
  return Void();
}

implementation::Return<implementation::Error> implementation::QtiComposerClient::setReadbackBuffer(
    uint64_t display, const hidl_handle &buffer, const hidl_handle &releaseFence) {
  return Error::NONE;
}

implementation::Return<void> implementation::QtiComposerClient::createVirtualDisplay_2_2(
    uint32_t width, uint32_t height, common_V1_1::PixelFormat formatHint,
    uint32_t outputBufferSlotCount, createVirtualDisplay_2_2_cb _hidl_cb) {
  int32_t format = static_cast<int32_t>(formatHint);
  uint64_t display = 1;

  auto error = HWC2_ERROR_NONE;
  if (static_cast<Error>(error) == Error::NONE) {
    std::lock_guard<std::mutex> lock(mDisplayDataMutex);

    auto dpy = mDisplayData.emplace(static_cast<Display>(display), DisplayData(true)).first;
    dpy->second.OutputBuffers.resize(outputBufferSlotCount);
  }
  _hidl_cb(static_cast<Error>(error), display, static_cast<common_V1_1::PixelFormat>(format));
  return Void();
}

implementation::Return<implementation::Error>
implementation::QtiComposerClient::getClientTargetSupport_2_2(uint64_t display, uint32_t width,
                                                              uint32_t height,
                                                              common_V1_1::PixelFormat format,
                                                              common_V1_1::Dataspace dataspace) {
  return Error::NONE;
}

implementation::Return<implementation::Error> implementation::QtiComposerClient::setPowerMode_2_2(
    uint64_t display, composer_V2_2::IComposerClient::PowerMode mode) {
  return Error::NONE;
}

implementation::Return<void> implementation::QtiComposerClient::getColorModes_2_2(
    uint64_t display, getColorModes_2_2_cb _hidl_cb) {
  hidl_vec<common_V1_1::ColorMode> modes;
  uint32_t count = 1;

  modes.resize(count);
  auto error = HWC2_ERROR_NONE;
  auto out_modes = reinterpret_cast<ColorMode *>(
      reinterpret_cast<std::underlying_type<common_V1_1::ColorMode>::type *>(modes.data()));
  out_modes[0] = ColorMode::NATIVE;

  _hidl_cb(static_cast<Error>(error), modes);
  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getRenderIntents(
    uint64_t display, common_V1_1::ColorMode mode, getRenderIntents_cb _hidl_cb) {
  return Void();
}

implementation::Return<implementation::Error> implementation::QtiComposerClient::setColorMode_2_2(
    uint64_t display, common_V1_1::ColorMode mode, common_V1_1::RenderIntent intent) {
  return Error::NONE;
}

implementation::Return<void> implementation::QtiComposerClient::getDataspaceSaturationMatrix(
    common_V1_1::Dataspace dataspace, getDataspaceSaturationMatrix_cb _hidl_cb) {
  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::executeCommands_2_2(
    uint32_t inLength, const hidl_vec<hidl_handle> &inHandles, executeCommands_2_2_cb _hidl_cb) {
  std::lock_guard<std::mutex> lock(mCommandMutex);
  bool outChanged = false;
  uint32_t outLength = 0;
  hidl_vec<hidl_handle> outHandles;

  if (!mReader.readQueue(inLength, inHandles)) {
    _hidl_cb(Error::BAD_PARAMETER, outChanged, outLength, outHandles);
    return Void();
  }

  Error err = mReader.parse();
  if (err == Error::NONE && !mWriter.writeQueue(outChanged, outLength, outHandles)) {
    err = Error::NO_RESOURCES;
  }

  _hidl_cb(Error::NONE, outChanged, outLength, outHandles);

  mReader.reset();
  mWriter.reset();

  return Void();
}

// Methods from ::android::hardware::graphics::composer::V2_3::IComposerClient follow.
implementation::Return<void> implementation::QtiComposerClient::getDisplayIdentificationData(
    uint64_t display, getDisplayIdentificationData_cb _hidl_cb) {
  uint8_t port = 1;
  uint32_t size = 0;
  std::vector<uint8_t> data(size);

  auto error = HWC2_ERROR_NONE;
  size = (uint32_t)(edid_.size());
  data.resize(size);
  memcpy(data.data(), edid_.data(), size);

  _hidl_cb(static_cast<Error>(error), port, data);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getReadbackBufferAttributes_2_3(
    uint64_t display, getReadbackBufferAttributes_2_3_cb _hidl_cb) {
  return Void();
}

implementation::Return<implementation::Error>
implementation::QtiComposerClient::getClientTargetSupport_2_3(uint64_t display, uint32_t width,
                                                              uint32_t height,
                                                              common_V1_2::PixelFormat format,
                                                              common_V1_2::Dataspace dataspace) {
  return Error::NONE;
}

implementation::Return<void>
implementation::QtiComposerClient::getDisplayedContentSamplingAttributes(
    uint64_t display, getDisplayedContentSamplingAttributes_cb _hidl_cb) {
  return Void();
}

implementation::Return<implementation::Error>
implementation::QtiComposerClient::setDisplayedContentSamplingEnabled(
    uint64_t display, composer_V2_3::IComposerClient::DisplayedContentSampling enable,
    hidl_bitfield<FormatColorComponent> componentMask, uint64_t maxFrames) {
  return Error::UNSUPPORTED;
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayedContentSample(
    uint64_t display, uint64_t maxFrames, uint64_t timestamp,
    getDisplayedContentSample_cb _hidl_cb) {
  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::executeCommands_2_3(
    uint32_t inLength, const hidl_vec<hidl_handle> &inHandles, executeCommands_2_3_cb _hidl_cb) {
  std::lock_guard<std::mutex> lock(mCommandMutex);

  bool outChanged = false;
  uint32_t outLength = 0;
  hidl_vec<hidl_handle> outHandles;

  if (!mReader.readQueue(inLength, inHandles)) {
    _hidl_cb(Error::BAD_PARAMETER, outChanged, outLength, outHandles);
    return Void();
  }

  Error err = mReader.parse();
  if (err == Error::NONE && !mWriter.writeQueue(outChanged, outLength, outHandles)) {
    err = Error::NO_RESOURCES;
  }

  _hidl_cb(Error::NONE, outChanged, outLength, outHandles);

  mReader.reset();
  mWriter.reset();

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getRenderIntents_2_3(
    uint64_t display, common_V1_2::ColorMode mode, getRenderIntents_2_3_cb _hidl_cb) {
  uint32_t count = 1;
  std::vector<RenderIntent> intents;

  intents.resize(count);
  auto error = HWC2_ERROR_NONE;
  auto out_intents = reinterpret_cast<RenderIntent *>(
      reinterpret_cast<std::underlying_type<RenderIntent>::type *>(intents.data()));
  out_intents[0] = RenderIntent::COLORIMETRIC;

  _hidl_cb(static_cast<Error>(error), intents);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getColorModes_2_3(
    uint64_t display, getColorModes_2_3_cb _hidl_cb) {
  hidl_vec<common_V1_2::ColorMode> modes;
  uint32_t count = 1;

  modes.resize(count);
  auto error = HWC2_ERROR_NONE;
  auto out_modes = reinterpret_cast<ColorMode *>(
      reinterpret_cast<std::underlying_type<common_V1_2::ColorMode>::type *>(modes.data()));
  out_modes[0] = ColorMode::NATIVE;

  _hidl_cb(static_cast<Error>(error), modes);

  return Void();
}

implementation::Return<implementation::Error> implementation::QtiComposerClient::setColorMode_2_3(
    uint64_t display, common_V1_2::ColorMode mode, common_V1_1::RenderIntent intent) {
  return Error::NONE;
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayCapabilities(
    uint64_t display, getDisplayCapabilities_cb _hidl_cb) {
  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getPerFrameMetadataKeys_2_3(
    uint64_t display, getPerFrameMetadataKeys_2_3_cb _hidl_cb) {
  std::vector<PerFrameMetadataKey> keys;
  uint32_t count = (uint32_t)(PerFrameMetadataKey::MAX_FRAME_AVERAGE_LIGHT_LEVEL) + 1;
  auto error = HWC2_ERROR_NONE;

  keys.resize(count);

  _hidl_cb(static_cast<Error>(error), keys);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getHdrCapabilities_2_3(
    uint64_t display, getHdrCapabilities_2_3_cb _hidl_cb) {
  uint32_t count = 0;
  hidl_vec<common_V1_2::Hdr> types;
  float max_lumi = 0.0f;
  float max_avg_lumi = 0.0f;
  float min_lumi = 0.0f;

  types.resize(count);
  auto error = HWC2_ERROR_NONE;

  _hidl_cb(static_cast<Error>(error), types, max_lumi, max_avg_lumi, min_lumi);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayBrightnessSupport(
    uint64_t display, getDisplayBrightnessSupport_cb _hidl_cb) {
  return Void();
}

implementation::Return<implementation::Error>
implementation::QtiComposerClient::setDisplayBrightness(uint64_t display, float brightness) {
  return Error::NONE;
}

// Methods from ::android::hardware::graphics::composer::V2_4::IComposerClient follow.
implementation::Return<void> implementation::QtiComposerClient::registerCallback_2_4(
    const sp<composer_V2_4::IComposerCallback> &callback) {
  callback_ = sp<composer_V2_1::IComposerCallback>(callback.get());
  callback24_ = callback;
  mUseCallback24_ = true;

  onHotplug(this, HWC_DISPLAY_PRIMARY, (int32_t)(HWC2::Connection::Connected));

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayCapabilities_2_4(
    uint64_t display, getDisplayCapabilities_2_4_cb _hidl_cb) {
  hidl_vec<composer_V2_4::IComposerClient::DisplayCapability> capabilities;
  bool isBuiltin = true;
  if (isBuiltin) {
    capabilities.resize(4);
    std::vector<HwcDisplayCapability> caps;
    // For now will not implement DozeSupport path

    caps = {HwcDisplayCapability::SKIP_CLIENT_COLOR_TRANSFORM, HwcDisplayCapability::BRIGHTNESS,
            HwcDisplayCapability::PROTECTED_CONTENTS};

    capabilities.setToExternal(caps.data(), caps.size());
  }

  auto error = HWC2_ERROR_NONE;

  _hidl_cb(static_cast<composer_V2_4::Error>(error), capabilities);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayConnectionType(
    uint64_t display, getDisplayConnectionType_cb _hidl_cb) {
  auto error = HWC2_ERROR_NONE;
  HwcDisplayConnectionType type = HwcDisplayConnectionType::INTERNAL;
  _hidl_cb(static_cast<composer_V2_4::Error>(error), type);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayAttribute_2_4(
    uint64_t display, uint32_t config, composer_V2_4::IComposerClient::Attribute attribute,
    getDisplayAttribute_2_4_cb _hidl_cb) {
  int32_t value = 0;
  auto error = HWC2_ERROR_NONE;

  switch (attribute) {
    case HwcAttribute::VSYNC_PERIOD:
      value = (int32_t)(default_variable_config_.vsync_period_ns);
      break;
    case HwcAttribute::WIDTH:
      value = (int32_t)(default_variable_config_.x_pixels);
      break;
    case HwcAttribute::HEIGHT:
      value = (int32_t)(default_variable_config_.y_pixels);
      break;
    case HwcAttribute::DPI_X:
      value = (int32_t)(default_variable_config_.x_dpi * 1000.0f);
      break;
    case HwcAttribute::DPI_Y:
      value = (int32_t)(default_variable_config_.y_dpi * 1000.0f);
      break;
    default:
      value = -1;
      error = HWC2_ERROR_UNSUPPORTED;
  }

  _hidl_cb(static_cast<composer_V2_4::Error>(error), value);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::getDisplayVsyncPeriod(
    uint64_t display, getDisplayVsyncPeriod_cb _hidl_cb) {
  VsyncPeriodNanos vsync_period;
  auto error = HWC2_ERROR_NONE;
  vsync_period = static_cast<VsyncPeriodNanos>((int32_t)(default_variable_config_.vsync_period_ns));

  _hidl_cb(static_cast<composer_V2_4::Error>(error), vsync_period);

  return Void();
}

implementation::Return<void> implementation::QtiComposerClient::setActiveConfigWithConstraints(
    uint64_t display, uint32_t config,
    const VsyncPeriodChangeConstraints &vsyncPeriodChangeConstraints,
    setActiveConfigWithConstraints_cb _hidl_cb) {
  VsyncPeriodChangeTimeline timeline;
  timeline.refreshRequired = false;
  timeline.refreshTimeNanos = 0;

  auto error = HWC2::Error::SeamlessNotAllowed;

  _hidl_cb(static_cast<composer_V2_4::Error>(error), timeline);

  return Void();
}

implementation::Return<implementation::composer_V2_4::Error>
implementation::QtiComposerClient::setAutoLowLatencyMode(uint64_t display, bool on) {
  return implementation::composer_V2_4::Error::UNSUPPORTED;
}

implementation::Return<void> implementation::QtiComposerClient::getSupportedContentTypes(
    uint64_t display, getSupportedContentTypes_cb _hidl_cb) {
  hidl_vec<composer_V2_4::IComposerClient::ContentType> types = {};
  _hidl_cb(composer_V2_4::Error::NONE, types);

  return Void();
}

implementation::Return<implementation::composer_V2_4::Error>
implementation::QtiComposerClient::setContentType(
    uint64_t display, composer_V2_4::IComposerClient::ContentType type) {
  return composer_V2_4::Error::UNSUPPORTED;
}

implementation::Return<void> implementation::QtiComposerClient::getLayerGenericMetadataKeys(
    getLayerGenericMetadataKeys_cb _hidl_cb) {
  hidl_vec<composer_V2_4::IComposerClient::LayerGenericMetadataKey> keys = {};
  _hidl_cb(composer_V2_4::Error::NONE, keys);

  return Void();
}

QtiComposerClient::CommandReader::CommandReader(QtiComposerClient &client)
    : mClient(client), mWriter(client.mWriter) {}

bool QtiComposerClient::CommandReader::parseCommonCmd(IComposerClient::Command command,
                                                      uint16_t length) {
  bool parsed = false;

  switch (command) {
    // Commands from ::android::hardware::graphics::composer::V2_1::IComposerClient follow.
    case IComposerClient::Command::SELECT_DISPLAY:
      parsed = parseSelectDisplay(length);
      // Displays will not be removed while processing the command queue.
      if (parsed && mClient.mDisplayData.find(mDisplay) == mClient.mDisplayData.end()) {
        ALOGW("Command::SELECT_DISPLAY: Display %lu not found. Dropping commands.", mDisplay);
        mDisplay = 1;
      }
      break;
    case IComposerClient::Command::SELECT_LAYER:
      parsed = parseSelectLayer(length);
      break;
    case IComposerClient::Command::SET_COLOR_TRANSFORM:
      parsed = parseSetColorTransform(length);
      break;
    case IComposerClient::Command::SET_CLIENT_TARGET:
      parsed = parseSetClientTarget(length);
      break;
    case IComposerClient::Command::SET_OUTPUT_BUFFER:
      parsed = parseSetOutputBuffer(length);
      break;
    case IComposerClient::Command::VALIDATE_DISPLAY:
      parsed = parseValidateDisplay(length);
      break;
    case IComposerClient::Command::ACCEPT_DISPLAY_CHANGES:
      parsed = parseAcceptDisplayChanges(length);
      break;
    case IComposerClient::Command::PRESENT_DISPLAY:
      parsed = parsePresentDisplay(length);
      break;
    case IComposerClient::Command::PRESENT_OR_VALIDATE_DISPLAY:
      parsed = parsePresentOrValidateDisplay(length);
      break;
    case IComposerClient::Command::SET_LAYER_CURSOR_POSITION:
      parsed = parseSetLayerCursorPosition(length);
      break;
    case IComposerClient::Command::SET_LAYER_BUFFER:
      parsed = parseSetLayerBuffer(length);
      break;
    case IComposerClient::Command::SET_LAYER_SURFACE_DAMAGE:
      parsed = parseSetLayerSurfaceDamage(length);
      break;
    case IComposerClient::Command::SET_LAYER_BLEND_MODE:
      parsed = parseSetLayerBlendMode(length);
      break;
    case IComposerClient::Command::SET_LAYER_COLOR:
      parsed = parseSetLayerColor(length);
      break;
    case IComposerClient::Command::SET_LAYER_COMPOSITION_TYPE:
      parsed = parseSetLayerCompositionType(length);
      break;
    case IComposerClient::Command::SET_LAYER_DATASPACE:
      parsed = parseSetLayerDataspace(length);
      break;
    case IComposerClient::Command::SET_LAYER_DISPLAY_FRAME:
      parsed = parseSetLayerDisplayFrame(length);
      break;
    case IComposerClient::Command::SET_LAYER_PLANE_ALPHA:
      parsed = parseSetLayerPlaneAlpha(length);
      break;
    case IComposerClient::Command::SET_LAYER_SIDEBAND_STREAM:
      parsed = parseSetLayerSidebandStream(length);
      break;
    case IComposerClient::Command::SET_LAYER_SOURCE_CROP:
      parsed = parseSetLayerSourceCrop(length);
      break;
    case IComposerClient::Command::SET_LAYER_TRANSFORM:
      parsed = parseSetLayerTransform(length);
      break;
    case IComposerClient::Command::SET_LAYER_VISIBLE_REGION:
      parsed = parseSetLayerVisibleRegion(length);
      break;
    case IComposerClient::Command::SET_LAYER_Z_ORDER:
      parsed = parseSetLayerZOrder(length);
      break;
    // Commands from ::android::hardware::graphics::composer::V2_2::IComposerClient follow.
    case IComposerClient::Command::SET_LAYER_PER_FRAME_METADATA:
      parsed = parseSetLayerPerFrameMetadata(length);
      break;
    case IComposerClient::Command::SET_LAYER_FLOAT_COLOR:
      parsed = parseSetLayerFloatColor(length);
      break;
    // Commands from ::android::hardware::graphics::composer::V2_3::IComposerClient follow.
    case IComposerClient::Command::SET_LAYER_COLOR_TRANSFORM:
      parsed = parseSetLayerColorTransform(length);
      break;
    case IComposerClient::Command::SET_LAYER_PER_FRAME_METADATA_BLOBS:
      parsed = parseSetLayerPerFrameMetadataBlobs(length);
      break;
    default:
      parsed = false;
      break;
  }

  return parsed;
}

Error QtiComposerClient::CommandReader::parse() {
  IComposerClient::Command command;
  uint16_t length;

  while (!isEmpty()) {
    if (!beginCommand(command, length)) {
      break;
    }

    bool parsed = false;
    switch (command) {
      default:
        parsed = parseCommonCmd(static_cast<IComposerClient::Command>(command), length);
        break;
    }

    endCommand();

    if (!parsed) {
      ALOGE("failed to parse command");
      break;
    }
  }

  return (isEmpty()) ? Error::NONE : Error::BAD_PARAMETER;
}

bool QtiComposerClient::CommandReader::parseSelectDisplay(uint16_t length) {
  if (length != CommandWriter::kSelectDisplayLength) {
    return false;
  }

  mDisplay = read64();
  mWriter.selectDisplay(mDisplay);

  return true;
}

bool QtiComposerClient::CommandReader::parseSelectLayer(uint16_t length) {
  if (length != CommandWriter::kSelectLayerLength) {
    return false;
  }

  mLayer = read64();

  return true;
}

bool QtiComposerClient::CommandReader::parseSetColorTransform(uint16_t length) {
  if (length != CommandWriter::kSetColorTransformLength) {
    return false;
  }
  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetClientTarget(uint16_t length) {
  // 4 parameters followed by N rectangles
  if ((length - 4) % 4 != 0) {
    return false;
  }

  bool useCache = false;
  auto slot = read();
  auto clientTarget = readHandle(useCache);
  shared_ptr<int32_t> fence = nullptr;
  readFence(&fence, "fbt");

  auto err = lookupBuffer(BufferCache::CLIENT_TARGETS, slot, useCache, clientTarget, &clientTarget);
  if (err == Error::NONE) {
    auto error = HWC2_ERROR_NONE;
    err = static_cast<Error>(error);
    auto updateBufErr = updateBuffer(BufferCache::CLIENT_TARGETS, slot, useCache, clientTarget);
    if (err == Error::NONE) {
      err = updateBufErr;
    }
  }
  if (err != Error::NONE) {
    mWriter.setError(getCommandLoc(), err);
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetOutputBuffer(uint16_t length) {
  if (length != CommandWriter::kSetOutputBufferLength) {
    return false;
  }

  bool useCache;
  auto slot = read();
  auto outputBuffer = readHandle(useCache);
  shared_ptr<int32_t> fence = nullptr;
  readFence(&fence, "outbuf");
  auto err = lookupBuffer(BufferCache::OUTPUT_BUFFERS, slot, useCache, outputBuffer, &outputBuffer);
  if (err == Error::NONE) {
    auto error = HWC2_ERROR_NONE;
    err = static_cast<Error>(error);
    auto updateBufErr = updateBuffer(BufferCache::OUTPUT_BUFFERS, slot, useCache, outputBuffer);
    if (err == Error::NONE) {
      err = updateBufErr;
    }
  }

  if (err != Error::NONE) {
    mWriter.setError(getCommandLoc(), err);
  }

  return true;
}

Error QtiComposerClient::CommandReader::validateDisplay(
    Display display, std::vector<Layer> &changedLayers,
    std::vector<IComposerClient::Composition> &compositionTypes, uint32_t &displayRequestMask,
    std::vector<Layer> &requestedLayers, std::vector<uint32_t> &requestMasks) {
  auto err = HWC2_ERROR_NONE;
  return static_cast<Error>(err);
}

bool QtiComposerClient::CommandReader::parseValidateDisplay(uint16_t length) {
  if (length != CommandWriter::kValidateDisplayLength) {
    return false;
  }

  std::vector<Layer> changedLayers;
  std::vector<IComposerClient::Composition> compositionTypes;
  uint32_t displayRequestMask;
  std::vector<Layer> requestedLayers;
  std::vector<uint32_t> requestMasks;

  auto err = validateDisplay(mDisplay, changedLayers, compositionTypes, displayRequestMask,
                             requestedLayers, requestMasks);

  if (static_cast<Error>(err) == Error::NONE) {
    mWriter.setChangedCompositionTypes(changedLayers, compositionTypes);
    mWriter.setDisplayRequests(displayRequestMask, requestedLayers, requestMasks);
  } else {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseAcceptDisplayChanges(uint16_t length) {
  if (length != CommandWriter::kAcceptDisplayChangesLength) {
    return false;
  }

  auto err = Error::NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

Error QtiComposerClient::CommandReader::presentDisplay(Display display,
                                                       std::vector<Layer> &layers) {
  int32_t err = HWC2_ERROR_NONE;
  return static_cast<Error>(err);
}

bool QtiComposerClient::CommandReader::parsePresentDisplay(uint16_t length) {
  if (length != CommandWriter::kPresentDisplayLength) {
    return false;
  }

  return true;
}

bool QtiComposerClient::CommandReader::parsePresentOrValidateDisplay(uint16_t length) {
  if (length != CommandWriter::kPresentOrValidateDisplayLength) {
    return false;
  }
  // First try to Present as is.
  mClient.getCapabilities();
  if (mClient.hasCapability(HWC2_CAPABILITY_SKIP_VALIDATE)) {
    std::vector<Layer> layers;
    auto err = Error::NONE;
    if (err == Error::NONE) {
      mWriter.setPresentOrValidateResult(1);
      return true;
    }
  }

  // Present has failed. We need to fallback to validate
  std::vector<Layer> changedLayers;
  std::vector<IComposerClient::Composition> compositionTypes;
  uint32_t displayRequestMask = 0x0;
  std::vector<Layer> requestedLayers;
  std::vector<uint32_t> requestMasks;

  auto err = validateDisplay(mDisplay, changedLayers, compositionTypes, displayRequestMask,
                             requestedLayers, requestMasks);
  if (err == Error::NONE) {
    mWriter.setPresentOrValidateResult(0);
    mWriter.setChangedCompositionTypes(changedLayers, compositionTypes);
    mWriter.setDisplayRequests(displayRequestMask, requestedLayers, requestMasks);
  } else {
    mWriter.setError(getCommandLoc(), err);
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerCursorPosition(uint16_t length) {
  if (length != CommandWriter::kSetLayerCursorPositionLength) {
    return false;
  }

  auto err = Error::NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerBuffer(uint16_t length) {
  if (length != CommandWriter::kSetLayerBufferLength) {
    return false;
  }

  bool useCache;
  auto slot = read();
  auto buffer = readHandle(useCache);
  shared_ptr<int32_t> fence = nullptr;
  readFence(&fence, "layer");
  auto error = lookupBuffer(BufferCache::LAYER_BUFFERS, slot, useCache, buffer, &buffer);
  if (error == Error::NONE) {
    auto err = HWC2_ERROR_NONE;
    error = static_cast<Error>(err);
    auto updateBufErr = updateBuffer(BufferCache::LAYER_BUFFERS, slot, useCache, buffer);
    if (static_cast<Error>(error) == Error::NONE) {
      error = updateBufErr;
    }
  }
  if (static_cast<Error>(error) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(error));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerSurfaceDamage(uint16_t length) {
  // N rectangles
  if (length % 4 != 0) {
    return false;
  }
  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }
  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerBlendMode(uint16_t length) {
  if (length != CommandWriter::kSetLayerBlendModeLength) {
    return false;
  }
  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerColor(uint16_t length) {
  if (length != CommandWriter::kSetLayerColorLength) {
    return false;
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerCompositionType(uint16_t length) {
  if (length != CommandWriter::kSetLayerCompositionTypeLength) {
    return false;
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerDataspace(uint16_t length) {
  if (length != CommandWriter::kSetLayerDataspaceLength) {
    return false;
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerDisplayFrame(uint16_t length) {
  if (length != CommandWriter::kSetLayerDisplayFrameLength) {
    return false;
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerPlaneAlpha(uint16_t length) {
  if (length != CommandWriter::kSetLayerPlaneAlphaLength) {
    return false;
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerSidebandStream(uint16_t length) {
  if (length != CommandWriter::kSetLayerSidebandStreamLength) {
    return false;
  }

  // Sideband stream is not supported
  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerSourceCrop(uint16_t length) {
  if (length != CommandWriter::kSetLayerSourceCropLength) {
    return false;
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerTransform(uint16_t length) {
  if (length != CommandWriter::kSetLayerTransformLength) {
    return false;
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }
  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerVisibleRegion(uint16_t length) {
  // N rectangles
  if (length % 4 != 0) {
    return false;
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerZOrder(uint16_t length) {
  if (length != CommandWriter::kSetLayerZOrderLength) {
    return false;
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerPerFrameMetadata(uint16_t length) {
  // (key, value) pairs
  if (length % 2 != 0) {
    return false;
  }

  std::vector<IComposerClient::PerFrameMetadata> metadata;
  metadata.reserve(length / 2);
  while (length > 0) {
    metadata.emplace_back(IComposerClient::PerFrameMetadata{
        static_cast<IComposerClient::PerFrameMetadataKey>(readSigned()), readFloat()});
    length -= 2;
  }

  std::vector<int32_t> keys;
  std::vector<float> values;
  keys.reserve(metadata.size());
  values.reserve(metadata.size());
  for (const auto &m : metadata) {
    keys.push_back(static_cast<int32_t>(m.key));
    values.push_back(m.value);
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerFloatColor(uint16_t length) {
  if (length != CommandWriter::kSetLayerFloatColorLength) {
    return false;
  }

  // setLayerFloatColor is not supported
  auto err = Error::UNSUPPORTED;
  mWriter.setError(getCommandLoc(), static_cast<Error>(err));

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerColorTransform(uint16_t length) {
  if (length != CommandWriter::kSetLayerColorTransformLength) {
    return false;
  }

  auto error = HWC2_ERROR_NONE;
  if (static_cast<Error>(error) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(error));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerPerFrameMetadataBlobs(uint16_t length) {
  // must have at least one metadata blob
  // of at least size 1 in queue (i.e {/numBlobs=/1, key, size, blob})
  if (length < 4) {
    return false;
  }

  uint32_t numBlobs = read();
  length--;
  std::vector<IComposerClient::PerFrameMetadataBlob> metadata;

  for (size_t i = 0; i < numBlobs; i++) {
    IComposerClient::PerFrameMetadataKey key =
        static_cast<IComposerClient::PerFrameMetadataKey>(readSigned());
    uint32_t blobSize = read();
    length -= 2;

    if (length * sizeof(uint32_t) < blobSize) {
      return false;
    }

    metadata.push_back({key, std::vector<uint8_t>()});
    IComposerClient::PerFrameMetadataBlob &metadataBlob = metadata.back();
    metadataBlob.blob.resize(blobSize);
    readBlob(blobSize, metadataBlob.blob.data());
  }

  std::vector<int32_t> keys;
  std::vector<uint32_t> sizes_of_metablob_;
  std::vector<uint8_t> blob_of_data_;
  keys.reserve(metadata.size());
  sizes_of_metablob_.reserve(metadata.size());
  for (const auto &m : metadata) {
    keys.push_back(static_cast<int32_t>(m.key));
    sizes_of_metablob_.push_back(m.blob.size());
    for (uint8_t i = 0; i < m.blob.size(); i++) {
      blob_of_data_.push_back(m.blob[i]);
    }
  }

  auto err = HWC2_ERROR_NONE;
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }
  return true;
}

hwc_rect_t QtiComposerClient::CommandReader::readRect() {
  return hwc_rect_t{
      readSigned(),
      readSigned(),
      readSigned(),
      readSigned(),
  };
}

std::vector<hwc_rect_t> QtiComposerClient::CommandReader::readRegion(size_t count) {
  std::vector<hwc_rect_t> region;
  region.reserve(count);
  while (count > 0) {
    region.emplace_back(readRect());
    count--;
  }

  return region;
}

hwc_frect_t QtiComposerClient::CommandReader::readFRect() {
  return hwc_frect_t{
      readFloat(),
      readFloat(),
      readFloat(),
      readFloat(),
  };
}

Error QtiComposerClient::CommandReader::lookupBufferCacheEntryLocked(BufferCache cache,
                                                                     uint32_t slot,
                                                                     BufferCacheEntry **outEntry) {
  auto dpy = mClient.mDisplayData.find(mDisplay);
  if (dpy == mClient.mDisplayData.end()) {
    return Error::BAD_DISPLAY;
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
      auto ly = dpy->second.Layers.find(mLayer);
      if (ly == dpy->second.Layers.end()) {
        return Error::BAD_LAYER;
      }
      if (slot < ly->second.Buffers.size()) {
        entry = &ly->second.Buffers[slot];
      }
    } break;
    case BufferCache::LAYER_SIDEBAND_STREAMS: {
      auto ly = dpy->second.Layers.find(mLayer);
      if (ly == dpy->second.Layers.end()) {
        return Error::BAD_LAYER;
      }
      if (slot == 0) {
        entry = &ly->second.SidebandStream;
      }
    } break;
    default:
      break;
  }

  if (!entry) {
    ALOGW("invalid buffer slot");
    return Error::BAD_PARAMETER;
  }

  *outEntry = entry;

  return Error::NONE;
}

Error QtiComposerClient::CommandReader::lookupBuffer(BufferCache cache, uint32_t slot,
                                                     bool useCache, buffer_handle_t handle,
                                                     buffer_handle_t *outHandle) {
  if (useCache) {
    std::lock_guard<std::mutex> lock(mClient.mDisplayDataMutex);

    BufferCacheEntry *entry;
    Error error = lookupBufferCacheEntryLocked(cache, slot, &entry);
    if (error != Error::NONE) {
      return error;
    }

    // input handle is ignored
    *outHandle = entry->getHandle();
  } else if (cache == BufferCache::LAYER_SIDEBAND_STREAMS) {
    if (handle) {
      *outHandle = native_handle_clone(handle);
      if (*outHandle == nullptr) {
        return Error::NO_RESOURCES;
      }
    }
  } else {
    if (!mHandleImporter.importBuffer(handle)) {
      return Error::NO_RESOURCES;
    }

    *outHandle = handle;
  }

  return Error::NONE;
}

Error QtiComposerClient::CommandReader::updateBuffer(BufferCache cache, uint32_t slot,
                                                     bool useCache, buffer_handle_t handle) {
  // handle was looked up from cache
  if (useCache) {
    return Error::NONE;
  }

  std::lock_guard<std::mutex> lock(mClient.mDisplayDataMutex);

  BufferCacheEntry *entry = nullptr;
  Error error = lookupBufferCacheEntryLocked(cache, slot, &entry);
  if (error != Error::NONE) {
    return error;
  }

  *entry = handle;
  return Error::NONE;
}

// Methods from ::android::hidl::base::V1_0::IBase follow.
implementation::QtiComposerClient *HIDL_FETCH_IQtiComposerClient(const char * /* name */) {
  return implementation::QtiComposerClient::CreateQtiComposerClientInstance();
}

}  // namespace implementation
}  // namespace V3_0
}  // namespace composer
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
