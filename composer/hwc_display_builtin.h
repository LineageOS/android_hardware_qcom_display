/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

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

/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __HWC_DISPLAY_BUILTIN_H__
#define __HWC_DISPLAY_BUILTIN_H__

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "utils/sync_task.h"
#include "utils/constants.h"
#include "cpuhint.h"
#include "hwc_display.h"
#include "hwc_layers.h"

#include "gl_layer_stitch.h"

namespace sdm {

enum class LayerStitchTaskCode : int32_t {
  kCodeGetInstance,
  kCodeStitch,
  kCodeDestroyInstance,
};

struct LayerStitchGetInstanceContext : public SyncTask<LayerStitchTaskCode>::TaskContext {
  LayerBuffer *output_buffer = NULL;
};

struct LayerStitchContext : public SyncTask<LayerStitchTaskCode>::TaskContext {
  vector<StitchParams> stitch_params;
  shared_ptr<Fence> src_acquire_fence = nullptr;
  shared_ptr<Fence> dst_acquire_fence = nullptr;
  shared_ptr<Fence> release_fence = nullptr;
};

class HWCDisplayBuiltIn : public HWCDisplay, public SyncTask<LayerStitchTaskCode>::TaskHandler {
 public:
  enum {
    SET_METADATA_DYN_REFRESH_RATE,
    SET_BINDER_DYN_REFRESH_RATE,
    SET_DISPLAY_MODE,
    SET_QDCM_SOLID_FILL_INFO,
    UNSET_QDCM_SOLID_FILL_INFO,
    SET_QDCM_SOLID_FILL_RECT,
  };

  static int Create(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                    HWCCallbacks *callbacks,  HWCDisplayEventHandler *event_handler,
                    qService::QService *qservice, hwc2_display_t id, int32_t sdm_id,
                    HWCDisplay **hwc_display);
  static void Destroy(HWCDisplay *hwc_display);
  virtual int Init();
  virtual HWC2::Error Present(shared_ptr<Fence> *out_retire_fence);
  virtual HWC2::Error CommitLayerStack();
  virtual HWC2::Error GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes);
  virtual HWC2::Error SetColorMode(ColorMode mode);
  virtual HWC2::Error GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                                       RenderIntent *out_intents);
  virtual HWC2::Error SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent);
  virtual HWC2::Error SetColorModeById(int32_t color_mode_id);
  virtual HWC2::Error SetColorModeFromClientApi(int32_t color_mode_id);
  virtual HWC2::Error SetColorTransform(const float *matrix, android_color_transform_t hint);
  virtual HWC2::Error RestoreColorTransform();
  virtual int Perform(uint32_t operation, ...);
  virtual int GetActiveSecureSession(std::bitset<kSecureMax> *secure_sessions);
  virtual int HandleSecureSession(const std::bitset<kSecureMax> &secure_session,
                                  bool *power_on_pending, bool is_active_secure_display);
  virtual void SetIdleTimeoutMs(uint32_t timeout_ms, uint32_t inactive_ms);
  virtual int FrameCaptureAsync(const BufferInfo &output_buffer_info, const CwbConfig &cwb_config);
  virtual int GetFrameCaptureStatus() { return frame_capture_status_; }
  virtual DisplayError SetDetailEnhancerConfig(const DisplayDetailEnhancerData &de_data);
  virtual DisplayError SetHWDetailedEnhancerConfig(void *params);
  virtual DisplayError ControlPartialUpdate(bool enable, uint32_t *pending);
  virtual HWC2::Error SetQSyncMode(QSyncMode qsync_mode);
  virtual DisplayError ControlIdlePowerCollapse(bool enable, bool synchronous);
  virtual HWC2::Error SetDisplayDppsAdROI(uint32_t h_start, uint32_t h_end, uint32_t v_start,
                                          uint32_t v_end, uint32_t factor_in, uint32_t factor_out);
  virtual DisplayError SetDynamicDSIClock(uint64_t bitclk);
  virtual DisplayError GetDynamicDSIClock(uint64_t *bitclk);
  virtual DisplayError GetSupportedDSIClock(std::vector<uint64_t> *bitclk_rates);
  virtual HWC2::Error UpdateDisplayId(hwc2_display_t id);
  virtual HWC2::Error SetPendingRefresh();
  virtual HWC2::Error SetPanelBrightness(float brightness);
  virtual HWC2::Error GetPanelBrightness(float *brightness);
  virtual HWC2::Error GetPanelMaxBrightness(uint32_t *max_brightness_level);
  virtual HWC2::Error SetFrameTriggerMode(uint32_t mode);
  virtual HWC2::Error SetBLScale(uint32_t level);
  virtual HWC2::Error SetClientTarget(buffer_handle_t target, shared_ptr<Fence> acquire_fence,
                                      int32_t dataspace, hwc_region_t damage);
  virtual bool IsSmartPanelConfig(uint32_t config_id);
  virtual bool HasSmartPanelConfig(void);
  virtual int Deinit();
  virtual int PostInit();

  virtual HWC2::Error SetDisplayedContentSamplingEnabledVndService(bool enabled);
  virtual HWC2::Error SetDisplayedContentSamplingEnabled(int32_t enabled, uint8_t component_mask,
                                                         uint64_t max_frames);
  virtual HWC2::Error GetDisplayedContentSamplingAttributes(int32_t *format, int32_t *dataspace,
                                                            uint8_t *supported_components);
  virtual HWC2::Error GetDisplayedContentSample(
      uint64_t max_frames, uint64_t timestamp, uint64_t *numFrames,
      int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
      uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS]);
  void Dump(std::ostringstream *os) override;
  virtual HWC2::Error SetPowerMode(HWC2::PowerMode mode, bool teardown);
  virtual bool IsDisplayIdle();
  virtual bool HasReadBackBufferSupport();
  virtual HWC2::Error NotifyDisplayCalibrationMode(bool in_calibration);
  virtual HWC2::Error CommitOrPrepare(bool validate_only, shared_ptr<Fence> *out_retire_fence,
                                      uint32_t *out_num_types, uint32_t *out_num_requests,
                                      bool *needs_commit);
  virtual HWC2::Error PreValidateDisplay(bool *exit_validate);
  virtual HWC2::Error PostCommitLayerStack(shared_ptr<Fence> *out_retire_fence);
  virtual HWC2::Error SetAlternateDisplayConfig(bool set);
  virtual HWC2::Error SetDimmingEnable(int int_enabled);
  virtual HWC2::Error SetDimmingMinBl(int min_bl);

 private:
  HWCDisplayBuiltIn(CoreInterface *core_intf, HWCBufferAllocator *buffer_allocator,
                    HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                    qService::QService *qservice, hwc2_display_t id, int32_t sdm_id);
  void SetMetaDataRefreshRateFlag(bool enable);
  virtual DisplayError SetDisplayMode(uint32_t mode);
  virtual DisplayError DisablePartialUpdateOneFrame();
  void ProcessBootAnimCompleted(void);
  void SetQDCMSolidFillInfo(bool enable, const LayerSolidFill &color);
  void ForceRefreshRate(uint32_t refresh_rate);
  uint32_t GetOptimalRefreshRate(bool one_updating_layer);
  virtual void HandleFrameCapture();
  bool CanSkipCommit();
  DisplayError SetMixerResolution(uint32_t width, uint32_t height);
  DisplayError GetMixerResolution(uint32_t *width, uint32_t *height);
  HWC2::Error CommitStitchLayers();
  void AppendStitchLayer();
  bool InitLayerStitch();
  void InitStitchTarget();
  bool AllocateStitchBuffer();
  void PostCommitStitchLayers();
  bool NeedsLargeCompPerfHint();
  void ValidateUiScaling();
  void EnablePartialUpdate();
  uint32_t GetUpdatingAppLayersCount();
  void LoadMixedModePerfHintThreshold();
  void HandleLargeCompositionHint(bool release);
  void ReqPerfHintRelease();

  // SyncTask methods.
  void OnTask(const LayerStitchTaskCode &task_code,
              SyncTask<LayerStitchTaskCode>::TaskContext *task_context);

  const int kPerfHintLargeCompCycle = 0x00001097;
  const int kPerfHintDisplayOff = 0x00001040;
  const int kPerfHintDisplayOn = 0x00001041;
  const int kPerfHintDisplayDoze = 0x00001053;
  HWCBufferAllocator *buffer_allocator_ = nullptr;
  CPUHint *cpu_hint_ = nullptr;

  bool pending_refresh_ = true;
  bool enable_optimize_refresh_ = false;
  bool enable_poms_during_doze_ = false;

  bool is_primary_ = false;
  bool disable_layer_stitch_ = true;
  HWCLayer* stitch_target_ = nullptr;
  SyncTask<LayerStitchTaskCode> layer_stitch_task_;
  GLLayerStitch* gl_layer_stitch_ = nullptr;
  BufferInfo buffer_info_ = {};
  DisplayConfigVariableInfo fb_config_ = {};

  bool qsync_enabled_ = false;
  bool qsync_reconfigured_ = false;
  // Members for Color sampling feature
  DisplayError HistogramEvent(int fd, uint32_t blob_id) override;
  histogram::HistogramCollector histogram;
  std::mutex sampling_mutex;
  bool api_sampling_vote = false;
  bool vndservice_sampling_vote = false;

  int perf_hint_large_comp_cycle_ = 0;
  bool force_reset_lut_ = false;
  bool disable_dyn_fps_ = false;
  bool enable_round_corner_ = false;
  bool enhance_idle_time_ = false;
  shared_ptr<Fence> retire_fence_ = nullptr;
  std::unordered_map<int32_t, int32_t> mixed_mode_threshold_;
  int alternate_config_ = -1;

  // Long term large composition hint
  int hwc_tid_ = 0;
  uint32_t num_basic_frames_ = 0;
};

}  // namespace sdm

#endif  // __HWC_DISPLAY_BUILTIN_H__
