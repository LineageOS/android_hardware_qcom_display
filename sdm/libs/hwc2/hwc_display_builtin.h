/*
 * Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
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

#ifndef __HWC_DISPLAY_BUILTIN_H__
#define __HWC_DISPLAY_BUILTIN_H__

#include <string>
#include <vector>

#include "cpuhint.h"
#include "hwc_display.h"

namespace sdm {

class HWCDisplayBuiltIn : public HWCDisplay {
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
  virtual int Deinit();
  virtual HWC2::Error Validate(uint32_t *out_num_types, uint32_t *out_num_requests);
  virtual HWC2::Error Present(int32_t *out_retire_fence);
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
  virtual int HandleSecureSession(const std::bitset<kSecureMax> &secure_session,
                                  bool *power_on_pending);
  virtual DisplayError Refresh();
  virtual void SetIdleTimeoutMs(uint32_t timeout_ms);
  virtual HWC2::Error SetFrameDumpConfig(uint32_t count, uint32_t bit_mask_layer_type,
                                         int32_t format, bool post_processed);
  virtual int FrameCaptureAsync(const BufferInfo &output_buffer_info, bool post_processed);
  virtual int GetFrameCaptureStatus() { return frame_capture_status_; }
  virtual DisplayError SetDetailEnhancerConfig(const DisplayDetailEnhancerData &de_data);
  virtual DisplayError ControlPartialUpdate(bool enable, uint32_t *pending);
  virtual HWC2::Error SetReadbackBuffer(const native_handle_t *buffer, int32_t acquire_fence,
                                        bool post_processed_output);
  virtual HWC2::Error GetReadbackBufferFence(int32_t *release_fence);
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
  virtual DisplayError TeardownConcurrentWriteback(void);
  virtual void SetFastPathComposition(bool enable) {
    fast_path_composition_ = enable && !readback_buffer_queued_;
  }
  virtual HWC2::Error UpdatePowerMode(HWC2::PowerMode mode);
  virtual HWC2::Error PostCommitLayerStack(int32_t *out_retire_fence);
  virtual bool IsSmartPanelConfig(uint32_t config_id);
  virtual bool HasSmartPanelConfig(void);
  virtual HWC2::Error SetPowerMode(HWC2::PowerMode mode, bool teardown);

 private:
  HWCDisplayBuiltIn(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                    HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                    qService::QService *qservice, hwc2_display_t id, int32_t sdm_id);
  void SetMetaDataRefreshRateFlag(bool enable);
  virtual DisplayError SetDisplayMode(uint32_t mode);
  virtual DisplayError DisablePartialUpdateOneFrame();
  void SetQDCMSolidFillInfo(bool enable, const LayerSolidFill &color);
  void ToggleCPUHint(bool set);
  void ForceRefreshRate(uint32_t refresh_rate);
  uint32_t GetOptimalRefreshRate(bool one_updating_layer);
  void HandleFrameOutput();
  void HandleFrameDump();
  void HandleFrameCapture();
  bool CanSkipCommit();
  DisplayError SetMixerResolution(uint32_t width, uint32_t height);
  DisplayError GetMixerResolution(uint32_t *width, uint32_t *height);
  void SetPartialUpdate(DisplayConfigFixedInfo fixed_info);
  class PMICInterface {
   public:
    PMICInterface() { }
    ~PMICInterface() { }
    DisplayError Init();
    void Deinit();
    DisplayError Notify(SecureEvent event);

   private:
    int fd_lcd_bias_ = -1;
    int fd_wled_ = -1;
  };

  BufferAllocator *buffer_allocator_ = nullptr;
  CPUHint *cpu_hint_ = nullptr;

  // Builtin readback buffer configuration
  LayerBuffer output_buffer_ = {};
  bool post_processed_output_ = false;
  bool readback_buffer_queued_ = false;
  bool readback_configured_ = false;

  // Members for N frame output dump to file
  bool dump_output_to_file_ = false;
  BufferInfo output_buffer_info_ = {};
  void *output_buffer_base_ = nullptr;
  int default_mode_status_ = 0;
  bool pending_refresh_ = true;
  bool enable_optimize_refresh_ = false;
  bool hdr_present_ = false;

  // Members for 1 frame capture in a client provided buffer
  bool frame_capture_buffer_queued_ = false;
  int frame_capture_status_ = -EAGAIN;
  // PMIC interface to notify secure display start/end
  PMICInterface *pmic_intf_ = nullptr;
  bool pmic_notification_pending_ = false;
};

}  // namespace sdm

#endif  // __HWC_DISPLAY_BUILTIN_H__
