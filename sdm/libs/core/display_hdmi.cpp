/*
* Copyright (c) 2014 - 2018, 2020, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright notice, this list of
*      conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright notice, this list of
*      conditions and the following disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its contributors may be used to
*      endorse or promote products derived from this software without specific prior written
*      permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <utils/constants.h>
#include <utils/debug.h>
#include <map>
#include <utility>
#include <vector>

#include "display_hdmi.h"
#include "hw_interface.h"
#include "hw_info_interface.h"

#define __CLASS__ "DisplayHDMI"

namespace sdm {

static bool IsFormatOnlyYUV(HWDisplayAttributes attrib) {
  if (attrib.pixel_formats > DisplayInterfaceFormat::kFormatNone &&
      !(DisplayInterfaceFormat::kFormatRGB & attrib.pixel_formats)) {
    return true;
  }
  return false;
}

DisplayHDMI::DisplayHDMI(DisplayEventHandler *event_handler, HWInfoInterface *hw_info_intf,
                         BufferSyncHandler *buffer_sync_handler, BufferAllocator *buffer_allocator,
                         CompManager *comp_manager)
  : DisplayBase(kHDMI, event_handler, kDeviceHDMI, buffer_sync_handler, buffer_allocator,
                comp_manager, hw_info_intf) {
}

DisplayHDMI::DisplayHDMI(int32_t display_id, DisplayEventHandler *event_handler,
                              HWInfoInterface *hw_info_intf,
                              BufferSyncHandler *buffer_sync_handler,
                              BufferAllocator *buffer_allocator, CompManager *comp_manager)
  : DisplayBase(display_id, kHDMI, event_handler, kDeviceHDMI, buffer_sync_handler,
                buffer_allocator, comp_manager, hw_info_intf) {
}

DisplayError DisplayHDMI::Init() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  DisplayError error = HWInterface::Create(display_id_, kHDMI, hw_info_intf_, buffer_sync_handler_,
                                           buffer_allocator_, &hw_intf_);
  if (error != kErrorNone) {
    return error;
  }

  hw_intf_->GetDisplayId(&display_id_);

  uint32_t active_mode_index = 0;
  std::ifstream res_file;
  DisplayInterfaceFormat pref_format = kFormatNone;

  res_file.open("/vendor/resolutions.txt");
  if (res_file) {
    DLOGI("Getting best resolution from file");
    active_mode_index = GetBestConfigFromFile(res_file, &pref_format);
    res_file.close();
  } else {
    char value[64] = "0";
    DLOGI("Computing best resolution");
    Debug::GetProperty(HDMI_S3D_MODE_PROP, value);
    HWS3DMode mode = (HWS3DMode)atoi(value);
    if (mode > kS3DModeNone && mode < kS3DModeMax) {
      active_mode_index = GetBestConfig(mode);
    } else {
      active_mode_index = GetBestConfig(kS3DModeNone);
    }
  }

  error = hw_intf_->SetDisplayAttributes(active_mode_index);
  if (error != kErrorNone) {
    HWInterface::Destroy(hw_intf_);
    return error;
  }

  error = DisplayBase::Init();
  if (error != kErrorNone) {
    HWInterface::Destroy(hw_intf_);
    return error;
  }

  // If pref_format has some valid value other than kFormatNone, it
  // means GetBestConfigFromFile is used for best resolution selection.
  error = SetBestColorFormat(active_mode_index, pref_format);
  if (error != kErrorNone) {
    HWInterface::Destroy(hw_intf_);
    return error;
  }

  GetScanSupport();
  underscan_supported_ = (scan_support_ == kScanAlwaysUnderscanned) || (scan_support_ == kScanBoth);

  s3d_format_to_mode_.insert(std::pair<LayerBufferS3DFormat, HWS3DMode>
                            (kS3dFormatNone, kS3DModeNone));
  s3d_format_to_mode_.insert(std::pair<LayerBufferS3DFormat, HWS3DMode>
                            (kS3dFormatLeftRight, kS3DModeLR));
  s3d_format_to_mode_.insert(std::pair<LayerBufferS3DFormat, HWS3DMode>
                            (kS3dFormatRightLeft, kS3DModeRL));
  s3d_format_to_mode_.insert(std::pair<LayerBufferS3DFormat, HWS3DMode>
                            (kS3dFormatTopBottom, kS3DModeTB));
  s3d_format_to_mode_.insert(std::pair<LayerBufferS3DFormat, HWS3DMode>
                            (kS3dFormatFramePacking, kS3DModeFP));

  error = HWEventsInterface::Create(display_id_, kHDMI, this, event_list_, hw_intf_,
                                    &hw_events_intf_);
  if (error != kErrorNone) {
    DisplayBase::Deinit();
    HWInterface::Destroy(hw_intf_);
    DLOGE("Failed to create hardware events interface. Error = %d", error);
  }

  current_refresh_rate_ = hw_panel_info_.max_fps;

  return error;
}

DisplayError DisplayHDMI::Prepare(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  uint32_t new_mixer_width = 0;
  uint32_t new_mixer_height = 0;
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  if (NeedsMixerReconfiguration(layer_stack, &new_mixer_width, &new_mixer_height)) {
    error = ReconfigureMixer(new_mixer_width, new_mixer_height);
    if (error != kErrorNone) {
      ReconfigureMixer(display_width, display_height);
    }
  }

  SetS3DMode(layer_stack);

  // Clean hw layers for reuse.
  hw_layers_ = HWLayers();

  return DisplayBase::Prepare(layer_stack);
}

DisplayError DisplayHDMI::GetRefreshRateRange(uint32_t *min_refresh_rate,
                                              uint32_t *max_refresh_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  if (hw_panel_info_.min_fps && hw_panel_info_.max_fps) {
    *min_refresh_rate = hw_panel_info_.min_fps;
    *max_refresh_rate = hw_panel_info_.max_fps;
  } else {
    error = DisplayBase::GetRefreshRateRange(min_refresh_rate, max_refresh_rate);
  }

  return error;
}

DisplayError DisplayHDMI::SetRefreshRate(uint32_t refresh_rate, bool final_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!active_) {
    return kErrorPermission;
  }

  if (current_refresh_rate_ != refresh_rate) {
    DisplayError error = hw_intf_->SetRefreshRate(refresh_rate);
    if (error != kErrorNone) {
      return error;
    }
  }

  current_refresh_rate_ = refresh_rate;
  return DisplayBase::ReconfigureDisplay();
}

bool DisplayHDMI::IsUnderscanSupported() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return underscan_supported_;
}

DisplayError DisplayHDMI::OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->OnMinHdcpEncryptionLevelChange(min_enc_level);
}

uint32_t DisplayHDMI::GetBestConfigFromFile(std::ifstream &res_file, DisplayInterfaceFormat *format) {
  string line;
  uint32_t num_modes = 0;
  uint32_t index = 0;
  std::map<std::string, DisplayInterfaceFormat> intf_format_to_str;
  intf_format_to_str[std::string("rgb")] = DisplayInterfaceFormat::kFormatRGB;
  intf_format_to_str[std::string("yuv422")] = DisplayInterfaceFormat::kFormatYCbCr422;
  intf_format_to_str[std::string("yuv422d")] = DisplayInterfaceFormat::kFormatYCbCr422d;
  intf_format_to_str[std::string("yuv420")] = DisplayInterfaceFormat::kFormatYCbCr420;
  intf_format_to_str[std::string("yuv420d")] = DisplayInterfaceFormat::kFormatYCbCr420d;
  intf_format_to_str[std::string("yuv444")] = DisplayInterfaceFormat::kFormatYCbCr444;
  hw_intf_->GetNumDisplayAttributes(&num_modes);
  DLOGI("Num modes = %d",num_modes);
  // Get display attribute for each mode
  std::vector<HWDisplayAttributes> attrib(num_modes);
  std::vector<uint32_t> vics(num_modes);
  for (index = 0; index < num_modes; index++) {
    hw_intf_->GetDisplayAttributes(index, &attrib[index]);
    vics[index] = attrib[index].vic;
  }
  try {
    char cr = '\r';
    while (std::getline(res_file, line, cr)) {
      char hash = '#';
      std::size_t found = 0;
      found = line.find(hash);
      if (found != std::string::npos) {
        // # is found, ignore this line
        DLOGI("Hash found");
        continue;
      }
      char colon = ':';
      found = line.find(colon);
      if (found != std::string::npos) {
        DLOGI("Colon found at %d",found);
        std::string vic_str = line.substr(0, found);
        int vic = std::stoi(vic_str);
        if (vic > standard_vic_) {
          DLOGE("Invalid svd %d",vic);
          continue;
        }
        std::string fmt_str = line.substr(found+1, line.size());
        std::map<std::string, DisplayInterfaceFormat>::iterator fmt_str_it = intf_format_to_str.find(fmt_str);
        if (fmt_str_it == intf_format_to_str.end()) {
          DLOGE("Invalid color token  %s",fmt_str.c_str());
          continue;
        }
        DisplayInterfaceFormat fmt = fmt_str_it->second;
        DLOGI("Preferred format = %d", fmt);
        std::vector<uint32_t>::iterator vic_itr = std::find(vics.begin(), vics.end(), vic);
        if (vic_itr != vics.end())
        {
          uint32_t index = static_cast<uint32_t>(vic_itr - vics.begin());
          DLOGI("Display supports vic %d!.. index = %d",vic, index);
          if (fmt == DisplayInterfaceFormat::kFormatRGB) {
            if (attrib[index].pixel_formats & DisplayInterfaceFormat::kFormatRGB) {
              *format = DisplayInterfaceFormat::kFormatRGB;
              return index;
            } else {
              DLOGI("RGB not supported by Display attributes[%d]",index);
            }
          } else if (fmt == DisplayInterfaceFormat::kFormatYCbCr422 ||
                     fmt == DisplayInterfaceFormat::kFormatYCbCr422d) {
            if(attrib[index].pixel_formats & DisplayInterfaceFormat::kFormatYCbCr422) {
              *format = DisplayInterfaceFormat::kFormatYCbCr422;
              return index;
            } else {
              DLOGI("YUV422 not supported by Display attributes[%d]",index);
            }
          } else if(fmt == DisplayInterfaceFormat::kFormatYCbCr420 ||
                    fmt == DisplayInterfaceFormat::kFormatYCbCr420d) {
            if(attrib[index].pixel_formats & DisplayInterfaceFormat::kFormatYCbCr420) {
              *format = DisplayInterfaceFormat::kFormatYCbCr420;
              return index;
            } else {
              DLOGI("YUV420 not supported by Display attributes[%d]", index);
            }
          } else {
            DLOGI("Invalid format %d",fmt);
          }
        } else {
          DLOGI("Display does not support vic %d ",vic);
        }
      } else {
        DLOGE("Delimiter : not found");
      }
    }
  } catch (const std::invalid_argument& ia) {
    DLOGE("Invalid argument exception %s",ia.what());
    return 0;
  } catch (const std::exception& e) {
    DLOGE("Exception occurred %s",e.what());
    return 0;
  } catch(...) {
      DLOGE("Exception occurred ");
      return 0;
  }
  // None of the resolutions are supported by TV.
  const int default_vic = 2;   // Default to 480p RGB.
  DisplayInterfaceFormat def_fmt = DisplayInterfaceFormat::kFormatRGB;
  std::vector<uint32_t>::iterator def_vic_itr = std::find(vics.begin(), vics.end(), default_vic);
  if (def_vic_itr != vics.end())
  {
    uint32_t def_index = static_cast<uint32_t>(def_vic_itr - vics.begin());
    *format = def_fmt;
    return def_index;
  } else {
    // Even 480p is not supported.
    DLOGE("480p is not supported!");
    return 0;
  }
  return 0;
}

uint32_t DisplayHDMI::GetBestConfig(HWS3DMode s3d_mode) {
  uint32_t best_index = 0, index;
  uint32_t num_modes = 0;

  hw_intf_->GetNumDisplayAttributes(&num_modes);

  // Get display attribute for each mode
  std::vector<HWDisplayAttributes> attrib(num_modes);
  for (index = 0; index < num_modes; index++) {
    hw_intf_->GetDisplayAttributes(index, &attrib[index]);
  }

  // Select best config for s3d_mode. If s3d is not enabled, s3d_mode is kS3DModeNone
  for (index = 0; index < num_modes; index ++) {
    if (attrib[index].s3d_config[s3d_mode]) {
      break;
    }
  }

  index = 0;
  best_index = UINT32(index);
  for (size_t index = best_index + 1; index < num_modes; index ++) {

    uint32_t best_clock_khz = IsFormatOnlyYUV(attrib[best_index]) ?
             attrib[best_index].clock_khz/2 : attrib[best_index].clock_khz;
    uint32_t current_clock_khz = IsFormatOnlyYUV(attrib[index]) ?
             attrib[index].clock_khz/2 : attrib[index].clock_khz;
    if (current_clock_khz > best_clock_khz) {
      DLOGI("Best index = %d .Best pixel clock = %d .Previous best was %d",
            index, current_clock_khz, best_clock_khz);
      best_index = UINT32(index);
    } else if (current_clock_khz == best_clock_khz) {
      DLOGI("Same pix clock. clock = %d . v1 = %d.. v2 = %d",
      current_clock_khz, attrib[best_index].vic, attrib[index].vic);
      if ((attrib[index].vic > standard_vic_ &&
          attrib[best_index].vic <= standard_vic_)) {
        // we should not select the non-standard vic-id.
        DLOGI("Standard vic already selected");
        continue;
      } else if((attrib[index].vic <= standard_vic_ &&
                attrib[best_index].vic > standard_vic_)) {
        // select the standard vic-id
        best_index = UINT32(index);
        DLOGI("Selecting Standard vic now. Best index = %d", best_index);
        continue;
      }
      if (attrib[index].x_pixels > attrib[best_index].x_pixels) {
        DLOGI("Best index = %d .Best xpixel  = %d .Previous best was %d",
              index,attrib[index].x_pixels,attrib[best_index].x_pixels);
        best_index = UINT32(index);
      } else if (attrib[index].x_pixels == attrib[best_index].x_pixels) {
        if (attrib[index].y_pixels > attrib[best_index].y_pixels) {
          DLOGI("Best index = %d .Best ypixel  = %d .Previous best was %d",
                index, attrib[index].y_pixels, attrib[best_index].y_pixels);
          best_index = UINT32(index);
        } else if (attrib[index].y_pixels == attrib[best_index].y_pixels) {
          if (attrib[index].vsync_period_ns < attrib[best_index].vsync_period_ns) {
            DLOGI("Best index = %d .Best vsync_period = %d .Previous best was %d",
                  index, attrib[index].vsync_period_ns,
                  attrib[best_index].vsync_period_ns);
            best_index = UINT32(index);
          }
        }
      }
    }
  }
  char val[kPropertyMax]={};
  // Used for changing HDMI Resolution - override the best with user set config
  bool user_config = (Debug::GetExternalResolution(val));

  if (user_config) {
    uint32_t config_index = 0;
    // For the config, get the corresponding index
    DisplayError error = hw_intf_->GetConfigIndex(val, &config_index);
    if (error == kErrorNone)
      best_index = config_index;
  }
  return best_index;
}

DisplayError DisplayHDMI::SetBestColorFormat(uint32_t best_index,
                                             DisplayInterfaceFormat pref_format) {
  uint32_t num_modes = 0;
  DisplayError error = kErrorNone;
  HWDisplayAttributes best_attrib;

  hw_intf_->GetNumDisplayAttributes(&num_modes);

  // Get display attribute for best config
  if (best_index >= num_modes) {
    DLOGE("Invalid mode index %d mode size %d", best_index, num_modes);
    return kErrorParameters;
  }

  hw_intf_->GetDisplayAttributes(best_index, &best_attrib);

  if (pref_format != DisplayInterfaceFormat::kFormatNone) {
    error = hw_intf_->SetDisplayFormat(best_index, pref_format);
    if (error == kErrorNone) {
      DLOGI("Preferred format(%d) from file is supported by Display attributes[%d]",
            pref_format, best_index);
    }
  } else {
    if (hw_panel_info_.hdr_enabled) {
      DLOGI("HDR Mode is ON. index: %d", best_index);
      if (best_attrib.pixel_formats & DisplayInterfaceFormat::kFormatYCbCr422) {
        error = hw_intf_->SetDisplayFormat(best_index,
                                           DisplayInterfaceFormat::kFormatYCbCr422);
        if (error == kErrorNone) {
          DLOGI("YUV422 is supported by Display attributes[%d]", best_index);
        }
      } else if (best_attrib.pixel_formats & DisplayInterfaceFormat::kFormatYCbCr420) {
        error = hw_intf_->SetDisplayFormat(best_index,
                                           DisplayInterfaceFormat::kFormatYCbCr420);
        if (error == kErrorNone) {
          DLOGI("YUV420 is supported by Display attributes[%d]", best_index);
        }
      } else if (best_attrib.pixel_formats & DisplayInterfaceFormat::kFormatRGB) {
        error = hw_intf_->SetDisplayFormat(best_index,
                                           DisplayInterfaceFormat::kFormatRGB);
        if (error == kErrorNone) {
          DLOGI("RGB is supported by Display attributes[%d]", best_index);
        }
      } else {
        DLOGE("No format supported with HDR enabled[%d]", best_index);
      }
    } else {
      DLOGI("HDR Mode is OFF. index: %d", best_index);
      if (best_attrib.pixel_formats & DisplayInterfaceFormat::kFormatRGB) {
        error = hw_intf_->SetDisplayFormat(best_index,
                                           DisplayInterfaceFormat::kFormatRGB);
        if (error == kErrorNone) {
          DLOGI("RGB is supported by Display attributes[%d]", best_index);
        }
      } else if (best_attrib.pixel_formats & DisplayInterfaceFormat::kFormatYCbCr420) {
        error = hw_intf_->SetDisplayFormat(best_index,
                                           DisplayInterfaceFormat::kFormatYCbCr420);
        if (error == kErrorNone) {
          DLOGI("YUV420 is supported by Display attributes[%d]", best_index);
        }
      } else if (best_attrib.pixel_formats & DisplayInterfaceFormat::kFormatYCbCr422) {
        error = hw_intf_->SetDisplayFormat(best_index,
                                           DisplayInterfaceFormat::kFormatYCbCr422);
        if (error == kErrorNone) {
          DLOGI("YUV422 is supported by Display attributes[%d]", best_index);
        }
      } else {
        DLOGE("No format supported with HDR disabled[%d]", best_index);
      }
    }
  }
  return error;
}

void DisplayHDMI::GetScanSupport() {
  DisplayError error = kErrorNone;
  uint32_t video_format = 0;
  uint32_t max_cea_format = 0;
  HWScanInfo scan_info = HWScanInfo();
  hw_intf_->GetHWScanInfo(&scan_info);

  uint32_t active_mode_index = 0;
  hw_intf_->GetActiveConfig(&active_mode_index);

  error = hw_intf_->GetVideoFormat(active_mode_index, &video_format);
  if (error != kErrorNone) {
    return;
  }

  error = hw_intf_->GetMaxCEAFormat(&max_cea_format);
  if (error != kErrorNone) {
    return;
  }

  // The scan support for a given HDMI TV must be read from scan info corresponding to
  // Preferred Timing if the preferred timing of the display is currently active, and if it is
  // valid. In all other cases, we must read the scan support from CEA scan info if
  // the resolution is a CEA resolution, or from IT scan info for all other resolutions.
  if (active_mode_index == 0 && scan_info.pt_scan_support != kScanNotSupported) {
    scan_support_ = scan_info.pt_scan_support;
  } else if (video_format < max_cea_format) {
    scan_support_ = scan_info.cea_scan_support;
  } else {
    scan_support_ = scan_info.it_scan_support;
  }
}

void DisplayHDMI::SetS3DMode(LayerStack *layer_stack) {
  uint32_t s3d_layer_count = 0;
  HWS3DMode s3d_mode = kS3DModeNone;
  uint32_t layer_count = UINT32(layer_stack->layers.size());

  // S3D mode is supported for the following scenarios:
  // 1. Layer stack containing only one s3d layer which is not skip
  // 2. Layer stack containing only one secure layer along with one s3d layer
  for (uint32_t i = 0; i < layer_count; i++) {
    Layer *layer = layer_stack->layers.at(i);
    LayerBuffer &layer_buffer = layer->input_buffer;

    if (layer_buffer.s3d_format != kS3dFormatNone) {
      s3d_layer_count++;
      if (s3d_layer_count > 1 || layer->flags.skip) {
        s3d_mode = kS3DModeNone;
        break;
      }

      std::map<LayerBufferS3DFormat, HWS3DMode>::iterator it =
                s3d_format_to_mode_.find(layer_buffer.s3d_format);
      if (it != s3d_format_to_mode_.end()) {
        s3d_mode = it->second;
      }
    } else if (layer_buffer.flags.secure && layer_count > 2) {
        s3d_mode = kS3DModeNone;
        break;
    }
  }

  if (hw_intf_->SetS3DMode(s3d_mode) != kErrorNone) {
    hw_intf_->SetS3DMode(kS3DModeNone);
    layer_stack->flags.s3d_mode_present = false;
  } else if (s3d_mode != kS3DModeNone) {
    layer_stack->flags.s3d_mode_present = true;
  }

  DisplayBase::ReconfigureDisplay();
}

void DisplayHDMI::CECMessage(char *message) {
  event_handler_->CECMessage(message);
}

DisplayError DisplayHDMI::VSync(int64_t timestamp) {
  if (vsync_enable_) {
    DisplayEventVSync vsync;
    vsync.timestamp = timestamp;
    event_handler_->VSync(vsync);
  }

  return kErrorNone;
}

}  // namespace sdm

