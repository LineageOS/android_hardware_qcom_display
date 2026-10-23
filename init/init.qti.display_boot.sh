#!/vendor/bin/sh
# Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
# Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#     * Neither the name of The Linux Foundation nor the names of its
#       contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
# Changes from Qualcomm Technologies, Inc. are provided under the following license:
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear


target=`getprop ro.board.platform`
if [ -f /sys/devices/soc0/soc_id ]; then
    soc_hwid=`cat /sys/devices/soc0/soc_id`
else
    soc_hwid=`cat /sys/devices/system/soc/soc0/id`
fi

case "$target" in
    "pineapple")
    #SOC ID for Pineapple is 557
    case "$soc_hwid" in
      557)
        setprop vendor.display.enable_fb_scaling 0
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.target.version 4
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 1
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.enable_latch_media_content 1
        setprop vendor.display.enable_inline_writeback 1
        setprop debug.sf.enable_hwc_vds 0
        setprop vendor.display.timed_render_enable 1
        setprop vendor.gralloc.hw_supports_ubwcp 1
        ;;
    esac
    ;;
    "kalama")
    #SOC ID for Kalama is 519
    case "$soc_hwid" in
      519)
        setprop vendor.display.enable_fb_scaling 0
        setprop vendor.display.target.version 4
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 0
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.enable_latch_media_content 1
        setprop vendor.display.enable_inline_writeback 1
        setprop vendor.display.timed_render_enable 1
        setprop debug.sf.disable_client_composition_cache 0
        setprop vendor.gralloc.hw_supports_ubwcp 0
        ;;
    esac
    ;;
    "taro")
    #Set property to differentiate Waipio
    #SOC ID for Waipio is 457
    #SOC ID for Cape MSM is 530
    #SOC ID for Cape APQ is 531
    #SOC ID for Cape 4g is 540
    case "$soc_hwid" in
        457)
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 0
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.target.version 3
        setprop vendor.display.enable_fb_scaling 0
        setprop vendor.display.disable_cwb_idle_fallback 1
        ;;
        530|531|540)
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 0
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.target.version 2
        setprop vendor.display.enable_qsync_idle 1
        setprop vendor.display.disable_cwb_idle_fallback 1
        ;;
        506|547)
        # Set property for Diwali
        # SOC ID for Diwali is 506
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 0
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.target.version 2
        setprop vendor.display.enable_qsync_idle 1
        ;;
    esac
    ;;
    "lahaina")
    #Set property to differentiate Lahaina & Shima
    #SOC ID for Lahaina is 415, Lahaina P is 439, Lahaina-ATP is 456
    case "$soc_hwid" in
        415|439|456)
        # Set property for lahaina
        setprop vendor.display.target.version 1
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_allow_idle_fallback 1
        ;;
        450)
        # Set property for shima
        setprop vendor.display.target.version 2
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_posted_start_dyn 1
        setprop vendor.display.enable_qsync_idle 1
        setprop vendor.display.enable_allow_idle_fallback 1
        ;;
        475)
        # Set property for Yupik
        setprop vendor.display.enable_posted_start_dyn 2
        ;;
    esac
    ;;
    "holi"|"blair")
    #Set property to differentiate Holi & Blair
    #SOC ID for Holi is 454 and for Blair is 507
    case "$soc_hwid" in
        454)
        # Set property for holi
        setprop vendor.display.target.version 3
        setprop vendor.display.disable_offline_rotator 0
        setprop vendor.display.disable_rotator_ubwc 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_posted_start_dyn 1
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.enable_async_powermode 0
        setprop vendor.display.enable_spec_fence 0
        setprop vendor.display.enable_inline_writeback 0
        setprop vendor.display.disable_cwb_idle_fallback 1
        setprop vendor.display.disable_gpu_color_convert 0
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.disable_get_screen_decorator_support 1
        setprop vendor.gralloc.secure_preview_only 1
        setprop vendor.display.perf.version 2
        ;;
        507|565|578)
        # Set property for blair
        # SOC ID for blair APQ is 565
        # SOC Id for Blair Lite is 578
        setprop vendor.display.target.version 3
        setprop vendor.display.disable_offline_rotator 0
        setprop vendor.display.disable_rotator_ubwc 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_posted_start_dyn 1
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.enable_async_powermode 0
        setprop vendor.display.enable_spec_fence 0
        setprop vendor.display.enable_inline_writeback 0
        setprop vendor.display.disable_cwb_idle_fallback 1
        setprop vendor.display.disable_gpu_color_convert 0
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.disable_get_screen_decorator_support 1
        setprop vendor.gralloc.secure_preview_only 1
        setprop vendor.display.perf.version 2
        ;;
    esac
    ;;
    "parrot")
    case "$soc_hwid" in
        537 | 583 | 613 | 631 | 633 | 634 | 638 | 663 | 713 | 714 | 715)
        # Set property for Parrot
        # SOC ID for Gaming SKU with Modem is 633
        # SOC ID for Gaming SKU without Modem is 634
        # SOC ID for SM7435 P is 638
        # SOC ID for Parrot PRO is 663
        # SOC ID for Parrot Lite id 715
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 1
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.target.version 2
        setprop vendor.display.enable_qsync_idle 1
        setprop vendor.display.disable_mitigated_fps 1
        setprop vendor.display.secure_preview_buffer_format 420_sp
        setprop vendor.gralloc.secure_preview_buffer_format 420_sp
        setprop vendor.display.disable_non_wfd_vds 1
        setprop vendor.display.supports_background_blur 0
	setprop vendor.gralloc.hw_supports_ubwcp 0
        setprop vendor.gralloc.enable_snapalloc 1
        setprop vendor.display.disable_get_screen_decorator_support 1
        if [ "$soc_hwid" -eq 633 ] || [ "$soc_hwid" -eq 634 ]; then
            setprop vendor.display.enable_latch_media_content 1
        fi
        ;;
        568|602|653|654|581|582)
        # Set property for Ravelin
        # SOC ID for Ravelin is 568
        # SOC ID for Ravelin APQ is 602
        # SOC ID for SG_RAVELIN is 653
        # SOC ID for SG_RAVELIN is 653
        # SOC ID for Ravelin_iot is 581
        # SOC ID for Ravelin_iot is 582
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 1
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.target.version 2
        setprop vendor.display.enable_qsync_idle 1
        setprop vendor.display.disable_mitigated_fps 1
        setprop vendor.display.secure_preview_buffer_format 420_sp
        setprop vendor.gralloc.secure_preview_buffer_format 420_sp
        setprop vendor.display.disable_cwb_idle_fallback 1
        setprop vendor.display.enable_rotator_concurrency 1
        setprop vendor.display.disable_offline_rotator 0
        setprop vendor.display.disable_rotator_ubwc 1
        setprop vendor.display.supports_background_blur 0
	setprop vendor.gralloc.hw_supports_ubwcp 0
        setprop vendor.gralloc.enable_snapalloc 1
        setprop vendor.display.disable_get_screen_decorator_support 1
        if [ "$soc_hwid" -eq 653 ] || [ "$soc_hwid" -eq 654 ]; then
            setprop vendor.display.enable_latch_media_content 1
        fi
        ;;
	esac
esac
