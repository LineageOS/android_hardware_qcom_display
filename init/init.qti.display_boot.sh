#!/vendor/bin/sh
# Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

target=`getprop ro.board.platform`
if [ -f /sys/devices/soc0/soc_id ]; then
    soc_hwid=`cat /sys/devices/soc0/soc_id`
else
    soc_hwid=`cat /sys/devices/system/soc/soc0/id`
fi

case "$target" in
    "sun")
    #SOC ID for Sun is 618
    #SOC ID for Sun APQ is 639
    #SOC ID for CQ8750S is 705
    #SOC ID for CQ8725S is 706
    case "$soc_hwid" in
      618|639|705|706)
        setprop vendor.display.enable_fb_scaling 0
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.target.version 6
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 1
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.enable_latch_media_content 1
        setprop vendor.display.enable_inline_writeback 1
        setprop vendor.display.timed_render_enable 1
        setprop vendor.gralloc.hw_supports_ubwcp 0
        setprop vendor.gralloc.enable_snapalloc 1
        setprop vendor.display.enable_idle_content_fps_hint 1
        setprop vendor.display.enable_optimal_refresh_rate 1
        setprop vendor.display.refresh_rate_changeable 1
        setprop debug.sf.enable_vrr_config 1
        setprop vendor.display.enable_hal_self_refresh 1
        setprop vendor.display.cpu_cluster_boost_mask 3
        ;;
      655|681|659|694|686|720|721|731|732)
        #SOC ID for tuna is 655
        #SOC ID for tuna7 is 681
        #SOC ID for tuna APQ is 694
        #SOC ID for kera is 659
        #SOC ID for kera is 686
        #SOC ID for kera is 720
        #SOC ID for kera is 721
        #SOC ID for kera iot is 731
        #SOC ID for kera iot is 732
        setprop vendor.display.enable_fb_scaling 0
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.target.version 6
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 1
        setprop vendor.display.thermal.version 2
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.enable_latch_media_content 1
        setprop vendor.display.enable_inline_writeback 1
        setprop vendor.display.timed_render_enable 1
        setprop vendor.gralloc.hw_supports_ubwcp 0
        setprop vendor.gralloc.enable_snapalloc 1
        setprop vendor.display.enable_idle_content_fps_hint 1
        setprop vendor.display.enable_optimal_refresh_rate 1
        setprop vendor.display.refresh_rate_changeable 1
        setprop vendor.display.cpu_cluster_boost_mask 15
        ;;
    esac
    ;;
    "niobe")
    #SOC ID for niobe is 629
    case "$soc_hwid" in
        629|652)
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 0
        setprop vendor.display.target.version 4
        setprop vendor.display.disable_mitigated_fps 1
        setprop vendor.display.disable_cwb_idle_fallback 1
        setprop vendor.display.enable_rounded_corner 0  #disable HW RC
        setprop vendor.display.idle_time 0  #disable idle fallback
        setprop vendor.display.idle_time_inactive 0  #disable idle fallback
        setprop vendor.display.use_smooth_motion 0  #disable smooth motion
        setprop vendor.gralloc.enable_snapalloc 1
        setprop vendor.gralloc.hw_supports_ubwcp 0
        ;;
    esac
    ;;
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
        setprop vendor.display.timed_render_enable 1
        setprop vendor.gralloc.hw_supports_ubwcp 0
        setprop vendor.gralloc.enable_snapalloc 1
        ;;
      614|632)
        # SOC ID for Palawan is 614
        # SOC ID for Lamma is 632
        setprop vendor.display.enable_fb_scaling 0
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.target.version 5
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_perf_hint_large_comp_cycle 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.enable_spec_fence 1
        setprop vendor.display.thermal.version 1
        setprop vendor.display.enable_rc_support 1
        setprop vendor.display.enable_latch_media_content 1
        setprop vendor.display.enable_inline_writeback 1
        setprop vendor.display.timed_render_enable 1
        setprop vendor.gralloc.hw_supports_ubwcp 0
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
    "neo61")
    case "$soc_hwid" in
        554)
        setprop vendor.display.enable_null_display 1
        ;;
        579)
        setprop vendor.gralloc.enable_snapalloc 1
        setprop vendor.gralloc.use_dma_buf_heaps 1
        setprop vendor.display.enable_posted_start_dyn 2
        setprop vendor.display.enable_allow_idle_fallback 1
        setprop vendor.display.enable_rotator_ui 1
        setprop vendor.display.thermal.version 1
        setprop vendor.display.target.version 3
        setprop vendor.display.disable_mitigated_fps 1
        setprop vendor.display.enable_rounded_corner 0
        setprop vendor.display.wait_for_primary_display 1
        setprop vendor.display.force_gpu_composition 1
        setprop vendor.display.allow_tonemap_native 1
        ;;
    esac
    ;;
    "holi")
    # Set property for holi
    setprop vendor.display.target.version 2
    setprop vendor.display.disable_offline_rotator 0
    setprop vendor.display.disable_rotator_ubwc 1
    setprop vendor.display.enable_perf_hint_large_comp_cycle 0
    setprop vendor.display.enable_posted_start_dyn 1
    setprop vendor.display.enable_allow_idle_fallback 1
    ;;
esac
