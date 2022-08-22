#include "SunlightEnhancement.h"

#define __CLASS__ "SunlightEnhancement"

namespace vendor {
namespace lineage {
namespace livedisplay {
namespace V2_0 {
namespace drm {

SunlightEnhancement::SunlightEnhancement(sdm::HWCSession *hwc_session) :
    hwc_session_(hwc_session) {}

// Methods from ::vendor::lineage::livedisplay::V2_0::ISunlightEnhancement follow.
Return<bool> SunlightEnhancement::isEnabled() {
    bool enabled = false;
    hwc_session_->GetHbmState(&enabled);
    DLOGE("%s: enable=%d", __func__, enabled);
    return enabled;
}

Return<bool> SunlightEnhancement::setEnabled(bool enabled) {
    DLOGE("%s: set enable=%d", __func__, enabled);
    return hwc_session_->SetHbmState(enabled) == 0;
}

}  // namespace drm
}  // namespace V2_0
}  // namespace livedisplay
}  // namespace lineage
}  // namespace vendor
