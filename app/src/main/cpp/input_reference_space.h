#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <openxr/openxr.h>
#include <glm/gtc/quaternion.hpp>

namespace eva {

inline XrPosef IdentityPose() { return {{0, 0, 0, 1}, {0, 0, 0}}; }

inline glm::quat Quaternion(const XrPosef& pose) {
    return {pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
}

// aFromB * bFromC = aFromC. OpenXR poses are right-handed, metres, xyzw.
inline XrPosef ComposePose(const XrPosef& a, const XrPosef& b) {
    const auto aq = Quaternion(a);
    const auto q = glm::normalize(aq * Quaternion(b));
    const auto p = glm::vec3(a.position.x, a.position.y, a.position.z) +
        aq * glm::vec3(b.position.x, b.position.y, b.position.z);
    return {{q.x, q.y, q.z, q.w}, {p.x, p.y, p.z}};
}

inline XrPosef InversePose(const XrPosef& pose) {
    const auto q = glm::conjugate(Quaternion(pose));
    const auto p = q * -glm::vec3(pose.position.x, pose.position.y, pose.position.z);
    return {{q.x, q.y, q.z, q.w}, {p.x, p.y, p.z}};
}

inline bool ValidRigidPose(const XrPosef& pose) {
    const auto q = Quaternion(pose);
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
        std::isfinite(pose.position.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
        std::isfinite(q.z) && std::isfinite(q.w) && std::abs(glm::dot(q, q) - 1.f) < 1e-3f;
}

inline bool TrackedPose(const XrSpaceLocation& location) {
    constexpr XrSpaceLocationFlags required = XR_SPACE_LOCATION_POSITION_VALID_BIT |
        XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT |
        XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    return (location.locationFlags & required) == required && ValidRigidPose(location.pose);
}

// Keep exported input in the first app space, independently of rendering space.
// poseInPreviousSpace is NEW natural origin expressed in the OLD natural space.
class InputReferenceSpace {
public:
    explicit InputReferenceSpace(XrReferenceSpaceType type = XR_REFERENCE_SPACE_TYPE_LOCAL,
                                 XrPosef naturalFromApp = IdentityPose())
        : type_(type), naturalFromApp_(naturalFromApp),
          valid_((type == XR_REFERENCE_SPACE_TYPE_LOCAL || type == XR_REFERENCE_SPACE_TYPE_STAGE) &&
                 ValidRigidPose(naturalFromApp)) {}

    void queue(const XrEventDataReferenceSpaceChangePending& event) {
        if (event.referenceSpaceType != type_) return;
        pending_.push_back({event.changeTime, event.poseValid == XR_TRUE,
                           event.poseValid ? event.poseInPreviousSpace : IdentityPose()});
        std::stable_sort(pending_.begin(), pending_.end(),
                         [](const Change& a, const Change& b) { return a.time < b.time; });
    }

    // Must use the SAME XrTime as xrLocateSpace; never compensate before changeTime.
    bool advance(XrTime sampleTime) {
        bool changed = false;
        while (!pending_.empty() && pending_.front().time <= sampleTime) {
            const auto change = pending_.front();
            pending_.erase(pending_.begin());
            changed = true;
            if (!change.valid || !ValidRigidPose(change.oldFromNew)) valid_ = false;
            if (valid_) {
                // Event poses exclude xrCreateReferenceSpace's application offset.
                const auto oldAppFromNewApp = ComposePose(InversePose(naturalFromApp_),
                    ComposePose(change.oldFromNew, naturalFromApp_));
                stableFromApp_ = ComposePose(stableFromApp_, oldAppFromNewApp);
            }
        }
        return changed;
    }

    // A missing/unknown origin change cannot be repaired by a later relative change.
    void invalidate() { valid_ = false; }
    bool valid() const { return valid_; }
    XrReferenceSpaceType type() const { return type_; }
    const XrPosef& stableFromApp() const { return stableFromApp_; }

private:
    struct Change { XrTime time; bool valid; XrPosef oldFromNew; };
    XrReferenceSpaceType type_;
    XrPosef naturalFromApp_;
    XrPosef stableFromApp_ = IdentityPose();
    bool valid_ = true;
    std::vector<Change> pending_;
};

}  // namespace eva
