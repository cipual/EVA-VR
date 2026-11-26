#include "input_reference_space.h"
#include <cassert>
#include <iostream>
#include <limits>

using eva::IdentityPose;
using eva::InputReferenceSpace;

static void near(float actual, float expected) { assert(std::abs(actual - expected) < 2e-5f); }
static void position(const XrPosef& p, float x, float y, float z) {
    near(p.position.x, x); near(p.position.y, y); near(p.position.z, z);
}
static void identityRotation(const XrPosef& p) {
    near(p.orientation.x, 0); near(p.orientation.y, 0); near(p.orientation.z, 0);
    near(std::abs(p.orientation.w), 1);
}
static XrPosef yaw90() { return {{0, std::sqrt(0.5f), 0, std::sqrt(0.5f)}, {0, 0, 0}}; }
static auto change(XrTime time, XrPosef pose, bool valid = true,
                   XrReferenceSpaceType type = XR_REFERENCE_SPACE_TYPE_LOCAL) {
    XrEventDataReferenceSpaceChangePending e{};
    e.type = XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING;
    e.referenceSpaceType = type; e.changeTime = time; e.poseValid = valid; e.poseInPreviousSpace = pose;
    return e;
}

int main() {
    // Predicted display time controls the boundary, not event delivery time.
    InputReferenceSpace s;
    auto reset = yaw90(); reset.position = {2, 0, -1};
    s.queue(change(100, reset));
    assert(!s.advance(99)); position(s.stableFromApp(), 0, 0, 0); identityRotation(s.stableFromApp());
    assert(s.advance(100)); assert(s.valid()); assert(!s.advance(101));

    // Independent analytic coordinates of the SAME stationary left/right hands.
    const auto inverseYaw = XrQuaternionf{0, -std::sqrt(0.5f), 0, std::sqrt(0.5f)};
    const XrPosef newLeft{inverseYaw, {-0.5f, 1, -1.7f}};
    const XrPosef newRight{inverseYaw, {-0.1f, 0.8f, -2.4f}};
    const auto stableLeft = eva::ComposePose(s.stableFromApp(), newLeft);
    const auto stableRight = eva::ComposePose(s.stableFromApp(), newRight);
    position(stableLeft, .3f, 1, -.5f); identityRotation(stableLeft);
    position(stableRight, -.4f, .8f, -.9f); identityRotation(stableRight);

    // A real 10cm movement remains 10cm along the original forward axis after reset.
    auto moved = newLeft; moved.position.x += .1f;
    const auto stableMoved = eva::ComposePose(s.stableFromApp(), moved);
    near(stableMoved.position.x - stableLeft.position.x, 0);
    near(stableMoved.position.z - stableLeft.position.z, -.1f);

    // Noncommuting rotations catch inverted composition and event ordering errors.
    InputReferenceSpace ordered;
    const XrPosef x90{{std::sqrt(.5f), 0, 0, std::sqrt(.5f)}, {0, 0, 0}};
    ordered.queue(change(20, x90)); ordered.queue(change(10, yaw90()));
    assert(ordered.advance(20));
    position(eva::ComposePose(ordered.stableFromApp(), {{0, 0, 0, 1}, {0, 1, 0}}), 1, 0, 0);
    const auto q = ordered.stableFromApp().orientation;
    near(q.x, .5f); near(q.y, .5f); near(q.z, -.5f); near(q.w, .5f);

    // Repeated recentering must accumulate rather than overwrite the correction.
    InputReferenceSpace twice;
    twice.queue(change(10, yaw90())); twice.queue(change(20, yaw90()));
    twice.advance(20);
    position(eva::ComposePose(twice.stableFromApp(), {{0, 0, 0, 1}, {1, 2, 3}}), -1, 2, -3);
    twice.queue(change(30, yaw90())); twice.queue(change(40, yaw90())); twice.advance(40);
    identityRotation(twice.stableFromApp());

    // xrCreateReferenceSpace offsets are NOT included in poseInPreviousSpace.
    auto offset = IdentityPose(); offset.position = {1, 0, 0};
    InputReferenceSpace offsetSpace(XR_REFERENCE_SPACE_TYPE_LOCAL, offset);
    offsetSpace.queue(change(1, yaw90())); offsetSpace.advance(1);
    position(offsetSpace.stableFromApp(), -1, 0, -1);

    InputReferenceSpace otherSpace;
    otherSpace.queue(change(1, yaw90(), true, XR_REFERENCE_SPACE_TYPE_STAGE));
    assert(!otherSpace.advance(1)); identityRotation(otherSpace.stableFromApp());
    InputReferenceSpace stage(XR_REFERENCE_SPACE_TYPE_STAGE);
    stage.queue(change(1, yaw90(), true, XR_REFERENCE_SPACE_TYPE_STAGE)); assert(stage.advance(1));
    position(eva::ComposePose(stage.stableFromApp(), {{0, 0, 0, 1}, {1, 0, 0}}), 0, 0, -1);
    assert(!InputReferenceSpace(XR_REFERENCE_SPACE_TYPE_VIEW).valid());

    InputReferenceSpace unknown;
    auto undefined = IdentityPose(); undefined.position.x = std::numeric_limits<float>::quiet_NaN();
    unknown.queue(change(10, undefined, false));
    unknown.advance(9); assert(unknown.valid()); unknown.advance(10); assert(!unknown.valid());
    unknown.queue(change(20, IdentityPose())); unknown.advance(20); assert(!unknown.valid());
    InputReferenceSpace malformed;
    malformed.queue(change(1, undefined)); malformed.advance(1); assert(!malformed.valid());
    InputReferenceSpace badQuaternion;
    auto zeroQuaternion = IdentityPose(); zeroQuaternion.orientation.w = 0;
    badQuaternion.queue(change(1, zeroQuaternion)); badQuaternion.advance(1); assert(!badQuaternion.valid());
    InputReferenceSpace lostEvents;
    lostEvents.invalidate(); lostEvents.queue(change(1, IdentityPose())); lostEvents.advance(1);
    assert(!lostEvents.valid());

    // A runtime-valid but untracked (inferred/cached) pose must not enable control.
    XrSpaceLocation hand{}; hand.pose = IdentityPose();
    hand.locationFlags = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    assert(!eva::TrackedPose(hand));
    hand.locationFlags |= XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    assert(eva::TrackedPose(hand));
    hand.pose.position.y = std::numeric_limits<float>::quiet_NaN(); assert(!eva::TrackedPose(hand));

    std::cout << "PASS: time boundary, both hands, real motion, noncommuting composition, repeated reset, offsets, space filtering, unknown/lost events\n";
}
