#include "input_reference_space.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <fstream>
#include <iomanip>

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

int main(int argc, char** argv) {
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

    // Controller forward, rather than headset/LOCAL yaw, defines the robot
    // reference. Both hands and all six translation directions must agree.
    std::ofstream fixtures;
    if (argc > 1) { fixtures.open(argv[1]); fixtures << std::setprecision(9); }
    for (float yaw : {-3.14159265f, -1.57079633f, -.7f, 0.f, 1.57079633f, 2.4f}) {
        InputReferenceSpace room(XR_REFERENCE_SPACE_TYPE_STAGE);
        eva::ControllerReferenceSpace frame;
        const float c = std::cos(yaw), sn = std::sin(yaw);
        XrSpaceLocation aim{}; aim.type = XR_TYPE_SPACE_LOCATION;
        aim.locationFlags = 15;
        aim.pose = {{0, std::sin(yaw / 2), 0, std::cos(yaw / 2)}, {0, 1, 0}};
        assert(frame.bind(aim, room, true));
        assert(frame.bound());
        // A later hand/head rotation cannot redefine the established axes.
        aim.pose = IdentityPose(); assert(frame.bind(aim, room, true));
        for (int side = 0; side < 2; ++side) {
            const float startX = side ? .3f : -.3f;
            const XrPosef start{{0, std::sin(yaw / 2), 0, std::cos(yaw / 2)},
                                {c * startX - sn * .5f, 1, -sn * startX - c * .5f}};
            const auto fromTracking = frame.inputFromTracking(room);
            const auto initial = eva::ComposePose(fromTracking, start);
            position(initial, startX, 1, -.5f); identityRotation(initial);
            for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
                const float dx = axis == 0 ? sign * .01f : 0;
                const float dy = axis == 1 ? sign * .01f : 0;
                const float dz = axis == 2 ? sign * .01f : 0;
                auto moved = start;
                moved.position.x += c * dx + sn * dz;
                moved.position.y += dy;
                moved.position.z += -sn * dx + c * dz;
                const auto exported = eva::ComposePose(fromTracking, moved);
                position(exported, startX + dx, 1 + dy, -.5f + dz);
                identityRotation(exported);
                if (fixtures.is_open()) {
                    fixtures << "{\"yaw\":" << yaw << ",\"hand\":" << side
                             << ",\"axis\":" << axis << ",\"sign\":" << sign
                             << ",\"origin\":[" << initial.position.x << ',' << initial.position.y << ',' << initial.position.z
                             << "],\"position\":[" << exported.position.x << ',' << exported.position.y << ',' << exported.position.z << "]}\n";
                }
            }
        }
        // Known room resets preserve the original physical direction.
        const auto before = frame.inputFromTracking(room);
        auto shift = yaw90(); shift.position = {2, 0, -1};
        room.queue(change(10, shift, true, XR_REFERENCE_SPACE_TYPE_STAGE)); room.advance(10);
        const XrPosef oldGrip{{0, 0, 0, 1}, {.2f, 1, -.6f}};
        const auto newGrip = eva::ComposePose(eva::InversePose(shift), oldGrip);
        const auto oldOutput = eva::ComposePose(before, oldGrip);
        const auto newOutput = eva::ComposePose(frame.inputFromTracking(room), newGrip);
        position(newOutput, oldOutput.position.x, oldOutput.position.y, oldOutput.position.z);
        identityRotation(eva::ComposePose(eva::InversePose(oldOutput), newOutput));
    }
    // Startup/reconnect can receive a preparation squeeze before neutral input
    // reaches the host. It must not lock a heading that survives the later grip.
    {
        InputReferenceSpace room(XR_REFERENCE_SPACE_TYPE_STAGE);
        eva::ControllerReferenceSpace frame;
        XrSpaceLocation aim{}; aim.type = XR_TYPE_SPACE_LOCATION;
        aim.locationFlags = 15; aim.pose = yaw90();
        assert(!frame.bind(aim, room, false));
        assert(!frame.bound());
        aim.pose = IdentityPose();
        assert(frame.bind(aim, room, true));
        position(eva::ComposePose(frame.inputFromTracking(room),
                                 {{0, 0, 0, 1}, {0, 0, -.1f}}), 0, 0, -.1f);
        // Once bound, connection/neutral transitions do not redefine the axes.
        aim.pose = yaw90();
        assert(frame.bind(aim, room, false));
        identityRotation(frame.inputFromTracking(room));
    }
    // LOCAL unknown recentering does not invalidate the independent room frame.
    InputReferenceSpace picoLocal;
    picoLocal.queue(change(10, undefined, false)); picoLocal.advance(10); assert(!picoLocal.valid());
    InputReferenceSpace room(XR_REFERENCE_SPACE_TYPE_STAGE);
    eva::ControllerReferenceSpace independent;
    XrSpaceLocation aim{}; aim.type = XR_TYPE_SPACE_LOCATION; aim.pose = yaw90(); aim.locationFlags = 15;
    assert(independent.bind(aim, room, true));
    assert(!eva::ControllerReferenceSpace().bind(aim, picoLocal, true));
    // Pitch/roll in a normal hold must not tilt the gravity-aligned axes.
    eva::ControllerReferenceSpace tilted;
    const XrPosef pitch{{std::sin(.2f), 0, 0, std::cos(.2f)}, {0, 0, 0}};
    const XrPosef roll{{0, 0, std::sin(.3f), std::cos(.3f)}, {0, 0, 0}};
    aim.pose = eva::ComposePose(yaw90(), eva::ComposePose(pitch, roll));
    assert(tilted.bind(aim, room, true));
    position(eva::ComposePose(tilted.inputFromTracking(room), {{0, 0, 0, 1}, {-1, 2, 0}}), 0, 2, -1);
    // Missing tracking or a vertical pointing ray cannot establish a heading.
    eva::ControllerReferenceSpace unbound;
    aim.locationFlags = 3; assert(!unbound.bind(aim, room, true));
    aim.locationFlags = 15; aim.pose = x90; assert(!unbound.bind(aim, room, true)); assert(!unbound.bound());
    aim.pose = yaw90(); assert(unbound.bind(aim, room, true));
    room.invalidate(); assert(!eva::ControllerReferenceSpace().bind(aim, room, true));

    std::cout << "PASS: timed transforms, both hands, all six directions, controller-defined heading, fixed shared reference, room resets and invalid tracking\n";
}
