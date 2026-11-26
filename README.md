# EVA-VR

Standalone repository for the EVA-CLIENT native PICO client: [github.com/Noietch/EVA-VR](https://github.com/Noietch/EVA-VR).

EVA-VR is a small native OpenXR client for PICO 4 Ultra. It replaces the
browser/WebXR input layer and keeps the controller input path available over
ADB-forwarded WebSocket connections.

The APK is named **EVA-VR** and uses the EVA logo as its launcher icon.
The complete Android/OpenXR source and the required OpenXR sample dependencies
are included in this repository; it does not depend on the EVA-CLIENT source
tree at build time.

## What It Does

- Runs as a native PICO VR application.
- Keeps a live diagnostic panel directly in front of the user's view.
- Shows head and controller `X/Y/Z` positions and `Roll/Pitch/Yaw` angles.
- Shows trigger, squeeze, thumbstick, button, click, and touch states for both controllers.
- Shows whether the EVA host WebSocket is connected.
- Sends the existing EVA `frame` protocol over WebSocket.
- Receives explicit `haptic` messages and invokes native OpenXR controller haptics only when both intensity and duration are positive.
- Never starts a standalone test pulse and never vibrates from ordinary input frames or `event_ack` messages.

## Requirements

- PICO 4 Ultra with USB debugging enabled.
- macOS or Linux development machine.
- Android SDK with platform 29 and build-tools 30.0.3.
- Android NDK `23.2.8568313`.
- Java 17.

Set the SDK and Java paths if they are not already configured:

```bash
export JAVA_HOME=/opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home
export ANDROID_HOME="$HOME/Library/Android/sdk"
```

## Install A Released APK

The easiest path is to download the latest APK release artifact (signed with the
project's development key):

- [Latest release](https://github.com/Noietch/EVA-VR/releases/latest)
- [Latest APK download](https://github.com/Noietch/EVA-VR/releases/latest/download/EVA-VR-v0.2.5.apk)

After enabling USB debugging on the PICO 4 Ultra:

```bash
adb devices
adb install -r EVA-VR-v0.2.5.apk
```

The installed app is named **EVA-VR** and its package is
`org.eva.pico.input`. It can also be started from the PICO app library:

```bash
adb shell am start -n org.eva.pico.input/.MainActivity
```

Select **Exit** in the status panel or hold both thumbstick clicks for two
seconds to leave EVA-VR. System-owned PICO buttons are left to the headset runtime.

The release APK is the recommended way to try the client. You only need the
source build below when changing the native OpenXR code or the UI.

## Build And Install From Source

The connected PICO 4 Ultra must have USB debugging enabled. From this repository:

```bash
export JAVA_HOME=/opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home
export ANDROID_HOME="$HOME/Library/Android/sdk"
./build.sh
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

## Connect To EVA

In EVA-CLIENT, select **DEVICE → Operation → VR → Start**. The workstation
establishes ADB reverse forwarding and launches EVA-VR with the input node's
current WebSocket endpoint and token. **Reconnect VR** relaunches the app.
With a configured Wi-Fi ADB target the button is **Reconnect Wi-Fi**; USB can
then be unplugged. The headset and workstation must share a reachable LAN.

The APK connects to a loopback WebSocket inside the PICO over USB or wireless
ADB. The workstation passes the endpoint using the `server_url` intent extra;
no endpoint entry in the headset is required. Without that extra, the app uses
`ws://127.0.0.1:43876/ws?token=eva`, which only works if the node uses that token.

Example with the existing EVA node on port `43876`:

```bash
adb reverse tcp:43876 tcp:43876
adb shell am force-stop org.eva.pico.input
adb shell am start -n org.eva.pico.input/.MainActivity
```

The in-view panel changes from `HOST: DISCONNECTED` to
`HOST: CONNECTED` after the WebSocket handshake succeeds.

Wear and wake the headset and both controllers during startup and collection.
The dashboard shows live head/left/right poses. Removing the headset can pause
the app and stop input even while ADB remains connected.
PICO safety-boundary dialogs can also pause EVA-VR. Complete the boundary
confirmation and return to EVA-VR to resume input. The client sends an explicit
WebSocket close reason when Android pauses it; EVA displays that reason and
stops a running rollout until the operator starts it again.

OpenXR session initialization waits for the Android activity to resume and
create its window. This lets NativeActivity complete its lifecycle callbacks
before PICO passthrough initialization. A sleeping headset logs
`Waiting for resumed Android window...` until it is awakened. A persistent
loading circle is not the dashboard; inspect `demos` and `EVA-VR` in logcat.

## Wire Protocol

The client sends one `frame` message after each complete left/right input
update. The payload is compatible with the current EVA WebXR node:

```json
{
  "type": "frame",
  "version": 1,
  "seq": 42,
  "client_time_ms": 1234567890,
  "reference_space": "local",
  "controllers": {
    "left": {
      "valid": true,
      "position": [0, 0, 0],
      "orientation_xyzw": [0, 0, 0, 1],
      "buttons": [],
      "axes": [],
      "profiles": ["pico-4-ultra"]
    },
    "right": {
      "valid": true,
      "position": [0, 0, 0],
      "orientation_xyzw": [0, 0, 0, 1],
      "buttons": [],
      "axes": [],
      "profiles": ["pico-4-ultra"]
    }
  }
}
```

The host may send an explicit haptic request using:

```json
{"type":"haptic","hand":"right","intensity":0.6,"duration_ms":80}
```

`hand` is `left` or `right`, `intensity` is clamped to `0..1`, and duration is
clamped to `1..1000` milliseconds. Requests with non-positive or invalid
intensity/duration are ignored. `event_ack` messages are logged but never
trigger vibration, and ordinary input frames never trigger vibration.

## Project Layout

- `app/src/main/cpp`: OpenXR renderer, controller input, UI, JNI bridge, and request-driven haptics.
- `app/src/main/java`: WebSocket connection and Android lifecycle code.
- `app/src/main/assets`: controller models and fonts used by the diagnostic scene.
- `assets/eva-logo.svg`: source for the EVA-VR launcher icon.
- `build.sh`: reproducible local build entry point.

The OpenXR sample code is included directly under `app/src/main`; there is no
`vendor/OpenXR_Demos` checkout or runtime patch step.

## License And Attribution

The native OpenXR sample portions retain their upstream source headers and
licenses. PICO SDK/OpenXR runtime components remain subject to their respective
PICO and Khronos licenses. EVA-specific bridge and application code is provided
for the EVA project.

## Stable Input Reference (source v0.2.7)

OpenXR uses right-handed metre coordinates. EVA's robot mapping expects +X
right, +Y up and -Z robot-forward. The controller must point in the robot's
forward direction at the first normal grip engagement. The headset only needs
to track the controllers; its viewing direction is not an alignment input.

At that first grip, EVA-VR projects the controller's **aim** ray (-Z) onto the
gravity-aligned horizontal plane and binds one shared heading for both hands.
Position and wrist orientation still come from **grip** poses. The heading is
then fixed for the OpenXR session: rotating either controller or the headset
does not continuously redefine translation axes. A new app/session binds again
automatically on its first normal grip; no forward-translation calibration is
required. Before a tracked, nonvertical aim can bind the heading, grip motion
is suppressed while face buttons remain usable.

Both grip poses are read directly in independent **STAGE** space at the frame's
predicted `XrTime`. The export transform is:

```text
controllerAlignedFromGrip = controllerAlignedFromInitialStage
                          * initialStageFromCurrentStage
                          * currentStageFromGrip
```

There is no initial-LOCAL yaw in this transform. PICO's observed LOCAL recenter
events can have `poseValid=false`, and its separately queried LOCAL grips and
LOCAL/STAGE relation can disagree. Neither is used to define the control
heading. LOCAL remains the rendering space. Known STAGE-origin changes are
accumulated at their `changeTime`. Without STAGE support, valid timed LOCAL
change events are required; an unknown origin change blocks input rather than
guessing a rotation. Both hands use the same position/orientation transform.
Frames report `local-floor` for controller-aligned STAGE-floor input, and
`local` for the LOCAL fallback. The host applies its robot-from-XR axis mapping
exactly once.

After a reference change the input socket/session is replaced and both controls
must return to neutral before the first fresh frame. Old grip anchors and
pending events are discarded without starting robot motion. Frames are published
only after both poses have been sampled; inactive or untracked grip actions are
invalid. A temporary failed room query can recover without rebinding heading.
An unknown STAGE-origin change or lost OpenXR events blocks input. Changes in
the physical robot/controller alignment require establishing the intended
forward direction again in a new session; the app cannot infer robot relocation.

Run the host-side regression without a headset or robot:

```bash
bash tests/run_unit.sh /path/to/work_dirs/debug/vr-reference-test
```

The regression covers effective-time boundaries, both hands, actual motion,
noncommuting/repeated rotations, application offsets, reference-space filtering,
unknown/malformed/lost-event cases, controller heading with pitch/roll,
shared axes for both hands, and six room headings by six movement directions.
It also writes `controller_direction_cases.jsonl` for host retargeting checks.
Build with `./build.sh --offline` and
validate PICO recentering with robot motion disabled.
