# Sony camera remote preview

This complete ESP-IDF project configures an ESP32-S31 as a Wi-Fi station,
connects to the Sony camera's smartphone remote-control hotspot, invokes Sony
Camera Remote API `startLiveview`, and continuously receives JPEG preview
frames. The S31 hardware JPEG decoder converts frames to RGB565, the application
scales them with centered letterboxing, and LVGL renders the preview and status
UI on the onboard 480x480 CO5300 AMOLED display.

## Project files

- `CMakeLists.txt`: ESP-IDF project entry point.
- `main/CMakeLists.txt`: application source files and component dependencies.
- `main/Kconfig.projbuild`: camera Wi-Fi/API settings and reconnect backoff.
- `sdkconfig.defaults`: ESP32-S31 target and Octal PSRAM defaults.
- `main/main.c`: Wi-Fi station setup, connection-state events, and reconnect task.
- `main/sony_liveview.c` / `main/sony_liveview.h`: Sony API, liveview parser,
  frame buffering, and queued shutter commands.
- `main/mosaico_display.c` / `main/mosaico_display.h`: CO5300 panel, hardware
  JPEG decode, preview scaling, LVGL UI, connection status, and preview FPS.
- `main/shutter_button.c` / `main/shutter_button.h`: onboard shutter button.
- `main/idf_component.yml`: managed CO5300, button, LVGL, and LVGL-port
  dependencies.

The complete application source and build configuration are in this project
directory; no generated code is required.

## Configure and build

Use an ESP-IDF installation that supports the `esp32s31` target. In a terminal
with ESP-IDF exported, run:

```sh
cd /path/to/sonycam
idf.py set-target esp32s31
idf.py menuconfig
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with the serial device (for example, `/dev/cu.usbmodemXXXX` on
macOS). The equivalent build-only command is `idf.py build`.

In menuconfig, open **Sony camera Wi-Fi** and enter the hotspot SSID and
password shown by the camera. Set **Sony Camera Remote API URL** if the endpoint
differs from `http://192.168.122.1/sony/camera`. Configure the initial and
maximum reconnect delay there as well. The SSID and password default to empty
and credentials are stored in local `sdkconfig`; do not commit that file.

The `esp32s31` target was built with the workspace's ESP-IDF 6.1 installation.
The CO5300 panel driver requires ESP-IDF 5.4 or later, and the LVGL port
requires ESP-IDF 5.2 or later. The complete project has not been validated on
ESP-IDF 5; use the tested ESP-IDF 6.1 setup or independently confirm component
and target compatibility before deploying on IDF 5.

Octal PSRAM is enabled and added to the general-purpose heap by
`sdkconfig.defaults`. The JPEG decoder buffers, scaled preview frame, and LVGL
draw buffers use PSRAM. After changing defaults in an existing checkout, use
`idf.py fullclean` and rebuild, or review the PSRAM settings in menuconfig;
existing `sdkconfig` values can override defaults.

Liveview JPEG decoding runs on the ESP32-S31 hardware decoder. The output is
scaled by nearest-neighbor sampling on the CPU to preserve aspect ratio. LVGL
uses PSRAM draw buffers with a smaller DMA transfer buffer to update the
display. The UI status bar reports Wi-Fi, liveview, and the rate of decoded
preview frames submitted to LVGL; the displayed FPS resets to zero after one
second without a new frame.

## Shutter button

The onboard active-low AI button on GPIO 7 controls the camera shutter:

- Press: call `actHalfPressShutter` to start autofocus.
- Hold for 600 ms: call `actTakePicture` once.
- Release: call `cancelHalfPressShutter`.

Camera commands are queued and serialized separately from the liveview stream.
The camera must support these Sony Remote API methods; unsupported commands are
reported in the serial log.

Wi-Fi reconnection runs in a dedicated task with exponential backoff, bounded
by the initial and maximum delays configured in menuconfig. Liveview retries
after a stream or API failure. The serial log reports the assigned IP address,
stream status, and periodic frame counts and sizes.

`sony_liveview_copy_latest_frame()` in [main/sony_liveview.h](main/sony_liveview.h)
copies the most recent JPEG into a caller-provided buffer. If the destination is
too small, it returns `ESP_ERR_INVALID_SIZE` and sets `frame_size` to the
required size. JPEG frames larger than 512 KiB are rejected.

## Notes and limitations

- The display pin mapping is selected from the board revision eFuse. Unknown
  revisions are rejected; verify the supported revision values if using a
  different ESP-Mosaico hardware revision.
- Wi-Fi credentials must match the camera's currently enabled smartphone
  remote-control hotspot. Camera hotspot availability and IP address depend on
  camera configuration.
- Sony Remote API methods and liveview behavior depend on camera model,
  firmware, and operating mode. Focus/capture calls may fail if the camera
  rejects the requested operation.
- The onboard AI button uses active-low GPIO 7: press starts half-press focus,
  holding for 600 ms takes one picture, and release cancels half-press.
- The project has been build-verified but still requires physical validation of
  the display, button, camera API, Wi-Fi recovery, and frame rate on the target
  board/camera.
