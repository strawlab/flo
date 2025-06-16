# Fast Lock On - FLO

[![DOI](https://zenodo.org/badge/845941788.svg)](https://zenodo.org/doi/10.5281/zenodo.13757150)

FLO is a method for videography using a camera system that automatically moves
to follow the subject.

FLO is described in the following publication:

[Free access]: https://strawlab.org/publications/?flo-paper-link

Vo-Doan TT, Titov VV, Harrap MJM, Lochner S, Straw AD. High Resolution Outdoor Videography of Insects Using Fast Lock-On Tracking. Science Robotics 9(95), eadm7689 (2024) [Free-access link][Free access] DOI: 10.1126/scirobotics.adm7689.

![flo video rollup](https://strawlab.org/assets/images/flo-video-rollup-tiny.gif)

## What is in this repository

This repository contains the source code for the primary FLO executable, called
`flo`. `flo` is written in the Rust language. `flo` receives image coordinates
from Strand Cam `imops` and controls motors. This is the main, high-level FLO
program.

## Other FLO-related repositories

- https://github.com/strawlab/flo-data-analysis data analysis for FLO data
- https://github.com/strawlab/flo-hardware build instructions and manual for the
  FLO BYO-camera insect tracking/filming setup.
- https://github.com/strawlab/red-button-trigger-timestamp hardware to record
  times of external trigger events.

## Overview of multiple pieces when using PWM servo motors

Other variants trade `rpipico-pantilt` and the PWM servo motors for other types
of motors, such as Trinamic stepper motors or SimpleBGC gimbal motors. Strand
Camera is available [here](https://strawlab.org/strand-cam/).

```
+-------------+
|             |         HTTP
|   FLO UI    +<----------------------+
|    (browser)|                       |
+-------------+                       |
                                      v
+-----------------------+      +-------------+     +-----------------+
|                       | UDP  |             | USB |                 |
| Strand Camera `imops` +----->|    flo      +<--->| rpipico-pantilt |
|                   (PC)|      |         (PC)|     |        (RP Pico)|
+-----------------------+      +-------------+     +-------+---------+
            ^                                              |
            |                                              |
            |                                              |
            |                                              v
       +----+---+                                      +--------+
       |        |                                      | PWM    |
       | camera |<-------------------------------------+ servo  |
       |        |                                      | motors |
       +--------+                                      +--------+
```

(Ascii art made with http://asciiflow.com/ )

## Overview of top-level directories

- `crates` Rust crates (libraries) used by `flo` and other software
- `firmware/beamdriver` Firmware for IR LED source
- `firmware/flo-tilta-dongle` Firmware for USB dongle to control Tilta motors.
- `firmware/rpipico-pantilt` Firmware for RPi Pico to control PWM servo motors.
- `src` Rust source code for `flo`

# How to

## How to connect to strandcam

1. Run strandcam. 

Specify the networking port using command-line argument: `--http-server-addr 0.0.0.0:5555` (5555 is the port number). Use different ports for different cameras (for example, 5555 for the primary camera, and 5556 for the secondary camera). Specify the camera to use with `--camera-name Basler-40522040` (40522040 is the serial number of the camera)

2. obtain the token link for each strandcam

Strandcam will dump a lot of info into the terminal. Look for a link like `* predicted URL http://127.0.0.1:5555/?token=f6868732-14da-427d-ac02-6f0c42d8cbff`. This is the link with the token. 

Repeat for the second camera, it will be a different token. Keep the strandcam instances running for the next step. 

3. put the token link into the FLO config file, and run FLO once.

Look for url fields in the FLO config file:
```
  strand_cam_main:
    url: http://127.0.0.1:5555    # <-- append the token part here
    on_attach_json_commands:
    - '{"ToCamera":{"SetImOpsCenterX":960}}'
    - '{"ToCamera":{"SetImOpsCenterY":600}}'
    - '{"ToCamera":{"SetImOpsThreshold":200}}'
    - '{"ToCamera":{"ToggleImOpsDetection":true}}'
    - '{"ToCamera":{"SetMp4MaxFramerate": "Fps60"}}'
    - '{"ToCamera":{"SetMp4Codec":{"Ffmpeg":{"device_args":[["-vaapi_device","/dev/dri/renderD128"]],"pre_codec_args":[["-vf","format=nv12,hwupload"]],"codec":"h264_vaapi","post_codec_args":null}}}}'
  strand_cam_secondary:
    url: http://127.0.0.1:5556    # <-- ... and here
    on_attach_json_commands:
    - '{"ToCamera":{"SetImOpsCenterX":960}}'
    - '{"ToCamera":{"SetImOpsCenterY":600}}'
    - '{"ToCamera":{"SetImOpsThreshold":200}}'
    - '{"ToCamera":{"ToggleImOpsDetection":true}}'
    - '{"ToCamera":{"SetMp4MaxFramerate": "Fps30"}}'
    - '{"ToCamera":{"SetMp4Codec":{"Ffmpeg":{"device_args":[["-vaapi_device","/dev/dri/renderD128"]],"pre_codec_args":[["-vf","format=nv12,hwupload"]],"codec":"h264_vaapi","post_codec_args":null}}}}'
```
Add the token part to the url fields. Then run FLO.

If FLO connects to the camera(s), a cookie will be saved, and the token will not be needed again to establish a connection. The cookie lives for 400 days. The token is only valid for the current strandcam session, and is refreshed (becomes invalid) if strandcam is restarted.

4. upon successful connection, remove the token part from the url fields.

Done!

Note that with this connection method, you have an option to configure imops and other aspects of strandcam automatically.