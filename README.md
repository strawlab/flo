# Fast Lock On - FLO

[![DOI](https://zenodo.org/badge/845941788.svg)](https://zenodo.org/doi/10.5281/zenodo.13757150)

FLO is a method for videography using a camera system that automatically moves
to follow the subject.

FLO is described in the following publication:

[FLO paper]: https://doi.org/10.1101/2023.12.20.572558

Vo-Doan TT, Titov VV, Harrap MJM, Lochner S, Straw AD. High Resolution Outdoor Videography of Insects Using Fast Lock-On Tracking. bioRxiv (2023) [doi:10.1101/2023.12.20.572558][FLO paper].

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
