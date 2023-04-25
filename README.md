# DW3000-TWR-RTLS

An UWB Real-Time Location System (RTLS).

## Features
This project implements the following features:

- Using Qorvo/Decawave DW3000 UWB radios.
- Based on Zephyr RTOS.
- Using IEEE 802.15.4 standard data frame.
- Nodes can be easily configured as anchor or tag.

## System Components

Device is divided into anchor and tag.

### Tag

Send UWB ranging signal to measure the distance to the anchors.

### anchor

Helps tags for measure.

## Hardware

use https://github.com/vacabun/uwb-demoBoard

