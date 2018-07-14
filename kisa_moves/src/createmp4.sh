#!/bin/bash
ffmpeg -framerate 15 -i frame%02d.png -r 15 output.mp4 -y
rm frame*.png

