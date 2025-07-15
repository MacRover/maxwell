#!/bin/sh

gst-launch-1.0 v4l2src device=/dev/video0 !  videoconvert !  video/x-raw,width=640,height=480,framerate=25/1 !   nvvidconv ! nvv4l2h265enc !   h265parse !   rtph265pay config-interval=1 !   udpsink host=127.0.0.1 port=5000 sync=false

sleep 1

gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   avdec_h265 !   videoconvert !   videorate !   video/x-raw,framerate=15/1 !   autovideosink sync=false

gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   nvv4l2decoder !   nvvidconv !   videorate !   video/x-raw,framerate=15/1 !   v4l2sink device=/dev/video8 sync=false


: <<'END'
# Pipeline 1: Encoding & Transmission (NVIDIA Device)

```bash
gst-launch-1.0 v4l2src device=/dev/video20 !  
videoconvert !  
video/x-raw,width=640,height=480,framerate=30/1 !   
nvvidconv ! 
nvv4l2h265enc !   
h265parse !   
rtph265pay config-interval=1 !   
udpsink host=192.168.1.1 port=42067 sync=false
```

## Element-by-Element Breakdown:

### `v4l2src device=/dev/video20`
- **Purpose**: Video source from USB/camera device
- **What it does**: Captures raw video frames from camera at `/dev/video20`
- **Output**: Raw video data in whatever format the camera provides (could be YUY2, MJPEG, etc.)

### `videoconvert`
- **Purpose**: Software-based pixel format conversion
- **What it does**: Converts camera's native format to something standardized
- **Why needed**: Camera might output YUY2, MJPEG, or other formats that need normalization
- **Output**: Standardized uncompressed video format

### `video/x-raw,width=640,height=480,framerate=30/1`
- **Purpose**: Caps filter to enforce specific video parameters
- **What it does**: Forces the video stream to be exactly 640x480 at 30fps
- **Why needed**: Ensures consistent resolution before hardware encoding
- **Output**: Uncompressed video guaranteed to be 640x480@30fps

### `nvvidconv`
- **Purpose**: NVIDIA hardware-accelerated video conversion
- **What it does**: Transfers video data to GPU memory and optimizes format for hardware encoder
- **Why needed**: Prepares video for efficient hardware encoding (CPU→GPU memory transfer)
- **Output**: GPU-resident video data in format optimized for NVIDIA encoder

### `nvv4l2h265enc`
- **Purpose**: NVIDIA hardware H.265/HEVC encoder
- **What it does**: Compresses video using dedicated hardware encoding blocks
- **Benefits**: Much faster than software encoding, lower CPU usage
- **Output**: Compressed H.265 video stream

### `h265parse`
- **Purpose**: H.265 stream parser and formatter
- **What it does**: Analyzes H.265 stream, extracts metadata, ensures proper frame boundaries
- **Why needed**: Prepares stream for RTP packetization
- **Output**: Properly formatted H.265 stream with metadata

### `rtph265pay config-interval=1`
- **Purpose**: RTP (Real-time Transport Protocol) payloader for H.265
- **What it does**: Packages H.265 data into RTP packets for network transmission
- **`config-interval=1`**: Sends codec configuration data every 1 second (helps with stream recovery)
- **Output**: RTP packets containing H.265 data

### `udpsink host=192.168.1.1 port=42067 sync=false`
- **Purpose**: UDP network sink
- **What it does**: Transmits RTP packets over UDP to specified host/port
- **`sync=false`**: Don't sync to clock (prioritize low latency over perfect timing)
- **Output**: UDP packets sent over network

---

# Pipeline 2: Reception & Decoding (Non-NVIDIA Device)

```bash
gst-launch-1.0 udpsrc port=42067 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   
rtpjitterbuffer latency=200 !   
rtph265depay !   
h265parse !   
queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   
avdec_h265 !   
videoconvert !   
videorate !   
video/x-raw,framerate=15/1 !   
autovideosink sync=false
```

## Element-by-Element Breakdown:

### `udpsrc port=42067 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265"`
- **Purpose**: UDP network source
- **What it does**: Receives UDP packets on port 42067
- **Caps specification**: Tells GStreamer to expect RTP video packets with H.265 encoding
- **`clock-rate=90000`**: Standard RTP clock rate for video (90kHz)
- **Output**: Raw UDP packets containing RTP data

### `rtpjitterbuffer latency=200`
- **Purpose**: Network jitter buffer
- **What it does**: Buffers incoming RTP packets to smooth out network timing variations
- **`latency=200`**: 200ms buffer to handle network jitter
- **Why needed**: Network packets arrive at irregular intervals; this smooths playback
- **Output**: Properly timed RTP packets

### `rtph265depay`
- **Purpose**: RTP H.265 depayloader
- **What it does**: Extracts H.265 data from RTP packets (opposite of rtph265pay)
- **Why needed**: Removes RTP packaging to get raw H.265 stream
- **Output**: Raw H.265 compressed video stream

### `h265parse`
- **Purpose**: H.265 stream parser
- **What it does**: Parses H.265 stream, identifies frame boundaries, extracts metadata
- **Why needed**: Prepares stream for decoder
- **Output**: Properly formatted H.265 stream

### `queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0`
- **Purpose**: Large buffer queue
- **What it does**: Provides substantial buffering between network and decoder
- **Parameters**: 
  - `max-size-buffers=3000`: Can hold up to 3000 frames
  - `max-size-time=0`: No time limit
  - `max-size-bytes=0`: No byte limit
- **Why needed**: Absorbs network irregularities and decoder timing variations

### `avdec_h265`
- **Purpose**: Software H.265 decoder (FFmpeg-based)
- **What it does**: Decompresses H.265 video back to raw frames
- **Why this decoder**: Uses CPU (no NVIDIA hardware available)
- **Output**: Uncompressed video frames

### `videoconvert`
- **Purpose**: Software pixel format conversion
- **What it does**: Converts decoded video to standard format
- **Why needed**: Decoder output might not match what display expects
- **Output**: Standard uncompressed video format

### `videorate`
- **Purpose**: Frame rate adjustment
- **What it does**: Adjusts frame rate by duplicating or dropping frames
- **Why needed**: Prepares for frame rate conversion
- **Output**: Video stream ready for frame rate modification

### `video/x-raw,framerate=15/1`
- **Purpose**: Frame rate caps filter
- **What it does**: Reduces frame rate from 30fps to 15fps
- **Why done**: Reduces display load on receiving device
- **Output**: 15fps video stream

### `autovideosink sync=false`
- **Purpose**: Automatic video display
- **What it does**: Displays video using best available video sink
- **`sync=false`**: Don't sync to clock (prioritize low latency)
- **Output**: Video displayed on screen

## Key Differences Between Pipelines:

| Aspect | Pipeline 1 (NVIDIA) | Pipeline 2 (Non-NVIDIA) |
|--------|-------------------|------------------------|
| **Encoding** | Hardware (nvv4l2h265enc) | Software decoding (avdec_h265) |
| **Performance** | Low CPU usage | Higher CPU usage |
| **Conversion** | nvvidconv (GPU) | videoconvert (CPU) |
| **Frame Rate** | 30fps input | 15fps output |
| **Buffering** | Minimal | Substantial (jitter buffer + queue) |

## Why This Setup Works:

1. **Hardware encoding** on NVIDIA device provides efficient compression
2. **Network transmission** handles the data transfer
3. **Software decoding** on non-NVIDIA device provides compatibility
4. **Frame rate reduction** (30→15fps) reduces load on receiving device
5. **Extensive buffering** on receiver handles network irregularities
END