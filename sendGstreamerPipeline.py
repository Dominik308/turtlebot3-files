import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
from threading import Thread
import signal
from time import sleep

Gst.init(None)

def gstreamer_pipeline_send(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
    host_ip='192.168.178.146',  # 10.1.1.0
    host_port=5000,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! "
        "rtph264pay ! "
        "udpsink host=%s port=%d "
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
            host_ip,
            host_port,
        )
    )

def cleanup(signum, frame):
    pipeline.set_state(Gst.State.NULL)
    gstream_loop.quit()

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

gstream_loop = GLib.MainLoop()
gstream_loop_thread = Thread(target=gstream_loop.run)
gstream_loop_thread.start()

pipeline = Gst.parse_launch(gstreamer_pipeline_send(flip_method=0))

pipeline.set_state(Gst.State.PLAYING)
print(pipeline)
try:
    while True:
        sleep(0.1)
except KeyboardInterrupt:
    cleanup(None, None)

gstream_loop.quit()
pipeline.set_state(Gst.State.NULL)
gstream_loop_thread.join()
