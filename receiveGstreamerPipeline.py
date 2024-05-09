import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

def gstreamer_pipeline_receive(
    host_ip='0.0.0.0',
    host_port=5000,
):
    return (
        "udpsrc port=%d ! "
        "application/x-rtp, encoding-name=H264, payload=96 ! "
        "rtph264depay ! "
        "avdec_h264 ! "
        "videoconvert ! "
        "autovideosink "
        % (
            host_port,
        )
    )

pipeline = Gst.parse_launch(gstreamer_pipeline_receive())

pipeline.set_state(Gst.State.PLAYING)

try:
    GLib.MainLoop().run()
except KeyboardInterrupt:
    pass

pipeline.set_state(Gst.State.NULL)