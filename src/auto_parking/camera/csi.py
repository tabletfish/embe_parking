import cv2


def gst_pipeline(sensor_id, capture_width=1280, capture_height=720,
                 output_width=640, output_height=360, fps=30):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, "
        f"format=NV12, framerate={fps}/1 ! "
        f"nvvidconv ! video/x-raw, width={output_width}, height={output_height}, "
        "format=I420 ! videoconvert ! video/x-raw, format=BGR ! "
        "appsink max-buffers=1 drop=True sync=false"
    )


def open_csi_camera(sensor_id, config):
    cam_cfg = config["camera"]
    cap = cv2.VideoCapture(
        gst_pipeline(
            sensor_id=sensor_id,
            capture_width=cam_cfg["capture_width"],
            capture_height=cam_cfg["capture_height"],
            output_width=cam_cfg["output_width"],
            output_height=cam_cfg["output_height"],
            fps=cam_cfg["fps"],
        ),
        cv2.CAP_GSTREAMER,
    )
    if not cap.isOpened():
        raise RuntimeError(f"CSI camera {sensor_id} open failed")
    return cap


def open_video_source(source, config):
    if source is None:
        return open_csi_camera(config["camera"]["front_sensor_id"], config)
    if isinstance(source, int):
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config["camera"]["output_width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config["camera"]["output_height"])
        cap.set(cv2.CAP_PROP_FPS, config["camera"]["fps"])
        if not cap.isOpened():
            raise RuntimeError(f"Video device {source} open failed")
        return cap
    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise RuntimeError(f"Video source {source} open failed")
    return cap

