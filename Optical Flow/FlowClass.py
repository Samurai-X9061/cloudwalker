import cv2
import numpy as np
import time


class OpticalFlowLK:
    def __init__(
        self,
        cam_index=0,
        width=640,
        height=480,
        fov_deg=60.0,
        max_corners=200
    ):
        """
        Lightweight optical flow generator (EKF-ready)

        cam_index : webcam index (0 for PC, /dev/video0 on RPi)
        fov_deg   : horizontal field of view of camera
        """

        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.width = width
        self.height = height

        # focal length in pixels
        self.fx = (width / 2) / np.tan(np.deg2rad(fov_deg / 2))
        self.fy = self.fx

        self.feature_params = dict(
            maxCorners=max_corners,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )

        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS |
                      cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        self.prev_gray = None
        self.prev_pts = None
        self.prev_time = None

    def _init_features(self, gray):
        self.prev_pts = cv2.goodFeaturesToTrack(gray, **self.feature_params)
        self.prev_gray = gray

    def get_flow(self):
        """
        Returns:
        flow_x_rad, flow_y_rad, quality (0-255), dt
        """

        ret, frame = self.cap.read()
        if not ret:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        now = time.time()

        if self.prev_gray is None or self.prev_pts is None:
            self._init_features(gray)
            self.prev_time = now
            return 0.0, 0.0, 0, 0.0

        dt = now - self.prev_time
        self.prev_time = now

        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_pts,
            None,
            **self.lk_params
        )

        good_old = self.prev_pts[status == 1]
        good_new = next_pts[status == 1]

        if len(good_old) < 10:
            self._init_features(gray)
            return 0.0, 0.0, 0, dt

        flow_pixels = good_new - good_old
        mean_flow = np.mean(flow_pixels, axis=0)

        # pixel → rad
        flow_x_rad = mean_flow[0] / self.fx
        flow_y_rad = mean_flow[1] / self.fy

        # quality metric
        quality = min(255, int(len(good_old) / 2))

        self.prev_gray = gray
        self.prev_pts = good_new.reshape(-1, 1, 2)

        return flow_x_rad, flow_y_rad, quality, dt

    def release(self):
        self.cap.release()
