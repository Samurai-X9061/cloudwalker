import cv2
import numpy as np
import time


class OpticalFlowPX4:
    def __init__(
        self,
        cam_index=0,
        width=640,
        height=480,
        fov_deg=60.0,
        grid_size=(4, 4),
        max_features_per_cell=20,
        K=None,
        D=None
    ):
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.width = width
        self.height = height

        # focal length (pixels)
        self.fx = (width / 2) / np.tan(np.deg2rad(fov_deg / 2))
        self.fy = self.fx

        # camera calibration (optional but recommended)
        self.K = K
        self.D = D

        self.grid_rows, self.grid_cols = grid_size
        self.max_features_per_cell = max_features_per_cell

        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS |
                      cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        self.prev_gray = None
        self.prev_pts = None
        self.prev_time = None

    # --------------------------------------------------------

    def _undistort(self, img):
        if self.K is None or self.D is None:
            return img
        return cv2.undistort(img, self.K, self.D)

    def _init_features_grid(self, gray):
        h, w = gray.shape
        pts = []

        cell_h = h // self.grid_rows
        cell_w = w // self.grid_cols

        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                y0 = r * cell_h
                y1 = (r + 1) * cell_h
                x0 = c * cell_w
                x1 = (c + 1) * cell_w

                roi = gray[y0:y1, x0:x1]
                corners = cv2.goodFeaturesToTrack(
                    roi,
                    maxCorners=self.max_features_per_cell,
                    qualityLevel=0.3,
                    minDistance=7,
                    blockSize=7
                )

                if corners is not None:
                    corners[:, 0, 0] += x0
                    corners[:, 0, 1] += y0
                    pts.append(corners)

        if pts:
            self.prev_pts = np.vstack(pts)
        else:
            self.prev_pts = None

        self.prev_gray = gray

    # --------------------------------------------------------

    def get_flow(self, gyro_x=0.0, gyro_y=0.0):
        """
        gyro_x, gyro_y : rad/s (body frame)
        Returns:
        flow_x_rad, flow_y_rad, quality (0-255), dt
        """

        ret, frame = self.cap.read()
        if not ret:
            return None

        now = time.time()
        gray = cv2.cvtColor(self._undistort(frame), cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None or self.prev_pts is None:
            self._init_features_grid(gray)
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

        if next_pts is None:
            self._init_features_grid(gray)
            return 0.0, 0.0, 0, dt

        good_old = self.prev_pts[status.flatten() == 1]
        good_new = next_pts[status.flatten() == 1]

        if len(good_old) < 15:
            self._init_features_grid(gray)
            return 0.0, 0.0, 0, dt

        # ----------------------------------------------------
        # FLOW VECTOR COMPUTATION
        # ----------------------------------------------------
        flow = good_new - good_old

        # Median-based outlier rejection (PX4FLOW-style)
        med = np.median(flow, axis=0)
        mad = np.median(np.abs(flow - med), axis=0) + 1e-6
        inliers = np.all(np.abs(flow - med) < 2.5 * mad, axis=1)

        flow = flow[inliers]
        inlier_ratio = np.sum(inliers) / len(inliers)

        if len(flow) < 10:
            self._init_features_grid(gray)
            return 0.0, 0.0, 0, dt

        mean_flow = np.median(flow, axis=0)

        # ----------------------------------------------------
        # PIXEL → RAD
        # ----------------------------------------------------
        flow_x = mean_flow[0] / self.fx
        flow_y = mean_flow[1] / self.fy

        # ----------------------------------------------------
        # GYRO COMPENSATION (CRITICAL)
        # camera facing down
        # ----------------------------------------------------
        flow_x -= gyro_y * dt
        flow_y += gyro_x * dt

        # ----------------------------------------------------
        # QUALITY METRIC (EKF-HONEST)
        # ----------------------------------------------------
        quality = int(
            min(
                255,
                255 * inlier_ratio * (len(flow) / 50)
            )
        )

        self.prev_gray = gray
        self.prev_pts = good_new.reshape(-1, 1, 2)

        return flow_x, flow_y, quality, dt

    def release(self):
        self.cap.release()
