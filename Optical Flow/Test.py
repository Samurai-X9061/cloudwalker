from FlowClass2 import OpticalFlowLK
from time import sleep

of = OpticalFlowLK(cam_index=0, fov_deg=60)

while True:
    fx, fy, q, dt = of.get_flow() #Add gyro integration
    print(f"Flow(rad): x={fx:.5f}, y={fy:.5f}, q={q}, dt={dt:.3f}")
    sleep(0.1)
