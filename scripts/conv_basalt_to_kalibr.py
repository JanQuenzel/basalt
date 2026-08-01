# written by gemini 3.6 thinking
#!/usr/bin/env python3
import argparse
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml


def pose_to_matrix(pose_dict):
    """Converts a Basalt pose dict {px, py, pz, qx, qy, qz, qw} to a 4x4 matrix using SciPy."""
    px, py, pz = pose_dict["px"], pose_dict["py"], pose_dict["pz"]
    qx, qy, qz, qw = (
        pose_dict["qx"],
        pose_dict["qy"],
        pose_dict["qz"],
        pose_dict["qw"],
    )

    # SciPy expects quaternions in [x, y, z, w] order
    rot = R.from_quat([qx, qy, qz, qw]) # standard is scalar last

    T = np.eye(4)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = [px, py, pz]
    return T


def convert_basalt_to_kalibr(basalt_data):
    # Support root key "value0" if present in Basalt JSON
    data = basalt_data.get("value0", basalt_data)

    T_imu_cam_list = data.get("T_imu_cam", [])
    intrinsics_list = data.get("intrinsics", [])
    resolution_list = data.get("resolution", [])
    cam_time_offset_ns = data.get("cam_time_offset_ns", 0)
    timeshift_sec = float(cam_time_offset_ns) / 1e9

    num_cams = len(intrinsics_list)
    T_cam_imu_mats = []
    T_imu_cam_mats = []

    for i in range(num_cams):
        T_imu_c = pose_to_matrix(T_imu_cam_list[i])
        # Invert to get T_cam_imu for Kalibr
        T_c_imu = np.linalg.inv(T_imu_c)
        T_imu_cam_mats.append(T_imu_c)
        T_cam_imu_mats.append(T_c_imu)

    kalibr_dict = {}

    for i in range(num_cams):
        cam_key = f"cam{i}"

        intr_info = intrinsics_list[i]
        cam_type = intr_info.get("camera_type", "kb4").lower()
        intr_params = intr_info.get("intrinsics", {})

        # Model mapping logic
        if cam_type in ["kb4", "kannala-brandt", "equidistant"]:
            camera_model = "pinhole"
            distortion_model = "equidistant"
            intrinsics = [
                float(intr_params["fx"]),
                float(intr_params["fy"]),
                float(intr_params["cx"]),
                float(intr_params["cy"]),
            ]
            distortion_coeffs = [
                float(intr_params["k1"]),
                float(intr_params["k2"]),
                float(intr_params["k3"]),
                float(intr_params["k4"]),
            ]
        elif cam_type == "pinhole":
            camera_model = "pinhole"
            distortion_model = "radtan"
            intrinsics = [
                float(intr_params["fx"]),
                float(intr_params["fy"]),
                float(intr_params["cx"]),
                float(intr_params["cy"]),
            ]
            distortion_coeffs = [
                float(intr_params.get("k1", 0.0)),
                float(intr_params.get("k2", 0.0)),
                float(intr_params.get("p1", 0.0)),
                float(intr_params.get("p2", 0.0)),
            ]
        elif cam_type in ["ds", "double_sphere"]:
            camera_model = "ds"
            distortion_model = "none"
            intrinsics = [
                float(intr_params["xi"]),
                float(intr_params["alpha"]),
                float(intr_params["fx"]),
                float(intr_params["fy"]),
                float(intr_params["cx"]),
                float(intr_params["cy"]),
            ]
            distortion_coeffs = []
        elif cam_type in ["eucm", "extended_unified"]:
            camera_model = "eucm"
            distortion_model = "none"
            intrinsics = [
                float(intr_params["alpha"]),
                float(intr_params["beta"]),
                float(intr_params["fx"]),
                float(intr_params["fy"]),
                float(intr_params["cx"]),
                float(intr_params["cy"]),
            ]
            distortion_coeffs = []
        else:
            raise ValueError(f"Unsupported Basalt camera type: {cam_type}")

        res = [int(resolution_list[i][0]), int(resolution_list[i][1])]

        cam_data = {
            "camera_model": camera_model,
            "distortion_coeffs": distortion_coeffs,
            "distortion_model": distortion_model,
            "intrinsics": intrinsics,
            "resolution": res,
            "rostopic": f"/cam{i}/image_raw",
        }

        # Relative transformation T_cn_cnm1 for stereo/multi-cam
        if i > 0:
            T_cn_cnm1 = T_cam_imu_mats[i] @ T_imu_cam_mats[i - 1]
            cam_data["T_cn_cnm1"] = T_cn_cnm1.tolist()

        # Transformation w.r.t IMU
        cam_data["T_cam_imu"] = T_cam_imu_mats[i].tolist()
        cam_data["timeshift_cam_imu"] = timeshift_sec

        kalibr_dict[cam_key] = cam_data

    return kalibr_dict


def main():
    parser = argparse.ArgumentParser(
        description="Convert Basalt JSON calibration to Kalibr camchain.yaml using SciPy and PyYAML."
    )
    parser.add_argument(
        "-i", "--input", required=True, help="Path to input Basalt calibration JSON file"
    )
    parser.add_argument(
        "-o",
        "--output",
        default="camchain.yaml",
        help="Path to output Kalibr YAML file (default: camchain.yaml)",
    )

    args = parser.parse_args()

    with open(args.input, "r") as f:
        basalt_data = json.load(f)

    kalibr_dict = convert_basalt_to_kalibr(basalt_data)

    # Dump Python dict directly into YAML file using PyYAML
    with open(args.output, "w") as f:
        yaml.dump(kalibr_dict, f, sort_keys=False, default_flow_style=None)

    print(f"Successfully converted '{args.input}' -> '{args.output}'")


if __name__ == "__main__":
    main()
