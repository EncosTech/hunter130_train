import pickle
import numpy as np
import argparse
from scipy.spatial.transform import Rotation 


class NumpyCompatibleUnpickler(pickle.Unpickler):
    """Read trusted NumPy 2 motion pickles in NumPy 1 environments as well."""

    def find_class(self, module, name):
        if module.startswith("numpy._core") and int(np.__version__.split(".")[0]) < 2:
            module = module.replace("numpy._core", "numpy.core", 1)
        return super().find_class(module, name)


def convert_pkl_to_custom(input_pkl, output_txt, fps):
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    dt = 1.0 / fps

    with open(input_pkl, "rb") as f:
        motion_data = NumpyCompatibleUnpickler(f).load()

    root_pos = motion_data["root_pos"]
    root_rot = Rotation.from_quat(motion_data["root_rot"])  # xyzw
    dof_pos = motion_data["dof_pos"]

    root_lin_vel = (root_pos[1:] - root_pos[:-1]) / dt
    # Preserve the original body-frame angular velocity convention: q(t)^-1 * q(t+dt).
    root_ang_vel = (root_rot[:-1].inv() * root_rot[1:]).as_rotvec() / dt

    dof_vel = (dof_pos[1:] - dof_pos[:-1]) / dt

    euler_angles = root_rot[:-1].as_euler('XYZ', degrees=False)
    euler_angles = np.unwrap(euler_angles, axis=0)

    data_output = np.concatenate(
        (root_pos[:-1], euler_angles, dof_pos[:-1],  
         root_lin_vel, root_ang_vel, dof_vel),
        axis=1
    )

    np.savetxt(output_txt, data_output, fmt='%f', delimiter=', ')
    with open(output_txt, 'r') as f:
        frames_data = f.readlines()

    frames_data_len = len(frames_data)
    with open(output_txt, 'w') as f:
        f.write('{\n')
        f.write('"LoopMode": "Wrap",\n')
        f.write(f'"FrameDuration": {dt!r},\n')
        f.write('"EnableCycleOffsetPosition": true,\n')
        f.write('"EnableCycleOffsetRotation": true,\n')
        f.write('"MotionWeight": 0.5,\n\n')
        f.write('"Frames":\n[\n')

        for i, line in enumerate(frames_data):
            line_start_str = '  ['
            if i == frames_data_len - 1:
                f.write(line_start_str + line.rstrip() + ']\n')
            else:
                f.write(line_start_str + line.rstrip() + '],\n')

        f.write(']\n}')
    print(f"✅ Successfully converted {input_pkl} to {output_txt}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_pkl", type=str, required=True)
    parser.add_argument("--output_txt", type=str, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    convert_pkl_to_custom(args.input_pkl, args.output_txt, args.fps)
