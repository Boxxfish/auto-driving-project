"""
Visualizes an episode.
"""
import rerun as rr
from pathlib import Path
from typing import *
import open3d
from tqdm import tqdm
import numpy as np
from math import radians, sin, cos

# Transform(Location(x=-112.302399, y=2.822677, z=3.647835), Rotation(pitch=0.000000, yaw=-0.139465, roll=0.000000))
class Location:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

class Rotation:
    def __init__(self, pitch: float, yaw: float, roll: float):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

class Transform:
    def __init__(self, location: Location, rotation: Rotation):
        self.location = location
        self.rotation = rotation

    def to_xform3d(self) -> rr.Transform3D:
        return rr.Transform3D(translation=self.location.to_list(), mat3x3=self.to_mat4()[:3, :3])
    
    def to_mat4(self) -> np.ndarray:
        a = radians(-self.rotation.yaw)
        yaw = np.array([
            [cos(a), -sin(a), 0],
            [sin(a), cos(a), 0],
            [0, 0, 1],
        ])
        b = radians(-self.rotation.pitch)
        pitch = np.array([
            [cos(b), 0, sin(b)],
            [0, 1, 0],
            [-sin(b), 0, cos(b)],
        ])
        g = radians(-self.rotation.roll)
        roll = np.array([
            [1, 0, 0],
            [0, cos(g), -sin(g)],
            [0, sin(g), cos(g)],
        ])

        [x, y, z] = self.location.to_list()
        xlate = np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1],
        ])
        rot = np.zeros((4, 4))
        rot[:3, :3] = yaw @ pitch @ roll
        rot[3, 3] = 1

        return xlate @ rot

def log_pc(entity_name: str, entity_path: Path, entity_xform: Transform, frame_idx: int):
    pc_file = open3d.io.read_point_cloud(str(entity_path / "pcds" / f"{frame_idx}.pcd"))
    points = pc_file.transform(entity_xform.to_mat4()).points
    rr.log(f"{entity_name}/points", rr.Points3D(points))

def main():
    rr.init("V2I Visualization")
    rr.connect()
    data_path = Path("data/carla")
    infra_path = data_path / "infra"
    car_path = data_path / "car" 
    num_frames = 300

    # Store the positions of infra and car
    infra_xforms: Mapping[str, Transform] = {}
    car_xforms: Mapping[str, Transform] = {}
    infra_pose_f = open(infra_path / "pose.txt", "r")
    car_pose_f = open(car_path / "pose.txt", "r")
    for i, (infra_line, car_line) in enumerate(zip(infra_pose_f, car_pose_f)):
        frame_idx = i + 1
        infra_xforms[frame_idx] = eval(" ".join(infra_line.split()[1:]))
        car_xforms[frame_idx] = eval(" ".join(car_line.split()[1:]))

    for frame_idx in tqdm(range(1, num_frames + 1)):
        # Log entity positions
        infra_xform = infra_xforms[frame_idx]
        rr.log("infra/xform", infra_xform.to_xform3d())
        car_xform = car_xforms[frame_idx]
        rr.log("car/xform", car_xform.to_xform3d())

        # Log point clouds
        log_pc("infra", infra_path, infra_xform, frame_idx)
        log_pc("car", car_path, car_xform, frame_idx)


if __name__ == "__main__":
    main()