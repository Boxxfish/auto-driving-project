"""
Splits the dataset into an easy and hard group, based on the criteria laid out in VI-Eye.

The hard group contains point clouds where the overlap is < 50%. We don't take rotation into account. The definition of
overlap comes from Fast and Robust Registration of Partially Overlapping Point Clouds:

The overlap ratio measures the overlap between point clouds as the percentage of points in the source point cloud that,
when aligned, are within a distance smaller than γ to any point in the target point cloud.
"""

from collections import namedtuple
from itertools import islice
import numpy as np
from typing import *
import sensor_msgs.point_cloud2 as pc2
import sensor_msgs.msg as msg
from bagpy import bagreader
import pandas as pd
from pathlib import Path
import json
import open3d as o3d
from tqdm import tqdm

Field = namedtuple("Field", ["name", "offset", "datatype", "count"])


def row_to_points(row: list[dict]) -> np.ndarray:
    """
    Converts a row from the reader into a numpy array of points.
    """
    header = {
        "seq": row[1]["header.seq"],
        "stamp": {
            "secs": row[1]["header.stamp.secs"],
            "nsecs": row[1]["header.stamp.nsecs"],
        },
        "frame_id": row[1]["header.frame_id"],
    }
    height = row[1]["height"]
    width = row[1]["width"]
    fields = row[1]["fields"]
    field_dicts = []
    for field in fields[1:-1].split(","):
        field_dict = {}
        for prop in field.split("\n"):
            [key, val] = prop.strip().split(": ")
            field_dict[key] = eval(val)
        field_dicts.append(Field(**field_dict))
    is_bigendian = row[1]["is_bigendian"]
    point_step = row[1]["point_step"]
    row_step = row[1]["row_step"]
    data = eval(row[1]["data"])
    is_dense = row[1]["is_dense"]
    cloud = msg.PointCloud2(
        header,
        height,
        width,
        field_dicts,
        is_bigendian,
        point_step,
        row_step,
        data,
        is_dense,
    )
    return np.array(list(pc2.read_points(cloud)))[:, :3]


def load_points_from_bag(path: str) -> list[np.ndarray]:
    """
    Loads point clouds from a bag file.
    Each bag contains ~300 frames of point cloud data.
    """
    data = bagreader(str(path))
    pcd_csv = data.message_by_topic("/ouster/points")
    pcd_df = pd.read_csv(pcd_csv)

    all_points = []
    for row in pcd_df.iterrows():
        points = row_to_points(row)
        all_points.append(points)
    return all_points


def read_matrix(iter: Iterable[str]) -> np.ndarray:
    return np.array([[float(n) for n in row.split(" ")] for row in islice(iter, 4)])


def check_overlap(
    src_cloud: np.ndarray, target_cloud: np.ndarray, gamma: float = 1.0
) -> float:
    in_cloud = 0
    for point in src_cloud:
        dists: np.ndarray = np.sqrt(((point[np.newaxis, ...] - target_cloud)**2).sum(1)) # Shape: (num_points,)
        min_dist = dists.min()
        if min_dist < gamma:
            in_cloud += 1
    return in_cloud / len(src_cloud)


if __name__ == "__main__":
    for dataset in ["Dataset_1", "Dataset_2", "Dataset_3"]:
        for sub_ds in ["D1", "D2", "D3", "D4", "D5"]:
            data_path = Path("data/carla_ali")
            bag_path = data_path / "bag_data" / "carla"
            ds_path = bag_path / dataset
            sub_ds_path = ds_path / sub_ds
            split_path = sub_ds_path / "splits"
            if split_path.exists():
                continue
            split_path.mkdir(exist_ok=True)
            print("Processing", dataset, sub_ds)

            infra_path = sub_ds_path / "infra_rewrite.bag"
            i_w_path = sub_ds_path / "I_W.txt"
            v_path = sub_ds_path / "vehicle_rewrite.bag"
            v_w_path = sub_ds_path / "V_W.txt"

            v_data = load_points_from_bag(v_path)
            v_w_iter = open(v_w_path, "r")
            v_w_mats = []
            for _ in range(300):
                v_w_mats.append(read_matrix(v_w_iter))

            infra_data = load_points_from_bag(infra_path)
            infra_mat = read_matrix(open(i_w_path, "r"))

            easy_set = []
            hard_set = []
            for i, (v_mat, v_points, i_points) in tqdm(enumerate(zip(v_w_mats, v_data, infra_data))):
                v_points_xformed = (
                    v_mat @ np.concatenate([v_points, np.ones([len(v_points), 1])], 1).T
                ).T[:, :3]
                i_points_xformed = (
                    infra_mat @ np.concatenate([i_points, np.ones([len(i_points), 1])], 1).T
                ).T[:, :3]

                overlap = check_overlap(v_points_xformed, i_points_xformed)
                if overlap >= 0.5:
                    easy_set.append(i)
                else:
                    hard_set.append(i)

                o3d.io.write_point_cloud(str(split_path / f"{i}_v.pcd"), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(v_points)))
                o3d.io.write_point_cloud(str(split_path / f"{i}_i.pcd"), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(i_points)))

            with open(split_path / "info.json", "w") as f:
                json.dump({"easy": easy_set, "hard": hard_set}, f)
