"""
Visualizes an episode.
"""
import rerun as rr
from pathlib import Path
from typing import *
import numpy as np
from bagpy import bagreader
import pandas as pd
import sensor_msgs.point_cloud2 as pc2
import sensor_msgs.msg as msg
from collections import namedtuple
from itertools import islice

# def log_pc(entity_name: str, file_path: Path):
#     pc_file = open3d.io.read_point_cloud(str(file_path))
#     points = pc_file.points
#     rr.log(f"{entity_name}/points", rr.Points3D(points))

def log_row(entity_name: str, row, xform: np.ndarray):
    header = {
        "seq": row[1]["header.seq"],
        "stamp": { "secs": row[1]["header.stamp.secs"], "nsecs": row[1]["header.stamp.nsecs"]},
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
    cloud = msg.PointCloud2(header, height, width, field_dicts, is_bigendian, point_step, row_step, data, is_dense)
    points = np.array(list(pc2.read_points(cloud)))[:, :3]
    points = (xform @ np.concatenate([points, np.ones([len(points), 1])], 1).T).T
    rr.log(f"{entity_name}/points", rr.Points3D(points[:, :3]))

def read_matrix(iter) -> np.ndarray:
    return np.array([[float(n) for n in row.split(" ")] for row in islice(iter, 4)])

Field = namedtuple("Field", ["name", "offset", "datatype", "count"])

def main():
    rr.init("V2I Visualization")
    rr.connect()
    data_path = Path("data/carla_ali")
    bag_path = data_path / "bag_data" / "carla"
    ds_path = bag_path / "Dataset_1"
    sub_ds_path = ds_path / "D1"

    infra_path = sub_ds_path / "infra_rewrite.bag"
    i_w_path = sub_ds_path / "I_W.txt"
    car_path = sub_ds_path / "vehicle_rewrite.bag"
    v_w_path = sub_ds_path / "V_W.txt"

    car_data = bagreader(str(car_path))
    car_pcd_csv = car_data.message_by_topic("/ouster/points")
    car_pcd_df = pd.read_csv(car_pcd_csv)
    v_w_iter = open(v_w_path, "r")

    infra_data = bagreader(str(infra_path))
    infra_pcd_csv = infra_data.message_by_topic("/ouster/points")
    infra_pcd_df = pd.read_csv(infra_pcd_csv)
    infra_mat = read_matrix(open(i_w_path, "r"))

    for (infra_row, car_row) in zip(infra_pcd_df.iterrows(), car_pcd_df.iterrows()):
        rr.log("infra/xform", rr.Transform3D(translation=infra_mat[:3, 3], mat3x3=infra_mat[:3, :3]))
        car_mat = read_matrix(v_w_iter)
        rr.log("car/xform", rr.Transform3D(translation=car_mat[:3, 3], mat3x3=car_mat[:3, :3]))
        log_row("infra", infra_row, infra_mat)
        log_row("car", car_row, car_mat)


if __name__ == "__main__":
    main()