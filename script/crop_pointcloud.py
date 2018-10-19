#!/usr/bin/env python
import argparse
import numpy as np
import pcl

parser = argparse.ArgumentParser(description='Input Point Cloud Filename.')
parser.add_argument('-f', '--filename', type=str)
parser.add_argument('-o', '--output', type=str)
parser.add_argument('--x_min', type=float)
parser.add_argument('--x_max', type=float)
parser.add_argument('--y_min', type=float)
parser.add_argument('--y_max', type=float)
parser.add_argument('--z_min', type=float)
parser.add_argument('--z_max', type=float)

args = parser.parse_args()

cloud = pcl.load(args.filename)

# pcl::CropBox<PointXYZI> clipper;
# clipper.setInputCloud(cloud);
clipper = cloud.make_cropbox()

# pcl::PCDWriter writer;
# pcl::PointCloud<PointXYZI>::Ptr outcloud;
outcloud = pcl.PointCloud()

# clipper.setTranslation(Eigen::Vector3f(pose->tx, pose->ty, pose->tz));
# clipper.setRotation(Eigen::Vector3f(pose->rx, pose->ry, pose->rz));
# clipper.setMin(-Eigen::Vector4f(tracklet->l/2, tracklet->w/2, 0, 0));
# clipper.setMax(Eigen::Vector4f(tracklet->l/2, tracklet->w/2, tracklet->h, 0));
# clipper.filter(*outcloud);
tx = 0
ty = 0
tz = 0
clipper.set_Translation(tx, ty, tz)

rx = 0
ry = 0
rz = 0
clipper.set_Rotation(rx, ry, rz)

minx = args.x_min
miny = args.y_min
minz = args.z_min
mins = 0

maxx = args.x_max
maxy = args.y_max
maxz = args.z_max
maxs = 0

clipper.set_MinMax(minx, miny, minz, mins, maxx, maxy, maxz, maxs)
outcloud = clipper.filter()

pcl.save(outcloud, args.output)