#!/usr/bin/env python
import argparse
import numpy as np
import pcl

parser = argparse.ArgumentParser(description='Input Point Cloud Filename.')
parser.add_argument('-f', '--filename', type=str)
parser.add_argument('-o', '--output', type=str)
args = parser.parse_args()

cloud = pcl.load(args.filename)
print("Input Cloud Size: {}".format(cloud.size))

seg = cloud.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
seg.set_normal_distance_weight(0.1)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_max_iterations(500)
seg.set_distance_threshold(0.05)
indices, model = seg.segment()

non_plane = cloud.extract(indices, negative=True)
print("Segment Model: {}".format(non_plane.size))

pcl.save(non_plane, args.output)