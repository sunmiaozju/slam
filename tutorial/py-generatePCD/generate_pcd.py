# -*- coding: utf-8 -*-
import numpy as np
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

DATA_PATH = '/home/sunmiao/work/data/slam/data'
scene_points_list = []
semantic_labels_list = []


def load_data():
    global scene_points_list, semantic_labels_list
    assert os.path.exists(DATA_PATH), 'train_data not found !!!'
    data_filename = os.path.join(DATA_PATH, 'scannet_test.pickle')
    with open(data_filename, 'rb') as fp:
        scene_points_list = pickle.load(fp)  # xyz坐标 一共312个点云，每一个点云大约13万个点
        semantic_labels_list = pickle.load(fp)  # 标签 0-20个类别  每一个点云大约13万个点 test数据集一共312个点云


def write_pcd(count, x1, y1, z1, filenum):
    output_filename = '{prefix}.pcd'.format(prefix='visual' + str(filenum))
    output = open(output_filename, "w+")

    list = ['# .PCD v.5 - Point Cloud Data file format\n', 'VERSION .5\n', 'FIELDS x y z\n', 'SIZE 4 4 4\n',
            'TYPE F F F\n', 'COUNT 1 1 1\n']
    output.writelines(list)
    output.write('WIDTH ')  # 注意后边有空格
    output.write(str(count))
    output.write('\nHEIGHT ')
    output.write(str(1))  # 强制类型转换，文件的输入只能是str格式
    output.write('\nPOINTS ')
    output.write(str(count))
    output.write('\nDATA ascii\n')
    for index, i in enumerate(x1):
        output.write(str(x1[index]) + ' ' + str(y1[index]) + ' ' + str(z1[index]) + '\n')
    output.close()


item_num = 19
x1 = []
y1 = []
z1 = []
count = 0
load_data()
unique_label = np.unique(semantic_labels_list[item_num])
for label in unique_label:
    for index, point_label in enumerate(semantic_labels_list[0]):
        if point_label == label:
            count += 1
            x1.append(scene_points_list[0][index][0])
            y1.append(scene_points_list[0][index][1])
            z1.append(scene_points_list[0][index][2])
    write_pcd(count, x1, y1, z1, label)
    count = 0
    x1 = []
    y1 = []
    z1 = []
