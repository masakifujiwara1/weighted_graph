import numpy as np
import yaml

with open('write_points.yaml', 'r') as yml:
    config_points = yaml.safe_load(yml)

with open('write_line.yaml', 'r') as yml:
    config_line = yaml.safe_load(yml)

# print(config_points)

# nodeの数
print(len(config_points['make_points']))

# 空のnode用意
node_l = {}
# node_l["s"] = int(config_points['make_points'][0]['id'])
for i in range(len(config_points['make_points'])):
    node_l[str(config_points['make_points'][i]['id'])] = np.inf

# 初期位置ノード反映(仮：node0)
node_l['0'] = 0

# 確定ルート
node_l_ = {}

# 空のedge用意
edge = {}


print(node_l)