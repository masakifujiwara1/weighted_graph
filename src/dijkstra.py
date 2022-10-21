import numpy as np
import yaml
import pprint

with open('write_points.yaml', 'r') as yml:
    config_points = yaml.safe_load(yml)

with open('write_line.yaml', 'r') as yml:
    config_line = yaml.safe_load(yml)

# print(config_points)

# nodeの数
# print(len(config_points['make_points']))

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
for i in range(len(config_line['make_line'])):
    # print(config_line['make_line'][i]['point1']['point_name'])
    # print(i)
    # print(edge.get(str(config_line['make_line'][i]['point1']['point_name'])))

    if (edge.get(str(config_line['make_line'][i]['point1']['point_name'])) != None) and (edge.get(str(config_line['make_line'][i]['point2']['point_name'])) != None):
        edge[str(config_line['make_line'][i]['point1']['point_name'])][str(config_line['make_line'][i]['point2']['point_name'])] = float(config_line['make_line'][i]['distance'])
        edge[str(config_line['make_line'][i]['point2']['point_name'])][str(config_line['make_line'][i]['point1']['point_name'])] = float(config_line['make_line'][i]['distance'])

    elif edge.get(str(config_line['make_line'][i]['point1']['point_name'])) != None:
            # print("exit!!")
        edge[str(config_line['make_line'][i]['point1']['point_name'])][str(config_line['make_line'][i]['point2']['point_name'])] = float(config_line['make_line'][i]['distance'])
        edge[str(config_line['make_line'][i]['point2']['point_name'])] = {str(config_line['make_line'][i]['point1']['point_name']) : float(config_line['make_line'][i]['distance'])}
        
    elif edge.get(str(config_line['make_line'][i]['point2']['point_name'])) != None:
        edge[str(config_line['make_line'][i]['point2']['point_name'])][str(config_line['make_line'][i]['point1']['point_name'])] = float(config_line['make_line'][i]['distance'])
        edge[str(config_line['make_line'][i]['point1']['point_name'])] = {str(config_line['make_line'][i]['point2']['point_name']) : float(config_line['make_line'][i]['distance'])}

    else:
        edge[str(config_line['make_line'][i]['point1']['point_name'])] = {str(config_line['make_line'][i]['point2']['point_name']) : float(config_line['make_line'][i]['distance'])}
        edge[str(config_line['make_line'][i]['point2']['point_name'])] = {str(config_line['make_line'][i]['point1']['point_name']) : float(config_line['make_line'][i]['distance'])}

pprint.pprint(node_l)
# pprint.pprint(edge)

# print(type(edge))

edge = dict(sorted(edge.items()))
# edge2 = sorted(edge2.items())
# edge2 = dict(sorted(edge2.items()))

# 辞書第2階層のソート
for k, v in edge.items():
    # print(k, v)
    if k == '0':
        # print("success")
        edge_ = {k : dict(sorted(v.items()))}
    else:
        edge_[k] = dict(sorted(v.items()))

# print(edge2)
# print(type(edge2))
pprint.pprint(edge_, sort_dicts=False)