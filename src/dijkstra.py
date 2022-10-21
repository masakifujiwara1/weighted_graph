import numpy as np
import yaml
import pprint

INITIAL_NODE = 3
GOAL_NODE = 6
POINT_PATH = 'write_points2.yaml'
LINE_PATH = 'write_line2.yaml'

with open(POINT_PATH, 'r') as yml:
    config_points = yaml.safe_load(yml)

with open(LINE_PATH, 'r') as yml:
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
node_l[str(INITIAL_NODE)] = 0

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
# print(len(edge))

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

prev_list = [0] * len(config_points['make_points'])

# calc route
while True:
    # 未確定ノードの中からルートが最小のノードの選択
    if len(node_l) == 1:
        min_edge = list(node_l.keys())[0]
    else:
        min_edge = min(node_l, key=node_l.get)
        # print(type(min_edge))
        # print(min_edge)

    # 探索したノード間のルートの削除
    del_list = []
    for i in edge.keys():
        for j in edge[i].keys():
            if j == min_edge:
                del_list.append(i+j)
                # print(del_list)
    for i in del_list:
        del edge[i[0]][i[1]]

    # 探索したノードのコストが以前のものより小さければ更新する
    for i,j in edge[min_edge].items():
        if (j + node_l[min_edge]) < node_l[i]:
            node_l[i] = j + node_l[min_edge]
            prev_list[int(i)] = int(min_edge) # iへnode(min_edge)を経由して到達

    # 確定したノードへのルートの削除
    node_l_[min_edge] = node_l[min_edge]
    if len(edge) == 1:
        break
    del edge[min_edge]
    del node_l[min_edge]

    print("未確定のノード ", end="")
    print(node_l)
    print("確定したノード ", end="")
    print(node_l_)
    print("-"*100)

print("確定したノード ", end="")
print(node_l_)

# print(prev_list)
# prev_list -> [0, 0, 0, 1, 3, 1]
route = [GOAL_NODE]
current_num = GOAL_NODE
# print(route)
if current_num != INITIAL_NODE:
    for i in range(len(config_points['make_points'])):
        route.append(prev_list[current_num])
        current_num = prev_list[current_num]
        if current_num == INITIAL_NODE:
            route.reverse()
            break

print("start:" + str(INITIAL_NODE), "goal:" + str(GOAL_NODE), "route:", end="")
print(route)