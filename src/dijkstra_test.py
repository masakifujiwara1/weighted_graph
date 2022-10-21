import numpy as np

node_l = {
    "S":0,
    "A":np.inf,
    "B":np.inf,
    "C":np.inf,
    "D":np.inf,
    "E":np.inf,
    "F":np.inf,
    "G":np.inf
}

# 確定したルート
node_l_ = {}

edge = {
    "S":{"A":5, "B":4, "C":1},
    "A":{"S":5, "D":2},
    "B":{"S":4, "D":5, "E":6},
    "C":{"S":1, "B":2},
    "D":{"A":2, "B":5, "F":1, "G":3},
    "E":{"B":6, "G":2},
    "F":{"D":1, "G":3},
    "G":{"D":3, "E":2, "F":4}
}

while True:
    # 未確定ノードの中からルートが最小のノードの選択
    if len(node_l) == 1:
        min_edge = list(node_l.keys())[0]
    else:
        min_edge = min(node_l, key=node_l.get)

    # 探索したノード間のルートの削除
    del_list = []
    for i in edge:
        for j in edge[i]:
            if j == min_edge:
                del_list.append(i+j)
    for i in del_list:
        del edge[i[0]][i[1]]

    # 探索したノードのコストが以前のものより小さければ更新する
    for i,j in edge[min_edge].items():
        if (j + node_l[min_edge]) < node_l[i]:
            node_l[i] = j + node_l[min_edge]

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