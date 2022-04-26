multipaths = {'AMRLIFT0': [205,206,207,208,209,210,211], 'AMRLIFT1': [223,214,212,211,210,5,210]}#,'AMRTOW0': [233,234,235,237,238,239]}#'AMRTOW0':[234,235,236,237,238,239]}

T_node = {}
not_overlap_path = []
check_node = []
check_node1 = []
path_list = list(multipaths.values())
rid_list = list(multipaths.keys())


# # 중복 제거한 path를 not_overlap_path에 대입 -> T_node 추가
# for rid in range(len(rid_list)):
#     not_overlap_path.append([])
#     for path in range(len(path_list[rid])):
#         try:
#             if path_list[rid][path] not in not_overlap_path[rid]:
#                 not_overlap_path[rid].append(path_list[rid][path])
#                 T_node[not_overlap_path[rid][path]] = []
#         except:
#             pass
# print('not_overlap_path :', not_overlap_path)


# # T_node에 robot_id 대입
# for path in range(len(path_list[rid])):
#     for rid in range(len(rid_list)):
#         try:
#             T_node[not_overlap_path[rid][path]].append(rid_list[rid])
#             print(T_node[not_overlap_path[rid][path]])
#         except:
#             pass
        
# print('T_node :', T_node)


# path를 T_node에 추가
for rid in range(len(rid_list)):
    for path in range(len(path_list[rid])):
        T_node[path_list[rid][path]] = []
print('path_list :', path_list)

# T_node에 robot_id 대입
for path in range(len(path_list[rid])):
    for rid in range(len(rid_list)):
        T_node[path_list[rid][path]].append(rid_list[rid])
print('T_node :', T_node)

for node in T_node.keys():
    a = T_node[node][0]
    for i in range(1, len(T_node[node])):
        if a != T_node[node][i]:
            check_node1.append(node)

for node in check_node1:
    if node not in check_node:
        check_node.append(node)
print('check_node :', check_node)