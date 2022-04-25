# new
# 1.T-Node정의 => 현재로봇들이 가지는 모든 path를 T에 따라 정리
# 2.T-Node에 로봇id 저장
# 3 T_Node를 이용하여 path_split
# 4 split_path를 이용하여 로봇 이동

import copy
import itertools


# 1.T-Node정의 => 현재로봇들이 가지는 모든 path를 T에 따라 정리
Multipath = {'r1': [1, 2, 3, 4, 5], 'r2': [ 3, 4, 5, 6], 'r3': [5, 6, 7, 8]}

# T_node = {7:[],218:[],219:[],220:[],221:[]} #T_node : {node_num :robot_id}
# 추후에 T_node ={}
# for node in path
#   T_node[node] = [] #이런식
T_node = {}
command_set = {}
#robot_command = {}
current_command = {}
subGoal = {}
start_idx = {}

rid_list = list(Multipath.keys())
path_list = list(Multipath.values())

# 중복 제거
for i in range(len(path_list)):
    path_a = path_list[i]
    path_b = itertools.groupby(path_a)
    path_list[i] = [k for k, v in path_b]


for id in rid_list:
    command_set[id] = []
    #robot_command[id] = []
    current_command[id] = []
    start_idx[id] = 0
    subGoal[id] = -1  # 완료되면 0으로 바뀜

# T_node에 key 추가
for i in range(len(Multipath.keys())):
    for j in range(len(path_list[i])):
        T_node[path_list[i][j]] = []

print(T_node)

# for i in range(len(path_list)):
#     path_list1 = []
#     path_list1.append(path_list[i][0])
#     for j in range(len(path_list[i])):
#         if path_list[i][j] != path_list[i][j-1]:
#             path_list1.append(path_list[i][j])

#print(path_list1)


# T_node[218].append(-1)a
# T_node[219].append(2)
# T_node['node_id'].append([3])

# path길이를 비교하고 가장 긴 path를 가져오자
for i in range(0, len(path_list)):
    l_len = len(path_list[0])
    if l_len < len(path_list[i]):
        l_len = len(path_list[i])

print('l_len:', l_len)


# 2 T-Node에 로봇id 저장
for t in range(l_len):  # 0~7
    for rid in range(0, len(rid_list)):  # 0~2
        try:
            T_node[path_list[rid][t]].append(rid_list[rid])
        except:
            pass
print('T_node:', T_node)

T_node

# 3 T_Node를 이용하여 path split

# #robot_id 2개 이상인 노드를 가져옴
# check_node = []
# for node in T_node.keys():
#     if len(T_node[node]) >= 2:
#         check_node.append(node) #219,220
# print('check_node:',check_node)

# 서로다른 로봇이 있는 노드를 가져옴
check_node1 = []
check_node = []
for node in T_node.keys():
    a = T_node[node][0]
    for i in range(1, len(T_node[node])):
        if a != T_node[node][i]:
            check_node1.append(node)

for node in check_node1:
    if node not in check_node:
        check_node.append(node)


print('check_node:', check_node)


# list = []
# a = [1,2,3]
# b = [a[0]]
# a[1]
# b.append(a[1])
# len(path_list)


# path split
for rid, path in Multipath.items():
    a = []

    if path != []:
        b = [path[0]]  # 0번째 원소 추가
        for i in range(1, len(path)):  # 1번째 원소부터 비교하며 추가
            if path[i] in check_node and path[i] != b[0]:  # check_node에 있는 노드라면
                a.append(b)
                b = [path[i]]

            else:
                if path[i] != b[-1]:  # 마지막 원소가 아니다
                    b.append(path[i])

        a.append(b)
    command_set[rid] = copy.copy(a)




# robotTM_set['r3'][-1][-1]
# list = []
# list = robotTM['r1'][0:2]



#마지막 element가 다음 첫번째로 들어갈 수 있도록
for current_robot_key in command_set.keys(): #로봇에 대해서
            current_robot_command = copy.copy(command_set[current_robot_key])
            for path_i in range(len(current_robot_command)):
                if len(current_robot_command) > path_i > 0:
                    #마지막 element가 다음 처번째로 들어갈 수 있도록
                    last_element = current_robot_command[path_i-1][-1] #하나 빼주고
                    first_element = current_robot_command[path_i][0] #하나 더함
                    if last_element != first_element: #다를 경우에만 넣어줌
                        current_robot_command[path_i].insert(0, last_element) #다르면 추가
                #robot_command[current_robot_key].append(current_robot_command[path_i])

#print('robot_command:',robot_command)
print('command_set:', command_set)




for id in rid_list:
    current_command[id] = command_set[id][start_idx[id]]

print('current_command', current_command)


# 진행할때마다 변함 ex)[1,2] > [2] > [3,4,5] > [4,5]
robotTM = copy.copy(current_command)
robotTM_set = copy.copy(command_set)  # 현재 가지고있는 path(변하지않음)

print('robotTM:',robotTM)
print('robotTM_set',robotTM_set)

# # Todo
# # 랜덤하게 robot_pose를 만들어서 넘겨주기
robotPose = {}
for id in rid_list:
    robotPose[id] = []
#1 2번
robotPose['r1'] = [2,2]
robotPose['r2'] = [3,3]
robotPose['r3'] = [5,5]
print(robotPose)



# #2
# robotPose['r1'] = [3,3]
# robotPose['r2'] = [4,4]
# robotPose['r3'] = [8,8]
# print(robotPose)


# # #3
# # robotPose['r1'] = [2, 2]
# # robotPose['r2'] = []
# # robotPose['r3'] = []
# # print(robotPose)

# # #4
# # robotPose['r1'] = []
# # robotPose['r2'] = [3,4]
# # robotPose['r3'] = [7,6]
# # print(robotPose)

# # #5
# # robotPose['r1'] = [3,3]
# # robotPose['r2'] = [4,4]
# # robotPose['r3'] = [8,8]
# #print(robotPose)

# robotgoal vertex도 존재한다 가정
# subgoal은
# update할때는 command_set이용 path전달할때는 robot_command이용
# ex robotTM :[123]
#T_node는 vertex를 벗어났을 때 삭제
#robotTM은 vertex에 도착했을 때 삭제
Rid_sendRobotTM = []


for rid, vid in robotPose.items():
    robotPose[rid] = vid  # 들어온 vetex로
    robotTM_check = {"current": False, "skip": False}
    if subGoal[rid] == -1:
        if robotTM[rid] != []:
            # 들어온 robot pose check
            # ex)[1,1][2,2]
            if (vid[0] == robotTM[rid][0]) and (vid[1] == robotTM[rid][0]):  # 한번 이동했을 때
                robotTM_check["current"] = True
            # 이 경우에만 T_node udpate
            elif (vid[0] == vid[1]):  # 몇번째로 앞에 들어왔는지 확인하고 skip
                if vid[0] in robotTM[rid]:  # T_node에서 pop할 list 생성
                    vidx = robotTM[rid].index(vid[0])  # 0부터시작
                    list = robotTM[rid][0:vidx]
                    for i in list:
                        T_node[i].pop(0)  # 여기서는 해당하는 index 앞까지 pop을 진행한다
                    vidx = vidx + 1
                    robotTM_check["skip"] = True

            # ex)[1,2][2,3]
            # 이때 T_node update
            elif (vid[0] < vid[1]):
                if vid[0] == robotTM[rid][0]:
                    robotTM_check["current"] = True
                    T_node[robotTM[rid][0]].pop(0)

                elif vid[0] != robotTM[rid][0]:
                    if vid[0] in robotTM[rid]:
                        vidx = robotTM[rid].index(vid[0])  # 0부터시작
                        vidx = vidx + 1
                        list = robotTM[rid][0:vidx]
                        for i in list:
                            T_node[i].pop(0)
                        robotTM_check["skip"] = True

            # ex)[2,1][3,2]
            # 이때 T_node update
            elif (vid[0] > vid[1]):
                if vid[1] == robotTM[rid][0]:
                    robotTM_check["current"] = True
                    T_node[robotTM[rid][0]].pop(0)

                elif vid[1] != robotTM[rid][0]:
                    if vid[1] in robotTM[rid]:
                        vidx = robotTM[rid].index(vid[1])
                        vidx = vidx + 1
                        list = robotTM[rid][0:vidx]
                        for i in list:
                            T_node[i].pop(0)
                        robotTM_check["skip"] = True

        # subGoal 완료처리 하는 부분 #종료되면 0 / 안되면 -1
        # 입력받은 vertex의 앞 뒤 숫자가 마지막 path의 값과 같으면 finish
        #robotTM_set['r3'][-1][-1]
        if robotTM_set[rid] != []:  # subGoal을 가진 상태
            if (vid[0] == robotTM_set[rid][-1][-1]) and (vid[1] == robotTM_set[rid][-1][-1]):
                # goal 완료하는 조건문
                subGoal[rid] = 0  # 종료되면 0 / 안되면 -1  
                print(rid,'finished')
        else:  # subGoal이 없는 상태
            subGoal[rid] = 0   

        # 지난 vertex삭제 / path 전달
        # if subGoal[rid] == -1:  # subGoal까지 안간 set에 대해서
        if robotTM_set[rid] != []:  # goal을 가지고 있다
            if robotTM[rid] != []:  # currnet_command를 가지고 있다 / current_command를 가지고 있지 않으면 수행을 안한거다 => robot_TM update할 필요가 없다.
                if robotTM_check["current"]:
                    temp_path = robotTM[rid]
                    del temp_path[0]
                    print("Current", rid, temp_path)
                    # 새로운 Current command
                    current_command[rid] = copy.copy(temp_path)

                elif robotTM_check["skip"]:
                    temp_path = robotTM[rid]
                    # skip한거까지 찾아서 제거
                    del temp_path[:vidx]
                    print("skip", rid, temp_path)
                    current_command[rid] = copy.copy(temp_path)

            # 2.T_node와의 start_condition을 만족할 경우 path 전달
            else: # currnet_command를 가지고 있지x [][]
                start_idx[rid] = start_idx[rid] + 1
                robotTM[rid] = copy.copy(robotTM_set[rid][start_idx[rid]])
                if rid == T_node[robotTM[rid][0]]:
                    # 그 경우 path를 넘겨줌
                    Rid_sendRobotTM.append(rid)
                    # 그 다음 path가 robot_TM
                print(Rid_sendRobotTM)
print('robotTM:',robotTM)
print('T_node:',T_node)

        # #goal까지 안갔는데 완료처리 했다고 할 때
        # elif subGoal[rid] == 0:
        #     if current_command[rid] != []:
        #         if (vid[0] == current_command[rid][0]) and (vid[1] ==current_command[rid][0]):
        #                 temp_path = current_command[rid]
        #                 del temp_path[0] #지워라
        #                 current_command[rid] = temp_path









# # #split_path를 이용하여 로봇 이동(update_robot_TM참고)
# # #각 로봇의 첫번째 path는 수행
# # #그 다음 path부터는 T_node[3][0] = robot_id 수행 아니면 대기
# # #T_node[4].pop(0)

# # for rid in rid_list:
# #     for i in range(len(command_set[rid])):
# #         print(command_set[rid][i])


# # for j in range(4):
# #     for rid in rid_list:
# #         try:
# #             print(command_set[rid][j])
# #         except:
# #             pass


# # T_node[4].pop(0)
# # T_node[3]
# # T_node[4]


# # #쓰레드를 빼서 실행하도록
# # for i in range(4):
# #     for rid in rid_list: #for문으로? 3,4
# #         try:
# #             if command_set[rid][i][0] in check_node:
# #                 if rid == T_node[command_set[rid][i][0]][0]:
# #                     print(command_set[rid][i],rid)
# #                     del T_node[command_set[rid][i][0]][0]
# #                     if rid == T_node[command_set[rid][i+1][0]][0]:
# #                         print(command_set[rid][i+1],rid)
# #                         del T_node[command_set[rid][i+1][0]][0]
# #             else:
# #                 print(command_set[rid][i],rid)
# #         except:
# #             pass


# # for i in range(4):
# #     print(1)
# #     for rid in rid_list:
# #         print(2)
