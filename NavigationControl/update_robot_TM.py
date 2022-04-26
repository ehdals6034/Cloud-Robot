#1. T_node update => 지난 vertex rid 삭제
#2. T_node와 scond 비교하여 path 전달

current_command = {}  # the current command for RobotTM
command_set = {}  # full sequence of commands for RobotTM #never change
robotGoal = {}  # New: The goal of each robot
robotPose = {}  # current pose of robot

# id
for id in self.AMR_IDs:
    current_command[id] = []  # a sequence of vertices
    command_set[id] = []  # [robotTM, robotTM, robotTM, ...] #never change
    robotGoal[id] = -1  #초기에는 -1




T_node = self.T_node
#robot pose : {rid,vid}
robotTM = copy.copy(self.current_command) #진행할때마다 변함 ex)[1,2] > [2] > [3,4,5] > [4,5]
robotTM_set = copy.copy(self.command_set)



Rid_sendRobotTM = []
list = []

list = [1,2,3,4]
list[0:3]
#1. T_node update => 지난 vertex rid 삭제
#(1)split된 path 진행사항 체크
#robot pose : {rid,vid} 
#vid = [a,a]
start_idx = 0

#1.T-Node정의 => 현재로봇들이 가지는 모든 path를 T에 따라 정리
Multipath = {'r1': [1,2,3,4,5], 'r2': [3,4,5],'r3':[5,6,7,8]}

#T_node = {7:[],218:[],219:[],220:[],221:[]} #T_node : {node_num :robot_id}
#추후에 T_node ={}
#for node in path
#   T_node[node] = [] #이런식 
T_node ={}
command_set = {}



rid_list = list(Multipath.keys())
path_list = list(Multipath.values())
robot_pose = {}


for id in rid_list:
    robot_pose[id] = []

robot_pose['r1']= [1,1]
robot_pose['r2']=[3,3]
robot_pose['r3']=[5,5]
print(robot_pose)
[1,1][3,3][5,5]



#goal vertex도 존


for rid, vid in robot_pose.items():
    robotPose[rid] = vid  #들어온 vetex로
    robotTM_check = {"current": False, "skip": False}
    
    if robotGoal[rid] != -1: #goal이 있는 로봇에대해서만 수행  
        if robotTM[rid] != []: #currnet_command를 가지고 있다
            #들어온 robot pose check
            #ex)[1,1][2,2]
            if (vid[0] == robotTM[rid][0]) and (vid[1] == robotTM[rid][0]): #한번 이동했을 때
                robotTM_check["current"] = True 
            elif (vid[0] == vid[1]): #몇번째로 앞에 들어왔는지 확인하고 skip
                if vid[0] in robotTM[rid]: #T_node에서 pop할 list 생성
                    vidx = robotTM[rid].index(vid[0]) #0부터시작
                    vidx = vidx + 1
                    list.append(robotTM[rid][0:vidx])
                    robotTM_check["skip"] = True

            #ex)[1,2][2,3]
            elif (vid[0] < vid[1]):
                if vid[0] == robotTM[rid][0]:
                    robotTM_check["current"] = True 
                    T_node[robotTM[rid][0]].pop(0)

                elif vid[0] != robotTM[rid][0]:
                    if vid[0] in robotTM[rid]:
                        vidx = robotTM[rid].index(vid[0]) #0부터시작
                        vidx = vidx + 1
                        list.append(robotTM[rid][0:vidx])
                        for i in list:
                            T_node[i].pop(0)
                        robotTM_check["skip"] = True
            
            #ex)[2,1][3,2]
            elif (vid[0] > vid[1]):
                if vid[1] == robotTM[rid][0]:    
                    robotTM_check["current"] = True 
                    T_node[robotTM[rid][0]].pop(0)
            
                elif vid[1] != robotTM[rid][0]:
                    if vid[1] in robotTM[rid]:
                        vidx = robotTM[rid].index(vid[1])
                        vidx = vidx + 1
                        list.append(robotTM[rid][0:vidx])
                        for i in list:
                            T_node[i].pop(0)
                        robotTM_check["skip"] = True
                    
                    

            # elif (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]): #완료
            #             self.Flag_terminate[rid] = 0
            #             self.robotGoal[rid] = -1

        
        
        
        #goal 완료처리 하는 부분
        #idx를 이용하여 완료체크            
        if robotTM_set[rid] != []: #goal을 가진 상태(
            if (len(robotTM_set[rid]) > 0) and (len(robotTM_set[rid][0] > 0)):  
                #goal 완료하는 조건문 
                if len(T_node) == 0: #완료처리가 됐는지 비교
                    self.Flag_terminate[rid] = 0  #종료되면 0 / 안되면 -1
                    self.robotGoal[rid] = -1  #종료되면 -1 / 안되면 vertex

        else: #goal이 없는 상태
            self.Flag_terminate[rid] = 0
            self.robotGoal[rid] = -1


        #지난 vertex삭제 / T_node의 진행된 path rid 제거 / path 전달
        if self.Flag_terminate[rid] == -1: #goal까지 안간 set에 대해서
                if robotTM_set[rid] != []: #goal을 가지고 있다
                    if robotTM[rid] != []: #currnet_command를 가지고 있다
                        if robotTM_check["current"]:
                            temp_path = robotTM[rid]
                            del temp_path[0] 
                            print("Current", rid, temp_path)
                            #새로운 Current command
                            self.current_command[rid] = copy.copy(temp_path)

                        elif robotTM_check["skip"]:
                            temp_path = robotTM[rid]
                            vidx = temp_path.index(vid[0])
                            #skip한거까지 찾아서 제거
                            del temp_path[:vidx + 1] 
                            print("skip", rid, temp_path)
                            for i in list:
                                T_node[i].pop(0)
                            self.current_command[rid] = copy.copy(temp_path)


                    #2.T_node와의 start_condition을 만족할 경우 path 전달
                    else: #currnet_command를 가지고 있지x [][]
                        start_idx = start_idx + 1
                        robotTM[rid] = copy.copy(robotTM_set[rid][start_idx])
                        if rid == T_node[robotTM[rid][0]]:
                        #그 경우 path를 넘겨줌 
                            Rid_sendRobotTM.append(rid)
                        #그 다음 path가 robot_TM
                            print(Rid_sendRobotTM)

if (vid[0] == robotTM[rid][0]) and (vid[1] == robotTM[rid][0]): #한번 이동했을 때
    robotTM_check["current"] = True 
elif (vid[0] == vid[1]): #몇번째로 앞에 들어왔는지 확인하고 skip
    if vid[0] in robotTM[rid]: #T_node에서 pop할 list 생성
        vidx = robotTM[rid].index(vid[0]) #0부터시작
        vidx = vidx + 1
        list.append(robotTM[rid][0:vidx])
        robotTM_check["skip"] = True

elif (vid[0] < vid[1]):#[1,2][2,3]
    if vid[0] == robotTM[rid][0]:
        robotTM_check["current"] = True 
        T_node[robotTM[rid][0]].pop(0)
    
    elif vid[0] != robotTM[rid][0]:
        if vid[0] in robotTM[rid]:
            vidx = robotTM[rid].index(vid[0]) #0부터시작
            vidx = vidx + 1
            list.append(robotTM[rid][0:vidx])
            for i in list:
                T_node[i].pop(0)
            robotTM_check["skip"] = True
            

elif (vid[0] > vid[1]):#[2,1][3,2]
    if vid[1] == robotTM[rid][0]:    
        robotTM_check["current"] = True 
        T_node[robotTM[rid][0]].pop(0)

    elif vid[1] != robotTM[rid][0]:
        if vid[1] in robotTM[rid]:
            vidx = robotTM[rid].index(vid[1])
            vidx = vidx + 1
            list.append(robotTM[rid][0:vidx])
            for i in list:
                T_node[i].pop(0)
            robotTM_check["skip"] = True
            