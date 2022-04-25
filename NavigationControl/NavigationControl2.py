import copy
import itertools
import time

class NavigationControl2:
    def __init__(self, AMR_IDs, AMR_LIFT_IDs, AMR_TOW_IDs):
        # initialize
        self.AMR_IDs = AMR_IDs
        self.AMR_TOW_IDs = AMR_TOW_IDs
        self.AMR_LIFT_IDs = AMR_LIFT_IDs

        
        # command for RobotTM
        self.current_command = {}  # the current command for RobotTM
        self.command_set = {}  # full sequence of commands for RobotTM #never change
        self.T_node = {}
        self.T_command = {} # the current command for T_node
        self.start_idx = {} # the idx for new command
        self.robotStart = {} #mapf에게 전달
        self.robotGoal = {}  # New: The goal of each robot
        self.subGoal = {} #MAPF에게 받은 path에 대한 goal
        self.robotPose = {}  # current pose of robot
    
        
        # id
        for id in self.AMR_IDs:
            self.current_command[id] = []  # a sequence of vertices
            self.command_set[id] = []  # [robotTM, robotTM, robotTM, ...] #never change
            self.T_command[id] = [] # a sequence of vertices
            self.robotGoal[id] = -1  # New: The goal of each robot 있으면 vertex
            self.subGoal[id] = -1  # 초기에는 -1 완료되면 0으로 바뀜
            self.start_idx[id] = 0  # num of split path from 0
    

    def allocate_goal(self, goals, robot_pose):  # execute when the robot TM allocates a goal to each robot
        # goals: {robot_id: goal vertex, ....}, robot_pose = {id, [vertex, vertex]}
        Rid_robotTM = []  # the list of robot ids that has an updated robotTM
        Rid_replan = []  # the list of robot ids for replanning/ replaning을 위한 id list
        flag_tow = False
        flag_lift = False
        # check if robot_ids are lifts or tows
        for rid in goals.keys():  #goal.keys => robot_id
            if rid in self.AMR_TOW_IDs:
                flag_tow = True
            if rid in self.AMR_LIFT_IDs:
                flag_lift = True
        check_ids = []
        #check_ids에 TOW,LIFT로봇의 id추가
        if flag_tow: check_ids.extend(self.AMR_TOW_IDs) 
        if flag_lift: check_ids.extend(self.AMR_LIFT_IDs)

        for rid in check_ids:#replan이 될 필요가있는 로봇을 찾거나 새로운 robotTM을 찾음 
             #만약 goal을 가지고 있다면
            if self.robotGoal[rid] != -1:  # the robot has a navigation job => initialize
                Rid_replan.append(rid)  # require replanning 
                if rid in goals.keys():  # got new job 
                    self.robotGoal[rid] = goals[rid] #새로운 goal을 줌
                #robotTM이 멈추기 위해 update함
                #진행하는 plan까지 하고 멈춰 ex)[0,1,2]=> [0]
                if self.current_command[rid] != []:  # the robot is executing the plan # TODO: test
                    self.current_command[rid] = [self.current_command[rid][0]]  #[1]
                    Rid_robotTM.append(rid)
                    self.robotStart[rid] = self.current_command[rid][0] #gl 형태로 보낼때 현재 pose 사용
                    self.command_set[rid] = [self.current_command[rid]]  # [robotTM, robotTM, robotTM, ...]
                  
                #plan을 수행하지 않으면 그 자리에 멈춰라함
                #robotTM을 수행하지 않음
                else:  #the robot is not executing plan
                    self.robotStart[rid] = robot_pose[rid][0]  # start point: the current vertex
                    self.current_command[rid] = []
                    self.command_set[rid] = []
              
            else:  # the robot does not have any job #goal을 가지고 있지 않다면
                if rid in goals.keys():  # get a new job
                    Rid_replan.append(rid)
                    self.robotStart[rid] = robot_pose[rid][0] #start position은 현재 position
                    self.robotGoal[rid] = goals[rid] #새로운 goal을 줌

        return Rid_replan, Rid_robotTM #replan이 필요한것들을 MAPF에게 넘겨줍니다./goal + currnet command를 가지는 로봇







    # def get_multipath_plan(self, multipaths): 
    #     # initialize T_node
    #     self.T_node = {}
    #     # initialize command for RobotTM
    #     for id in multipaths.keys():
    #         #이거 한번 체크
    #         if len(multipaths[id]) == 1: #움직임의 요청이 들어오지 않았다? 
    #             stationary_check = (self.robotPose[id][0] == self.robotPose[id][1] == multipaths[id][0])
    #         else: #움직임의 요청이 들어왔다
    #             stationary_check = False

    #         if not stationary_check:
    #             self.current_command[id] = []  # a sequence of vertices 
    #             self.command_set[id] = []  # [robotTM, robotTM, robotTM, ...]
    #             self.start_idx[id] = 0  # start condition of robotTM
    #             self.subGoal[id] = -1
    
    #     # initialize T_node
    #     #새로운 path가 들어왔을 때 T_node 초기화
    #     #1.중복 path 제거
    #     path_list = list(multipaths.values())
    #     rid_list = list(multipaths.keys())
    #     for i in range(len(path_list)):
    #         path_a = path_list[i]
    #         path_b = itertools.groupby(path_a)
    #         path_list[i] = [k for k, v in path_b]

    #     #2. T_node에 key 추가
    #     for i in range(len(multipaths.keys())):
    #         for j in range(len(path_list[i])):
    #             self.T_node[path_list[i][j]] = []


    #     #3. T-Node에 로봇id 저장
    #     # path길이를 비교하고 가장 긴 path의 크기(l_len)를 가져오자
    #     for i in range(0, len(path_list)):
    #         l_len = len(path_list[0])
    #         if l_len < len(path_list[i]):
    #             l_len = len(path_list[i])

    #     #print('l_len:', l_len)


    #     for t in range(l_len):  # 0~7
    #         for rid in range(0, len(rid_list)):  # 0~2
    #             try:
    #                 self.T_node[path_list[rid][t]].append(rid_list[rid])
    #             except:
    #                 pass
    #     print('T_node:', self.T_node)

    #     # 서로다른 로봇이 있는 노드를 가져옴 => 이 노드 기준으로 split
    #     check_node1 = []
    #     check_node = []
    #     for node in self.T_node.keys():
    #         a = self.T_node[node][0]
    #         for i in range(1, len(self.T_node[node])):
    #             if a != self.T_node[node][i]:
    #                 check_node1.append(node)

    #     for node in check_node1:
    #         if node not in check_node:
    #             check_node.append(node)


    #     print('check_node:', check_node)

    #     # path split
    #     for rid, path in multipaths.items():
    #         a = []
    #         if path != []:
    #             b = [path[0]]  # 0번째 원소 추가
    #             for i in range(1, len(path)):  # 1번째 원소부터 비교하며 추가
    #                 if path[i] in check_node and path[i] != b[0]:  # check_node에 있는 노드라면
    #                     a.append(b)
    #                     b = [path[i]]

    #                 else:
    #                     if path[i] != b[-1]:  # 마지막 원소가 아니다
    #                         b.append(path[i])

    #             a.append(b)
    #         self.command_set[rid] = copy.copy(a)

    #     #마지막 element가 다음 첫번째로 들어갈 수 있도록
    #     for current_robot_key in self.command_set.keys(): #로봇에 대해서
    #                 current_robot_command = copy.copy(self.command_set[current_robot_key])
    #                 for path_i in range(len(current_robot_command)):
    #                     if len(current_robot_command) > path_i > 0:
    #                         #마지막 element가 다음 처번째로 들어갈 수 있도록
    #                         last_element = current_robot_command[path_i-1][-1] #하나 빼주고
    #                         first_element = current_robot_command[path_i][0] #하나 더함
    #                         if last_element != first_element: #다를 경우에만 넣어줌
    #                             current_robot_command[path_i].insert(0, last_element) #다르면 추가
    #                     #robot_command[current_robot_key].append(current_robot_command[path_i])
        
    #     #print('robot_command:',robot_command)
    #     print('command_set:', self.command_set)

    #     for id in multipaths.keys():
    #         self.current_command[id] = copy.copy(self.command_set[id][self.start_idx[id]]) #copy로 해야 reference가 안됨
    #         self.T_command[id] = copy.copy(self.command_set[id][self.start_idx[id]])
            
    #     print('current_command', self.current_command)
    def get_multipath_plan(self, multipaths):
        self.T_node = {}
        not_overlap_path = []
        check_node = []
        check_node1 = []
        path_list = list(multipaths.values())
        rid_list = list(multipaths.keys())
        # initialize command for RobotTM
        for id in multipaths.keys():
            if len(multipaths[id]) == 1:
                stationary_check = (self.robotPose[id][0] == self.robotPose[id][1] == multipaths[id][0])
            else: # control 요청
                stationary_check = False
            if not stationary_check:
                self.current_command[id] = []  # a sequence of vertices
                self.command_set[id] = []  # [robotTM, robotTM, robotTM, ...]
                self.start_idx[id] = 0  # start condition of robotTM
                self.subGoal[id] = -1
        # 중복 제거한 path를 not_overlap_path에 대입 -> T_node 추가
        for rid in range(len(rid_list)):
            not_overlap_path.append([])
            for path in range(len(path_list[rid])):
                try:
                    if path_list[rid][path] not in not_overlap_path[rid]:
                        not_overlap_path[rid].append(path_list[rid][path])
                        self.T_node[not_overlap_path[rid][path]] = []
                except:
                    pass
        print('not_overlap_path :', not_overlap_path)
        # T_node에 robot_id 대입
        for path in range(len(not_overlap_path[rid])):
            for rid in range(len(rid_list)):
                try:
                    self.T_node[not_overlap_path[rid][path]].append(rid_list[rid])
                except:
                    pass
        print('T_node :', self.T_node)
        # 여러 대의 robot_id를 가지고 있는 node 저장 => 이 node 기준으로 split
        for node in self.T_node.keys():
            a = self.T_node[node][0]
            for i in range(1, len(self.T_node[node])):
                if a != self.T_node[node][i]:
                    check_node1.append(node)
        for node in check_node1:
            if node not in check_node:
                check_node.append(node)
        print('check_node:', check_node)
        # path split
        for rid, path in multipaths.items():
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
            self.command_set[rid] = copy.copy(a)
        # 마지막 element를 다음 path 첫 번째에 대입
        for rid in self.command_set.keys(): #로봇에 대해서
            command_set = copy.copy(self.command_set[rid])
            for path in range(len(command_set)):
                if len(command_set) > path > 0:
                    last_element = command_set[path-1][-1] # 맨 뒤 path (ex. [1, 2, 3] -> 3)
                    command_set[path].insert(0, last_element) # 맨 뒤 path를 다음 path의 맨 앞에 추가 (ex. [1][2] -> [1][1, 2])
        print('command_set:', self.command_set)
        for id in multipaths.keys():
            self.current_command[id] = copy.copy(self.command_set[id][self.start_idx[id]]) #copy로 해야 reference가 안됨
            self.T_command[id] = copy.copy(self.command_set[id][self.start_idx[id]])
        print('current_command', self.current_command)

    #추후에 많은 수정이 필요
    #skip된 robot_pose는 고려되지 않은 사항
    def update_T_node(self, robotPose):     
        T_command = copy.copy(self.T_command) #T_node update시 참고할 current_command
        for rid, pose_info in robotPose.items():
            if self.robotGoal[rid] != -1:        
                vid = pose_info[0]
                robotPose[rid] = vid
                if len(T_command[rid]) >= 2:
                    if T_command[rid][0] < T_command[rid][1]:
                        if (vid[0] == vid[1]):
                            if(T_command[rid][0] != vid[0]): #첫번째로 들어온게 아니다[123] => [22]
                                vidx = T_command[rid].index(vid[0])  # 0부터시작 => 1
                                list = T_command[rid][0:vidx] #[0:1] => [1]
                                for i in list:
                                    self.T_node[i].pop(0)  #T_node update
                                del T_command[rid][:vidx] #[123] => [23]
                                self.T_command[rid] = T_command[rid]
                        # ex)[1,2][2,3] 
                        elif (vid[0] < vid[1]): #[1,2]              
                            if vid[1] in T_command[rid]:#[1,2] [123] #큰 원소로 비교
                                vidx = T_command[rid].index(vid[1])  #위와 동일
                                list = T_command[rid][0:vidx]
                                for i in list:
                                    self.T_node[i].pop(0)
                                del T_command[rid][:vidx]
                                self.T_command[rid] = T_command[rid]
                        # ex)[2,1][3,2] [1 2] [11][12][21]  [11][21] [12][21] [1 2 3] => [1,2], [3,2]
                        # 이때 T_node update
                        elif (vid[0] > vid[1]): #[21] [123] // 이전에 [12]가 들어오면 T_command의 [1]이 없기 때문
                            if vid[0] in T_command[rid]:
                                vidx = T_command[rid].index(vid[0])  # 위와 동일
                                list = T_command[rid][0:vidx]
                                for i in list:
                                    self.T_node[i].pop(0)
                                del T_command[rid][:vidx]
                                self.T_command[rid] = T_command[rid]
                    #[54321]
                    elif T_command[rid][0] > T_command[rid][1]:
                        if (vid[0] == vid[1]): #[5,5]
                            if(T_command[rid][0] != vid[0]): #첫번째로 들어온게 아니다[123] => [22]
                                vidx = T_command[rid].index(vid[0])  # 0부터시작 => 1
                                list = T_command[rid][0:vidx] #[0:1] => [1]
                                for i in list:
                                    self.T_node[i].pop(0)  #T_node update
                                del T_command[rid][:vidx] #[123] => [23]
                                self.T_command[rid] = T_command[rid]
                        
                        elif (vid[0] < vid[1]): #[4,5]       
                            if vid[1] in T_command[rid]:#[543] #큰 원소로 비교
                                vidx = T_command[rid].index(vid[0])  #위와 동일
                                list = T_command[rid][0:vidx]
                                for i in list:
                                    self.T_node[i].pop(0)
                                del T_command[rid][:vidx]
                                self.T_command[rid] = T_command[rid]
                        # ex)[2,1][3,2] [1 2] [11][12][21]  [11][21] [12][21] [1 2 3] => [1,2], [3,2]
                        # 이때 T_node update
                        elif (vid[0] > vid[1]): #[5,4]
                            if vid[0] in T_command[rid]:#[543]
                                vidx = T_command[rid].index(vid[1])  # 위와 동일
                                list = T_command[rid][0:vidx]
                                for i in list:
                                    self.T_node[i].pop(0)
                                del T_command[rid][:vidx]
                                self.T_command[rid] = T_command[rid]


                        
    
        print('T_node:',self.T_node)
        print('T_command:',self.T_command)
    # def update_T_node(self, robotPose):     
    #     T_command = copy.copy(self.T_command) #T_node update시 참고할 current_command
    #     for rid, pose_info in robotPose.items():
    #         if self.robotGoal[rid] != -1:        
    #             vid = pose_info[0]
    #             robotPose[rid] = vid
    #             if self.subGoal[rid] == -1:
    #                 if T_command[rid] != []:
    #                     # ex)[1,1][2,2]
    #                     if (vid[0] == vid[1]):
    #                         if(T_command[rid][0] != vid[0]): #첫번째로 들어온게 아니다[123] => [22]
    #                             vidx = T_command[rid].index(vid[0])  # 0부터시작 => 1
    #                             list = T_command[rid][0:vidx] #[0:1] => [1]
    #                             for i in list:
    #                                 self.T_node[i].pop(0)  #T_node update
    #                             del T_command[rid][:vidx] #[123] => [23]
    #                             self.T_command[rid] = T_command[rid]
    #                     # ex)[1,2][2,3] 
    #                         if vid[0] == T_command[rid][0]:#T_command: [123] #큰 원소로 비교
    #                             #print(T_command[rid],123123)
    #                             vidx = T_command[rid].index(vid[1])  #위와 동일
    #                             list = T_command[rid][0:vidx]
    #                             #print(list,1111111111)
    #                             for i in list:
    #                                 self.T_node[i].pop(0)
    #                             del T_command[rid][:vidx]
    #                             self.T_command[rid] = T_command[rid]

    #                         if vid[1] == T_command[rid][0]:#T_command : [210]
    #                             vidx = T_command[rid].index(vid[0])  
    #                             list = T_command[rid][0:vidx]
    #                             for i in list:
    #                                 self.T_node[i].pop(0)
    #                             del T_command[rid][:vidx]
    #                             self.T_command[rid] = T_command[rid]

    #                     # ex)[2,1][3,2] [1 2] [11][12][21]  [11][21] [12][21] [1 2 3] => [1,2], [3,2]
    #                     # 이때 T_node update
    #                     elif (vid[0] > vid[1]): #[21] [123] // 이전에 [12]가 들어오면 T_command의 [1]이 없기 때문
    #                         if vid[1] == T_command[rid][0]: #
    #                             vidx = T_command[rid].index(vid[0])  # 위와 동일
    #                             list = T_command[rid][0:vidx]
    #                             for i in list:
    #                                 self.T_node[i].pop(0)
    #                             del T_command[rid][:vidx]
    #                             self.T_command[rid] = T_command[rid]

    #                         if vid[0] == T_command[rid][0]: #[21] [210]
    #                             vidx = T_command[rid].index(vid[1])  # 위와 동일
    #                             list = T_command[rid][0:vidx]
    #                             for i in list:
    #                                 self.T_node[i].pop(0)
    #                             del T_command[rid][:vidx]
    #                             self.T_command[rid] = T_command[rid]

    #     print('T_node:',self.T_node)
    #     print('T_command:',self.T_command)
        
                        


 
    # =>update_robot_command? 
    #안에 사용되는 변수 이름은 그대로
    #temp_path이런거 변경 혹은 유지?
    def update_robot_command(self, robotPose): 
        Rid_sendRobotTM = [] #robotTM에게 보낼 id
        # 진행할때마다 변함 ex)[1,2] > [2] > [3,4,5] > [4,5]
        robotTM = copy.copy(self.current_command)
        robotTM_set = copy.copy(self.command_set)  # 현재 가지고있는 path(변하지않음)
        for rid, pose_info in robotPose.items(): #ex)['rid',[1,2],1]]
            if self.robotGoal[rid] != -1:
                vid = pose_info[0]
                check_num = pose_info[1]
                robotPose[rid] = vid  # 들어온 vetex로
                robotTM_check = {"current": False, "skip": False}
                if self.subGoal[rid] == -1:
                    if robotTM[rid] != []:
                        # 들어온 robot pose check
                        # ex)[1,1][2,2]
                        if (vid[0] == robotTM[rid][0]) and (vid[1] == robotTM[rid][0]):  # 한번 이동했을 때            
                            robotTM_check["current"] = True
                        elif (vid[0] == vid[1]):  # 몇번째로 앞에 들어왔는지 확인하고 skip
                            if vid[0] in robotTM[rid]:  # T_node에서 pop할 list 생성
                                vidx = robotTM[rid].index(vid[0])  # 0부터시작[2,2]
                                list = robotTM[rid][0:vidx] #[0 1]
                                vidx = vidx + 1 #뒤에 robotTM update하기 위해
                                robotTM_check["skip"] = True

                        # ex)[1,2][2,3] vid[] / T_node / robotTM
                        # 이때 T_node update #robot_pose[i-1][1,1] robot_pose[i][4,3][3,4]
                        elif (vid[0] < vid[1]):
                            if vid[0] == robotTM[rid][0]: #vid[0]을 기준으로 비교
                                robotTM_check["current"] = True

                            elif vid[0] != robotTM[rid][0]:   
                                if vid[0] in robotTM[rid]:
                                    vidx = robotTM[rid].index(vid[0])  # 0부터시작
                                    vidx = vidx + 1
                                    robotTM_check["skip"] = True


                        # ex)[2,1][3,2] [1 2] [11][12][21]  [11][21] [12][21] [1 2 3] => [1,2], [3,2]
                        # 이때 T_node update
                        elif (vid[0] > vid[1]):
                            if vid[1] == robotTM[rid][0]: #vid[1]을 기준으로 비교
                                robotTM_check["current"] = True

                            elif vid[1] != robotTM[rid][0]:
                                if vid[1] in robotTM[rid]:
                                    vidx = robotTM[rid].index(vid[1])
                                    vidx = vidx + 1
                                    robotTM_check["skip"] = True

                    # 지난 vertex삭제 / path 전달
                    # if subGoal[rid] == -1:  # subGoal까지 안간 set에 대해서
                    if robotTM_set[rid] != []:  # goal을 가지고 있다
                        if robotTM[rid] != []:  # currnet_command를 가지고 있다 / current_command를 가지고 있지 않으면 수행을 안한거다 => robot_TM update할 필요가 없다.
                            if robotTM_check["current"]:
                                temp_path = robotTM[rid]
                                del temp_path[0]
                                #print("Current", rid, temp_path)
                                # 새로운 Current command
                                self.current_command[rid] = copy.copy(temp_path)
                            elif robotTM_check["skip"]:
                                temp_path = robotTM[rid]
                                # skip한거까지 찾아서 제거
                                del temp_path[:vidx]
                                #print("skip", rid, temp_path)
                                self.current_command[rid] = copy.copy(temp_path)

                    # subGoal 완료처리 하는 부분 #종료되면 0 / 안되면 -1
                    # 입력받은 vertex의 앞 뒤 숫자가 마지막 path의 값과 같으면 finish
                    #robotTM_set['r3'][-1][-1]
                    if check_num == 1:  # subGoal을 가진 상태
                        if (vid[0] == self.robotGoal[rid]) or (vid[1] ==self.robotGoal[rid]): #두 vertex중 하나라도 같으면 goal #추후에 수정ex)1234543이경우 문제 발생 => TM의 마지막 list?
                            # goal 완료하는 조건문
                            self.subGoal[rid] = 0  # 종료되면 0 / 안되면 -1  s
                            self.robotGoal[rid] = -1
                            #print(self.subGoal[rid])
                            print(rid,'--------------------finished-------------------------')

                    #goal vertex에 도달했다고 가정하고 path 넘겨줌
                    if robotTM_set[rid] != []:  # goal을 가지고 있다[][234] T[2] 
                        if robotTM[rid] == []:
                            #if path를 처음받는 경우일 때(이전에 조건을 만족 못시켜서 진행이 안되었을 때x)
                            self.start_idx[rid] = self.start_idx[rid] + 1
                            #if 뒤에 path가 더 있을경우
                            #print(robotTM_set[rid][self.start_idx[rid]])
                            robotTM[rid] = copy.copy(robotTM_set[rid][self.start_idx[rid]])
                            #print(self.T_node,4444444)
                            if self.T_node[robotTM[rid][1]][0] == rid:
                               #print(self.T_node[robotTM[rid][1]][0],55555555)
                                # 그 경우 path를 넘겨줌
                                #print(rid)
                                Rid_sendRobotTM.append(rid)
                                self.current_command[rid] = copy.copy(robotTM[rid])
                                self.T_command[rid] = copy.copy(robotTM[rid])
                                #self.Flag[rid] = 1 추후에 사용?
                            else: #start_condition을 만족시키지 못할때
                                #print(rid,111)
                                self.start_idx[rid] = self.start_idx[rid] - 1 #start_idx원래대로
                        



        print('Rid_sendRobotTM:',Rid_sendRobotTM)
        print('current_command:',self.current_command)
        #print('T_node:',self.T_node)
        #print('Rid_sendRobotTM:',Rid_sendRobotTM )
        return Rid_sendRobotTM                 


if __name__ == "__main__":
    AMR_IDs = ['r1','r2','r3']
    navcont = NavigationControl2(AMR_IDs)
    multipaths = {'r1': [1, 2, 3, 4, 5], 'r2': [3,4,5,6], 'r3': [5,6,7,8]}
    multipaths2 = {'r1': [7,8,9,10], 'r2': [3,4,5,6], 'r3': [5,6,7,8]}
    navcont.get_multipath_plan(multipaths) #이때 바로 path전달?
    navcont.get_multipath_plan(multipaths2)
#     T=1
#     if T != 0:  # T_node가 없으면 끝나는거로
#         robot_pose = {}

#         # robot_pose = {'rp1' : [[2,2],[2,2],[2,2]],# [a,a] 2개이상
#         #             'rp2' : [[3,3],[3,3],[3,4]],
#         #             'rp3' : [[5,5],[5,5],[5,6]]
#         # }

#         # robot_pose = {'rp1' : [[2,2],[2,2],[2,3],[3,3],[4,4],[4,4]], #[123][345] [11][33] [33] [345][][
#         #             'rp2' : [[3,3],[3,4],[4,4],[4,5],[4,5],[5,5]],
#         #             'rp3' : [[5,5],[5,6],[6,7],[7,8],[8,8],[8,8]]
#         # }

#         #로봇자체에서 끝나는거 체크 robot generator에서
#         robot_pose = {'rp1' : [[1,1],[1,2],[2,1],[2,2],[2,2],[3,3],[3,3],[3,3],[4,4],[4,4],[5,5]],# [a,a] 2개이상
#                     'rp2' : [[3,3],[3,3],[3,4],[4,3],[4,4],[4,4],[4,5],[5,5],[5,5],[5,6],[6,6]],
#                     'rp3' : [[5,5],[5,5],[5,6],[6,5],[6,6],[7,7],[8,8],[8,8],[8,8],[8,8],[8,8]]
#         }      

#         navcont.get_robot_pose(robot_pose)
#         #time.sleep(0.1)


