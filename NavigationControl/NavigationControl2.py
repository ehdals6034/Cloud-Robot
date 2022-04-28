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
        self.robotPose = {}  # current pose of robot
    
        
        # id
        for id in self.AMR_IDs:
            self.current_command[id] = []  # a sequence of vertices
            self.command_set[id] = []  # [robotTM, robotTM, robotTM, ...] #never change
            self.T_command[id] = [] # a sequence of vertices
            self.robotGoal[id] = -1  # New: The goal of each robot 있으면 vertex
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
        # initialize T_node
        self.T_node = {}
        max_len = 0
        # multipaths = {'LIFT_0': [205,206,207,208,208,209,209,210,211], 'LIFT_1': [223,214,5,5,208,5,5,211,214,212,211,210,5,5,5,210]}
        rid_list = list(multipaths.keys())
        path_list = list(multipaths.values())
        # initialize command for RobotTM
        for id in multipaths.keys():
            #이거 한번 체크
            if len(multipaths[id]) == 1: #움직임의 요청이 들어오지 않았다?
                stationary_check = (self.robotPose[id][0] == self.robotPose[id][1] == multipaths[id][0])
            else: #움직임의 요청이 들어왔다
                stationary_check = False
            if not stationary_check:
                self.current_command[id] = []  # a sequence of vertices
                self.command_set[id] = []  # [robotTM, robotTM, robotTM, ...]
                self.start_idx[id] = 0  # start condition of robotTM
                #self.subGoal[id] = -1
        # 연속되는 중복 path 제거
        print('path_list :', path_list)
        path_list2 = {}
        for rid in range(len(rid_list)):
            path_list2[rid] = []
            path_list2[rid].append(path_list[rid][0])
            for path in range(len(path_list[rid])-1):
                if path_list[rid][path] != path_list[rid][path+1]:
                    path_list2[rid].append(path_list[rid][path+1])
            path_list[rid] = path_list2[rid]
        print('path_list :', path_list)
        # path를 T_node에 추가
        for rid in range(len(rid_list)):
            for path in range(len(path_list[rid])):
                self.T_node[path_list[rid][path]] = []
        # 가장 긴 path 길이 찾기
        for i in range(len(path_list)):
            max_len = len(path_list[0])
            if max_len < len(path_list[i]):
                max_len = len(path_list[i])
        print('max_len :', max_len)
        # T_node에 robot_id 대입 -> path 길이가 다를 때 rid를 비교하면 빈칸이어서 오류 발생(try)
        for path in range(max_len):
            for rid in range(len(rid_list)):
                try:
                    self.T_node[path_list[rid][path]].append(rid_list[rid])
                except:
                    pass
        print('T_node :', self.T_node)
        # 여러 대의 robot_id를 가지고 있는 node 저장 => 이 node 기준으로 split
        check_node1 = []
        check_node = []
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
                        if path[i] in check_node and path[i] != b[-1]:  # check_node에 있는 노드라면
                            a.append(b)
                            b = [path[i]]
                        else:
                            b.append(path[i])
                    a.append(b)
                self.command_set[rid] = copy.copy(a)
        print('split', self.command_set)
        # 마지막 element를 다음 path 첫 번째에 넣어주기 (eg. [1,2][3] -> [1,2][2,3])
        for current_robot_key in self.command_set.keys(): #로봇에 대해서
                    current_robot_command = copy.copy(self.command_set[current_robot_key])
                    for path_i in range(len(current_robot_command)):
                        if len(current_robot_command) > path_i > 0:
                            last_element = current_robot_command[path_i-1][-1]
                            first_element = current_robot_command[path_i][0]
                            if last_element != first_element: #다를 경우에만 넣어줌
                                current_robot_command[path_i].insert(0, last_element) #다르면 추가
                        #robot_command[current_robot_key].append(current_robot_command[path_i])
        #print('robot_command:',robot_command)
        print('command_set:', self.command_set)
        for id in rid_list:
            self.current_command[id] = copy.copy(self.command_set[id][self.start_idx[id]]) #copy로 해야 reference가 안됨
            self.T_command[id] = copy.copy(self.command_set[id][self.start_idx[id]])
        print('current_command', self.current_command)
        print('\n')
        print('-------------------------------------------------------------------------------------------------')

    #추후에 많은 수정이 필요
    #suppose 1.skip된 경우 제외 2.모든 robotpose가 존재
    #update되는 타이밍이 다름
    #[12][21]
    #[321]
    #[21][12]
    #[12][12]가 pose로 들어올 경우 pop이 2번 발생 앞뒤 원소를 비교를 해야한다
    #[132][342]
    #[13][31][31][33][32][23]
    #[213]
    #[21][12][11][13][31]
    def update_T_node(self, robotPose):     
        T_command = copy.copy(self.T_command) #T_node update시 참고할 current_command
        for rid, pose_info in robotPose.items():
            if self.robotGoal[rid] != -1:    
                vid = pose_info[0]
                robotPose[rid] = vid
                command_check = {"current": False}
                if len(T_command[rid]) >= 2:
                    if T_command[rid][0] < T_command[rid][1]: #중복삭제 방지하기 위해
                        if (vid[0] > vid[1]) and (vid[1] == T_command[rid][0]):
                            command_check["current"] = True

                    if T_command[rid][0] > T_command[rid][1]:
                        if (vid[0] < vid[1]) and (vid[1] == T_command[rid][0]):
                            command_check["current"] = True


                if command_check["current"] == True:
                    node = T_command[rid][0]
                    self.T_node[node].pop(0)
                    T_command[rid].pop(0)
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
    # =>update_robot_command? 
    #안에 사용되는 변수 이름은 그대로
    #temp_path이런거 변경 혹은 유지?
    def update_robot_command(self, robotPose):
        Rid_sendRobotTM = []
        current_command = copy.copy(self.current_command)
        command_set = copy.copy(self.command_set)  # 현재 가지고있는 path(변하지않음)
        for rid, pose_info in robotPose.items():  # ex)['rid',[1,2],1]]
            if self.robotGoal[rid] != -1:
                vid = pose_info[0]
                check_num = pose_info[1]
                robotPose[rid] = vid  # 들어온 vetex로
                ##command_check = {"current": False} #아직사용x
                if current_command[rid] != []:
                    if (vid[0] == current_command[rid][0]) and (vid[1] == current_command[rid][0]):         
                        current_command[rid].pop(0)

                #goal check
                #if command_set[rid] == []:
                if check_num == 1 and len(command_set[rid]) == 1:  # subGoal을 가진 상태
                    if (vid[0] == self.robotGoal[rid]) or (vid[1] ==self.robotGoal[rid]): #두 vertex중 하나라도 같으면 goal #추후에 수정ex)1234543이경우 문제 발생 => TM의 마지막 list?
                        # goal 완료하는 조건문
                        #self.subGoal[rid] = 0  # 종료되면 0 / 안되면 -1  s
                        self.robotGoal[rid] = -1
                        #print(self.subGoal[rid])
                        print(rid,'--------------------finished-------------------------')

                #path 전달
                if self.robotGoal[rid] != -1:
                    if command_set[rid] != []:  # goal을 가지고 있다[][234] T[2] 
                        if current_command[rid] == []:
                            #if path를 처음받는 경우일 때(이전에 조건을 만족 못시켜서 진행이 안되었을 때x)
                            self.start_idx[rid] = self.start_idx[rid] + 1
                            #if 뒤에 path가 더 있을경우
                            #print(robotTM_set[rid][self.start_idx[rid]])
                            current_command[rid] = copy.copy(command_set[rid][self.start_idx[rid]])
                            #print(self.T_node,4444444)
                            if self.T_node[current_command[rid][1]][0] == rid:
                                # 그 경우 path를 넘겨줌
                                Rid_sendRobotTM.append(rid)
                                self.current_command[rid] = copy.copy(current_command[rid])
                                self.T_command[rid] = copy.copy(current_command[rid])
                                #self.Flag[rid] = 1 추후에 사용?
                            else: #start_condition을 만족시키지 못할때
                                #print(rid,111)
                                self.start_idx[rid] = self.start_idx[rid] - 1 #start_idx원래대로
        print('Rid_sendRobotTM:',Rid_sendRobotTM)
        print('current_command:',self.current_command)
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


