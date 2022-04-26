import copy


class NavigationControl:
    def __init__(self, AMR_IDs, AMR_LIFT_IDs, AMR_TOW_IDs):
        # initialize
        self.AMR_IDs = AMR_IDs
        self.AMR_TOW_IDs = AMR_TOW_IDs
        self.AMR_LIFT_IDs = AMR_LIFT_IDs

        # path given by multi-robot path finder
        # self.NavPath = {}
        # self.timing = {} # execution timing
        # for id in self.AMR_IDs:
        #    self.NavPath[id] = []
        #    self.timing[id] = [] # if NavPath[rid1][idx1] == NavPath[rid2][idx2] and idx2<idx1 : timing[id]=[[rid2, time_idx2, NavPath[rid1][idx1]], ...]

        # command for RobotTM
        self.current_command = {}  # the current command for RobotTM
        self.command_set = {}  # full sequence of commands for RobotTM #never change
        self.command_start_condition = {}  # start condition of robotTM
        self.robotGoal = {}  # New: The goal of each robot
        self.robotStart = {}  # New: The goal of each robot
        self.robotPose = {}  # current pose of robot
        # id
        for id in self.AMR_IDs:
            self.current_command[id] = []  # a sequence of vertices
            self.command_set[id] = []  # [robotTM, robotTM, robotTM, ...] #never change
            self.command_start_condition[id] = []  # start condition of robotTM
            self.robotGoal[id] = -1  #초기에는 -1

        # using for update robotTM
        self.PlanExecutedIdx = {}
        self.Flag_terminate = {}  # -1: Not terminated, 0: Success Terminate 1: Fail Terminate
        # id
        for id in self.AMR_IDs:
            self.PlanExecutedIdx[id] = [-1, -1]  # [i,j] Save the last index of NavPath[i][j] which the robot follows
            self.Flag_terminate[id] = 0

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
        print(check_ids,666)
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
    
    # New
    #새로운 goal을 받았을 때 update해주는 함수
    def allocate_goal_(self, goals, robot_pose):  # execute when the robot TM allocates a goal to each robot
        # goals: {robot_id: goal vertex, ....}, robot_pose = {id, [vertex, vertex]}

        Rid_robotTM = []  # the list of robot ids that has an updated robotTM
        Rid_replan = []  # the list of robot ids for replanning
        flag_tow = False
        flag_lift = False
        # check if robot_ids are lifts or tows
        for rid in goals.keys():
            if rid in self.AMR_TOW_IDs:
                flag_tow = True
            if rid in self.AMR_LIFT_IDs:
                flag_lift = True

        check_ids = []

        if flag_tow: check_ids.extend(self.AMR_TOW_IDs)
        if flag_lift: check_ids.extend(self.AMR_LIFT_IDs)
        for rid in check_ids:
            '''
            if self.robotGoal[rid] != -1: # the robot has a navigation job => initialize
                print("here5-------------------------")
                Rid_replan.append(rid) # require replanning
                if rid in goals.keys(): # got new job
                    print("here6-------------------------")
                    self.robotGoal[rid] = goals[rid]
                if self.robotTM[rid] != []: # the robot is executing the plan # TODO: test
                    print("here7-------------------------")
                    self.robotTM[rid] = [self.robotTM[rid][0]]
                    Rid_robotTM.append(rid)
                    self.robotStart[rid] = self.robotTM[rid][0]
                    print(str(self.robotStart[rid]))
                    print(str(self.robotTM[rid]))
                    print("here8-------------------------")
                    self.robotTM_set[rid] = [self.robotTM[rid]]  # [robotTM, robotTM, robotTM, ...]
                    self.PlanExecutedIdx[rid] = [0, -1]
                    self.robotTM_scond[rid] = [[]]  # start condition of robotTM
                else:
                    self.robotStart[rid] = robot_pose[rid][0] # start point: the current vertex
                    self.robotTM[rid] = []
                    self.robotTM_set[rid] = []
                    self.PlanExecutedIdx = [-1, -1]
                    self.robotTM_scond = []
            else: # the robot does not have any job
                '''
            if rid in goals.keys():  # get a new job
                Rid_replan.append(rid)
                self.robotStart[rid] = robot_pose[rid][0]
                self.robotGoal[rid] = goals[rid]

        return Rid_replan, Rid_robotTM

    def get_multipath_plan(self, multipaths):  # multipath: MultiPath type
        for id in multipaths.keys():
            if len(multipaths[id]) == 1:
                stationary_check = (self.robotPose[id][0] == self.robotPose[id][1] == multipaths[id][0])
            else:
                stationary_check = False

            if not stationary_check:
                self.current_command[id] = []  # a sequence of vertices 
                self.command_set[id] = []  # [robotTM, robotTM, robotTM, ...]
                self.command_start_condition[id] = []  # start condition of robotTM
                self.PlanExecutedIdx[id] = [-1,
                                            -1]  # [i,j] Save the last index of NavPath[i][j] which the robot follows
                self.Flag_terminate[id] = -1

        start_condition = {}
        for key in multipaths.keys():
            start_condition[key] = []
        # Analyze timing
        for rid, path in multipaths.items():

            for idx in range(0, len(path)):  # compare
                scond = []
                rid_list = list(multipaths.keys())
                rid_list.remove(rid)
                for rid2 in rid_list:
                    for idx2 in range(min(idx, len(multipaths[rid2]) - 1), -1, -1):
                        if path[idx] == multipaths[rid2][idx2]:
                            scond.append([rid2, idx2])

                start_condition[rid].append(scond)

        # --------------------------------------
        # modify stat_condition -10 26
        last_idx_init = {}
        start_condition2 = {}

        for rid in multipaths.keys():
            last_idx_init[rid] = -1
            start_condition2[rid] = []

        for rid1 in multipaths.keys():
            last_idx = last_idx_init.copy()
            robot_start_cond = start_condition[rid1].copy()
            for tidx in range(0, len(robot_start_cond)):  # time
                scond = []  # initialize
                for cond in robot_start_cond[tidx]:
                    if cond != []:
                        if last_idx[cond[0]] < cond[1]:
                            last_idx[cond[0]] = cond[1]
                            scond.append(cond)
                    else:
                        scond.append([])

                start_condition2[rid1].append(scond)

        start_condition = start_condition2
        # --------------------------------------

        # Split navigation paths to robotTM
        # 일렬로 길게 온 path를 회피하는 path인지 분석하고 회피를 잘 할수 있도록 쪼개주는 함수
        robotTM_mapping_set = {}
        scondTM_set = {}

        for rid, path in multipaths.items():
            robotTM_seq = []
            robotTM_mapping = []  # save index of robotTM_set corresponding to each element in path
            scond = []
            t1 = 0
            t2 = 0

            if path != []:
                robotTM = [path[0]]
                scond = [start_condition[rid][0]]
                for ii in range(1, len(path)):
                    if start_condition[rid][ii] == []:
                        if path[ii] != robotTM[-1]:
                            robotTM.append(path[ii])
                            t2 = t2 + 1

                    else:
                        robotTM_seq.append(robotTM)
                        t1 = t1 + 1
                        t2 = 0
                        robotTM = [path[ii]]
                        scond.append(start_condition[rid][ii])

                    robotTM_mapping.append([t1, t2])

                robotTM_seq.append(robotTM)

            self.command_set[rid] = copy.copy(robotTM_seq)
            robotTM_mapping_set[rid] = robotTM_mapping
            scondTM_set[rid] = scond

        # find start condnition for TM
        scond_TM_translated = {}
        for rid in multipaths.keys():
            scond = []
            for tt in range(0, len(scondTM_set[rid])):
                if scondTM_set[rid][tt] != []:
                    cond_cur = []
                    for cond in scondTM_set[rid][tt]:
                        if (len(robotTM_mapping_set[cond[0]][0]) > 0) and (len(robotTM_mapping_set[cond[0]]) > cond[1]):
                            cond_cur.append([cond[0], robotTM_mapping_set[cond[0]][cond[1]]])

                    scond.append(cond_cur)
                else:
                    scond.append([])

            #            scond_TM_translated[rid] = scond
            self.command_start_condition[rid] = scond

    #        self.robotTM_scond = scond_TM_translated

    #처음 상태가 [-1,0]이 되야함
    def update_robot_TM(self, robot_pose):
        Rid_sendRobotTM = [] #robotTM에게 보낼 id

        # robotTM = copy.deepcopy(self.robotTM)
        # robotTM_set = copy.deepcopy(self.robotTM_set)
        # robotTM_scond = copy.deepcopy(self.robotTM_scond) 
        # 참조하는 것이아닌 값을 가져오기 위해서
        robotTM = copy.copy(self.current_command) #진행할때마다 변함 ex)[1,2] > [2] > [3,4,5] > [4,5]
        robotTM_set = copy.copy(self.command_set) #현재 가지고있는 path(변하지않음)
        robotTM_scond = copy.copy(self.command_start_condition) #1번로봇이 가지는 scond{'2' : [0,5]}
        test = 0
        ##vertex = (a,a)?
        #1.executedidx  2.지난 vertex삭제
        #id와 vertex를 입력받음
        for rid, vid in robot_pose.items(): #robot pose는 한번에 4개에대해서 들어옴
            #robot id와 vertex를 가져옴
            self.robotPose[rid] = vid  #들어온 vetex로
            robotTM_check = {"current": False, "skip": False}

            # if rid == "AMR_TOW1":
            #     print("[" + str(rid) + "]" + str(vid[0]) + " " + str(vid[1]) + " " + str(self.robotGoal[rid]) + " " + str(robotTM[rid]) + " " + str(robotTM_set[rid]))
            ## PlanExecutedIdx : plan이 얼마나 진행되었는지 보여주는 idx
            ## Scond : counterpart 로봇의 start condition으로 PlanExecutedIdx와 비교하여 출발할지 결정
            if self.robotGoal[rid] != -1: #goal이 있는 로봇에대해서만 수행     
                if robotTM[rid] != []: #current_command를 가지고 있다
                # if len(robotTM[rid]) > 0:
                    if (vid[0] == robotTM[rid][0]) and (vid[1] == robotTM[rid][0]): #지났다고 판단하면
                        self.PlanExecutedIdx[rid][1] += 1  #1을 더해줌 
                        robotTM_check["current"] = True 
                    elif (vid[0] == vid[1]): #몇번째로 앞에 들어왔는지 확인하고 skip
                        if vid[0] in robotTM[rid]:
                            vidx = robotTM[rid].index(vid[0])
                            self.PlanExecutedIdx[rid][1] += (vidx + 1)
                            robotTM_check["skip"] = True
                    elif (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]): #완료
                        self.Flag_terminate[rid] = 0
                        self.robotGoal[rid] = -1
                elif (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]): #완료
                    self.Flag_terminate[rid] = 0
                    self.robotGoal[rid] = -1
                
                #goal 완료처리 하는 부분
                #idx를 이용하여 완료체크
                if robotTM_set[rid] != []: #goal을 가진 상태
                # if (len(robotTM_set[rid]) > 0) and (len(robotTM_set[rid][0] > 0)):
                    if self.PlanExecutedIdx[rid] == [len(robotTM_set[rid]) - 1, len(robotTM_set[rid][-1]) - 1]: #완료처리가 됐는지 비교
                        if (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]):
                            self.Flag_terminate[rid] = 0  #종료되면 0 / 안되면 -1
                            self.robotGoal[rid] = -1  #종료되면 -1 / 안되면 vertex
                else: #goal이 없는 상태
                    if (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]):
                        self.Flag_terminate[rid] = 0
                        self.robotGoal[rid] = -1

            #끝났는지 안끝났는지 확인하는 파트
            #지난 vertex삭제
            #def 
            if self.Flag_terminate[rid] == -1: #goal까지 안간 set에 대해서
                if robotTM_set[rid] != []: #goal을 가지고 있다
                # if (len(robotTM_set[rid]) > 0) and (len(robotTM_set[rid][0] > 0)):
                    # if (robotTM[rid]!=[]) and (self.PlanExecutedIdx[rid][1]!=-1):
                    if robotTM[rid] != []: #currnet_command를 가지고 있다
                    # if len(robotTM[rid]) > 0:
                        # if (vid[0]=robotTM[rid][0]) and (vid[1]==robotTM[rid][1]):
                        if robotTM_check["current"]:
                            temp_path = robotTM[rid]
                            # print("delete", rid, temp_path[0])
                            # print("before delete", rid, temp_path)
                            #current일때는 젤 앞에거 제거
                            del temp_path[0] 
                            # self.robotTM[rid] = copy.deepcopy(temp_path)
                            print("Current", rid, temp_path)
                            #새로운 Current command
                            self.current_command[rid] = copy.copy(temp_path)
                            # print("after delete", rid, self.robotTM[rid])
                            # print(rid, self.PlanExecutedIdx)
                        # elif (vid[0]==vid[1]):
                        elif robotTM_check["skip"]:
                            temp_path = robotTM[rid]
                            vidx = temp_path.index(vid[0])
                            #skip한거까지 찾아서 제거
                            del temp_path[:vidx + 1] 
                            print("skip", rid, temp_path)
                            # self.robotTM[rid] = copy.deepcopy(robotTM[rid])
                            self.current_command[rid] = copy.copy(temp_path)
                    else: #currnet_command를 가지고 있지x 
                        start_idx = self.PlanExecutedIdx[rid][0] + 1 #다지났으니 +1
                        print('[In the NC] rid : '+str(rid))
                        print('[In the NC] start_idx : '+str(start_idx))

                        print('[In the NC] robotTM_scond : '+str(robotTM_scond))
                        if robotTM_scond[rid][start_idx] == []: #scond이 없을때
                            # robotTM[rid] = copy.deepcopy(robotTM_set[rid][start_idx])
                            # self.robotTM[rid] = copy.deepcopy(robotTM[rid])
                            robotTM[rid] = copy.copy(robotTM_set[rid][start_idx])
                            self.current_command[rid] = copy.copy(robotTM[rid]) #current_command에 새로운 path를 넣어줌
                            self.PlanExecutedIdx[rid] = [start_idx, -1] #뒤에있는것은 초기화
                            Rid_sendRobotTM.append(rid) #그 다음 path를 수행해야함
                        else:  #scond이 있을때 []
                            flag_start = True
                            for cond in robotTM_scond[rid][start_idx]: #현재로봇의 scond을 가져옴
                                #scond이 다음의 조건을 만족시킬 때
                                if self.PlanExecutedIdx[cond[0]][0] < cond[1][0] or self.PlanExecutedIdx[cond[0]][1] < \
                                        cond[1][1]:
                                    flag_start = False
                            if flag_start:
                                # robotTM[rid] = copy.deepcopy(robotTM_set[rid][start_idx])
                                # self.robotTM[rid] = copy.deepcopy(robotTM[rid])
                                robotTM[rid] = copy.copy(robotTM_set[rid][start_idx])
                                self.current_command[rid] = copy.copy(robotTM[rid])
                                self.PlanExecutedIdx[rid] = [start_idx, -1]
                                Rid_sendRobotTM.append(rid)
            elif self.Flag_terminate[rid] == 0: #goal없다
                if self.current_command[rid] != []: #path가 있다
                # if len(self.robotTM[rid]) > 0:
                    if (vid[0] == self.current_command[rid][0]) and (vid[1] == self.current_command[rid][0]):
                        temp_path = self.current_command[rid]
                        del temp_path[0] #지워라
                        self.current_command[rid] = temp_path
            # print(rid, robotTM[rid], robotTM_set[rid], self.PlanExecutedIdx[rid])
            # print(self.robotGoal)
        return Rid_sendRobotTM #path가 새로 생긴로봇을 리턴함

    ### BACKUP ###
    def update_robot_TM_(self, robot_pose):  # call when the robot position changes - # TODO
        # robot_pose ={robot_id: [vertex, vertex], ...}
        Rid_sendRobotTM = []  # list of robot ids, of which robot TM a navigation controller should send
        # print("Robot TM 2:    ", self.robotTM)
        # check whether the current TM command is executed
        for rid, vid in robot_pose.items():
            self.robotPose[rid] = vid
            if self.robotGoal[rid] != -1:
                if self.current_command[rid] != []:  # check plan execution
                    compare_nodes = [[self.current_command[rid][0]] * 2]
                    if len(self.current_command[rid]) > 1:
                        compare_nodes.append(self.current_command[rid][0:2])
                        compare_nodes.append([self.current_command[rid][1], self.current_command[rid][0]])
                    # if vid in [[self.robotTM[rid][0]]*2]:
                    if (vid[0] == self.current_command[rid][0]) and (vid[1] == self.current_command[rid][0]):
                        self.PlanExecutedIdx[rid][1] = self.PlanExecutedIdx[rid][1] + 1

                    if (vid[0] != self.current_command[rid][0]) and (vid[0] == vid[1]):
                        vidx = self.current_command[rid].index(vid[0])
                        self.PlanExecutedIdx[rid][1] = self.PlanExecutedIdx[rid][1] + vidx + 1
                else:
                    if (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]):
                        self.Flag_terminate[rid] = 0
                        self.robotGoal[rid] = -1

                if self.command_set[rid] != []:  # check flag_terminate
                    if self.PlanExecutedIdx[rid] == [len(self.command_set[rid]) - 1,
                                                     len(self.command_set[rid][-1]) - 1]:
                        if vid == [self.robotGoal[rid]] * 2:
                            self.Flag_terminate[rid] = 0
                            self.robotGoal[rid] = -1
                        # else:
                        #     self.Flag_terminate[rid] = 1
                else:
                    # if vid == [self.robotGoal[rid]]*2:
                    if (vid[0] == self.robotGoal[rid]) and (vid[1] == self.robotGoal[rid]):
                        self.Flag_terminate[rid] = 0
                        self.robotGoal[rid] = -1
                    # else:
                    #     self.Flag_terminate[rid] = 1
        # Update the robot TM
        # print("AMR_TOW1", robot_pose["AMR_TOW1"], self.robotGoal["AMR_TOW1"], self.Flag_terminate["AMR_TOW1"])
        # print("AMR_TOW1", " TM", self.robotTM["AMR_TOW1"])
        # print("AMR_TOW1 ", "TM_set ", self.robotTM_set["AMR_TOW1"])
        # print("AMR_TOW1 ", "ExecutedIdx", self.PlanExecutedIdx["AMR_TOW1"])
        # print("AMR_TOW1 ", "scond ", self.robotTM_scond["AMR_TOW1"])
        # print("AMR_TOW2", robot_pose["AMR_TOW2"], self.robotGoal["AMR_TOW2"], self.Flag_terminate["AMR_TOW2"])
        # print("AMR_TOW2", " TM", self.robotTM["AMR_TOW2"])
        # print("AMR_TOW2 ", "TM_set ", self.robotTM_set["AMR_TOW2"])
        # print("AMR_TOW2 ", "ExecutedIdx", self.PlanExecutedIdx["AMR_TOW2"])
        # print("AMR_TOW2 ", "scond ", self.robotTM_scond["AMR_TOW2"])
        for rid, vid in robot_pose.items():
            if self.Flag_terminate[rid] == -1:
                if self.command_set[rid] != []:
                    if (self.current_command[rid] != []) and (self.PlanExecutedIdx[rid][1] != -1):  # update robotTM
                        # if vid in [[self.robotTM[rid][0]]*2]: # if a robot arrives at self.robotTM[rid][0]
                        if (vid[0] == self.current_command[rid][0]) and (vid[1] == self.current_command[rid][0]):
                            temp_path = self.current_command[rid]
                            del temp_path[0]
                            self.current_command[rid] = temp_path
                            # self.robotTM[rid].pop(0)
                        if (vid[0] != self.current_command[rid][0]) and (vid[0] == vid[1]):
                            vidx = self.current_command[rid].index(vid[0])
                            del self.current_command[rid][:vidx + 1]
                    else:  # allocate a new robotTM
                        start_idx = self.PlanExecutedIdx[rid][0] + 1
                        if self.command_start_condition[rid][start_idx] == []:  # no condition
                            self.current_command[rid] = copy.copy(self.command_set[rid][start_idx])
                            self.PlanExecutedIdx[rid] = [start_idx, -1]
                            # send the command
                            Rid_sendRobotTM.append(rid)
                        else:  # check condition
                            flag_start = True
                            for cond in self.command_start_condition[rid][start_idx]:
                                #로봇의 scond이랑 excutedidx를 비교
                                if self.PlanExecutedIdx[cond[0]][0] < cond[1][0] or self.PlanExecutedIdx[cond[0]][1] < \
                                        cond[1][1]:
                                    flag_start = False
                            # for cond in self.robotTM_scond[rid][start_idx][0]:
                            #     if self.PlanExecutedIdx[cond[0]][0] < cond[1][0] or self.PlanExecutedIdx[cond[0]][1] < cond[1][1]:
                            #         flag_start = False
                            if flag_start:
                                self.current_command[rid] = copy.copy(self.command_set[rid][start_idx])
                                self.PlanExecutedIdx[rid] = [start_idx, -1]
                                # send the command
                                Rid_sendRobotTM.append(rid)
                        # if rid=="AMR_TOW1" or rid=="AMR_TOW2":
                        #     print(rid, "start_idx: ", start_idx, "executedidx: ", self.PlanExecutedIdx[rid])
            elif self.Flag_terminate[rid] == 0:
                if self.current_command[rid] != []:
                    if (vid[0] == self.current_command[rid][0]) and (vid[1] == self.current_command[rid][0]):
                        temp_path = self.current_command[rid]
                        del temp_path[0]
                        self.current_command[rid] = temp_path
                        # self.robotTM[rid].pop(0)

        return Rid_sendRobotTM

    def send_RobotTM(self, robotid, robotTM):  # send command to RobotTM
        print("send a command to RobotTM: ", robotid, robotTM)

    def update_start_goal_collision(self, robot_ids):  # execute when the robot TM allocates a goal to each robot ##사용?
        # goals: {robot_id: goal vertex, ....}, robot_pose = {id, [vertex, vertex]}

        for rid in robot_ids:
            if self.current_command[rid] != []:  # the robot is executing the plan
                self.current_command[rid] = [self.current_command[rid][0]]
                #self.robotStart[rid] = self.current_command[rid][0]
                self.command_set[rid] = [self.current_command[rid]]  # [robotTM, robotTM, robotTM, ...]
                #self.PlanExecutedIdx[rid] = [0, -1]
                #self.command_start_condition[rid] = [[]]  # start condition of robotTM


if __name__ == "__main__":
    AMR_IDs = ['AMRLIFT0', 'AMRLIFT1', 'AMRTOW0', 'AMRTOW1']
    navcont = NavigationControl(AMR_IDs)
    multipaths = {'AMRLIFT0': [1, 2, 4, 6, 5], 'AMRLIFT1': [3, 3, 2, 4, 6, 7, 7, 8], 'AMRTOW0': [], 'AMRTOW1': []}
    navcont.get_multipath_plan(multipaths)

    # test - update extract_TM
    # rid = AMR_IDs[1]
    # for sidx in range(0,6):
    #    navcont.extract_TM(navcont.NavPath[rid][sidx:], navcont.timing[rid][sidx:])

    # test -update_TM
    paths_execute = {'AMRLIFT0': [0, 1, 2, 4, 6, 5, 5, 5], 'AMRLIFT1': [3, 3, 2, 4, 6, 7, 7, 8], 'AMRTOW0': [1] * 10,
                     'AMRTOW1': [2] * 10}
    for t in range(0, 7):
        robot_pose = {}
        for key, val in paths_execute.items():
            robot_pose[key] = [val[t]] * 2
        print(paths_execute['AMRLIFT0'][t], paths_execute['AMRLIFT1'][t])
        navcont.update_robot_TM(robot_pose)

    print("done")

    a = [[1,2],[3,4]]
    a[0][0]