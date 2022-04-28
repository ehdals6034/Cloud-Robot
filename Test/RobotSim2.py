import random
import copy
class RobotSim2:
    #로봇이 가져야할 정보 : id,state,random 이동 state
    def __init__(self,id,x,y):
        self.current_state = [] #update TM에 보낼 state
        self.trajectory = []
        self.id = id
        self.plan = []  # [type, value], ['move', the list of vertex), 'load'], ['unload']]
        self.status = 'none' # ['none','ing','done'] if a robot is executing a plan 2: done
        self.random_list = []
        self.x = x
        self.y = y

    #로봇에게 plan을 줌
    def insert_plan(self, plan): 
        self.plan = []
        self.plan = plan.copy()
        self.status = 'ing'


    def random_pose_generator(self):
        self.random_list = []
        plan = self.plan[1].copy() #plan : ['move',[1,2]]
        pose_list1 = [] #pose ex)[1,1],[1,2],.
        pose_list2 = [] #pose with number(0,1) ex)[[1,1],0],[[1,1],0],...
        pose_list1.append([plan[0],plan[0]])
        for i in range(len(plan)-1):         
            pose_list1.append([plan[i], plan[i+1]])
            pose_list1.append([plan[i+1], plan[i]])
            pose_list1.append([plan[i+1], plan[i+1]])
        
        pose_list2 = []
        for i in pose_list1[:-2]:           
            pose_list2.append([i,0])
        for i in pose_list1[-2:]:   
            pose_list2.append([i,1])

        
        self.random_list.extend([pose_list2[0],pose_list2[0]])# 처음 값은 사라지면 안되니 append 해주기  
        for i in range(1, len(pose_list2)-1):  # 처음과 마지막 pose는 append로 직접 넣어주고, 가운데 pose는 1~3 사이 값(개수)을 곱해서 append
            n = random.randrange(1, 4)
            for j in range(n): # n!=0이면 n개 만큼 pose 추가
                    self.random_list.append(pose_list2[i]) 
                
        self.random_list.append(pose_list2[-1])
        
        print(self.id,'random_pose:',self.random_list)
    
        return self.random_list

    # def random_pose_generator(self):
    #     self.random_list = []
    #     plan = self.plan[1].copy() #plan : ['move',[1,2]]
    #     pose_list1 = [] #pose ex)[1,1],[1,2],.
    #     pose_list2 = [] #pose with number(0,1) ex)[[1,1],0],[[1,1],0],...
    #     pose_list1.append([plan[0],plan[0]])
    #     for i in range(len(plan)-1):         
    #         pose_list1.append([plan[i], plan[i+1]])
    #         pose_list1.append([plan[i+1], plan[i]])
    #         pose_list1.append([plan[i+1], plan[i+1]])
        
    #     pose_list2.extend([pose_list1[0],pose_list1[0]])
    #     for i in range(1, len(pose_list1)-1):  # 처음과 마지막 pose는 append로 직접 넣어주고, 가운데 pose는 1~3 사이 값(개수)을 곱해서 append
    #         n = random.randrange(1, 4)
    #         for j in range(n): # n!=0이면 n개 만큼 pose 추가
    #             pose_list2.append(pose_list1[i])        
    #     pose_list2.append(pose_list1[-1])
        

    #     for i in pose_list2:
    #         if i in pose_list2[-1:]: #[1,1][1,2] [2,1] [2,2]
    #             self.random_list.append([i,1])
    #         else:
    #             self.random_list.append([i,0])
    #     print(self.id,'random_pose:',self.random_list)
    #     return self.random_list
    
    def move(self):
        self.random_list
        randomlist = self.random_list.copy()
        #print(randomlist,111)
        self.current_state = randomlist[0].copy() #trajectory에 저장할 state
        self.trajectory.extend(self.current_state[0]) #vertex정보만 저장
        #print(self.trajectory,2222)
        if randomlist != []:          
            if len(randomlist) != 1: 
                randomlist.pop(0)
                self.current_state = randomlist[0].copy() #update에게 보낼 state
                self.random_list = randomlist
                #print(self.random_list,2222)
            elif len(randomlist) == 1:
                self.status = 'done'

        

    def clear_plan(self):
        self.plan = []
        

if __name__ =="__main__":
    temp = RobotSim2(123)
    temp.insert_plan(['move',[1,2,3,4]])
    temp.random_pose_generator()
    #print(temp.status)
    # temp.random_pose_generator([1,2,3,4,5])
    # temp.
