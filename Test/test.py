def update_T_node(self, robotPose):     
        T_command = copy.copy(self.T_command) #T_node update시 참고할 current_command
        for rid, pose_info in robotPose.items():
            if self.robotGoal[rid] != -1:    
                vid = pose_info[0]
                robotPose[rid] = vid
                if (vid[0] != vid[1]):
                    if (vid[0] in T_command) and (vid[1] in T_command):
                        node = T_command[rid][0]
                        self.T_node[node].pop(0)
                        T_command[rid].pop(0)
                        self.T_command[rid] = T_command[rid]

    print('T_node:',self.T_node)
    print('T_command:',self.T_command)

a = [1,2,3]
if (2 in a) and (1 in a):
    print(123)