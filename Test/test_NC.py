import sys
import os
import matplotlib.pyplot as plt
import copy
#from Test.TestToolkit import draw
#print(os.getcwd())
#sys.path.append('C:/Users/user/OneDrive/robot')
sys.path.append('c:/Users/user/Desktop/robot')
#sys.path.append('C:/Users/admin/Desktop/robot')
from NavigationControl.NavigationControl2 import *
from Test.RobotSim2 import RobotSim2
from MapManagement.MapMOS import MapMOS
from MapManagement.MapCloudlet import MapCloudlet
from matplotlib.animation import FuncAnimation

# Setting
map_file = "./data/map_cloud.txt"
MAP = MapMOS(map_file)
AMR_LIFT_IDs = ['AMRLIFT0', 'AMRLIFT1']
AMR_TOW_IDs = ['AMRTOW0', 'AMRTOW1']
draw_set={'AMRLIFT0':'ro','AMRLIFT1': 'bo', 'AMRTOW0':'r^', 'AMRTOW1':'b^'}



#AMR_IDs = ['AMRLIFT0','AMRLIFT1','AMRLIFT2']
AMR_LIFT_IDs = ['AMRLIFT0', 'AMRLIFT1']
AMR_TOW_IDs = ['AMRTOW0', 'AMRTOW1']



#MultiPath = {'AMRLIFT0': [219, 220, 7], 'AMRLIFT1': [222,221,220,219]}
multipath1 = {'AMRLIFT0': [205,206,207,208,209,210,211], 'AMRLIFT1': [223,214,212,211,210,5]}#,'AMRTOW0': [233,234,235,237,238,239]}#'AMRTOW0':[234,235,236,237,238,239]}
#MultiPath2 = {'AMRLIFT0': [219, 220, 6], 'AMRLIFT1': [221,221,220,219,218]}
Goal1 = {'AMRLIFT0': 211,'AMRLIFT1': 5}#, 'AMRTOW0':239}
#Goal2 = {'AMRLIFT1':218}
# RobotInit
AMR_LIFT_init = {'AMRLIFT0':205, 'AMRLIFT1':207} # 218, 222
AMR_TOW_init = {'AMRTOW0':233, 'AMRTOW1':103}
init_poses = {'AMRLIFT0':[205,205], 'AMRLIFT1':[223,223]}#, 'AMRTOW0': [223,223]}
# init_poses2 = {'AMRLIFT0':[218,219], 'AMRLIFT1':[223,223], 'AMRTOW0':[22,22]}

# # CargoInit
# Cargo_init = [6,3]
# # RACK
# Rack_LIFT_init = {'RACKLIFT0':18, 'RACKLIFT1':19, 'RACKLIFT2':6, 'RACKLIFT3':3}
# Rack_TOW_init = {'RACKTOW0':22, 'RACKTOW1':13}

# # Door_init
# Door_init = {'Door0':0}



# smap = MapCloudlet(map_file,AMR_TOW_init = AMR_TOW_init, AMR_LIFT_init=AMR_LIFT_init,
#                    RACK_TOW_init=Rack_TOW_init, RACK_LIFT_init=Rack_LIFT_init,
#                    Door_init = Door_init, t_init=0) # semantic map



navcont = NavigationControl2(AMR_LIFT_IDs + AMR_TOW_IDs, AMR_LIFT_IDs, AMR_TOW_IDs)
navcont.allocate_goal(Goal1, robot_pose=init_poses)
navcont.get_multipath_plan(multipath1) # allocate multi paths

robots = {}
for rid, node in AMR_LIFT_init.items():
    robots[rid] = RobotSim2(rid, MAP.VertexPos[node][0], MAP.VertexPos[node][2])
for rid, node in AMR_TOW_init.items():
    robots[rid] = RobotSim2(rid, MAP.VertexPos[node][0], MAP.VertexPos[node][2])

# # MAP.draw_map()
# # for rid, robot in robots.items():
# #     plt.plot(robot.x,robot.y, draw_set[rid],markersize=12)
# #plt.show()

for rid in multipath1.keys():   
    robots[rid].insert_plan(['move',navcont.current_command[rid].copy()])
    robots[rid].random_pose_generator()

#Run
FLAG_RUN = True
num11 = 0
while FLAG_RUN:
    # if num11 == 1: # The new goal is allocated 추후에 추가
    #     Rid_replan, Rid_robotTM = navcont.allocate_goal(Goal2, robot_pose=init_poses2)
    #     for rid in Rid_robotTM:
    #         print(Rid_robotTM,1234,navcont.robotGoal[rid])
    #         robots[rid].insert_plan(['move',navcont.current_command[rid].copy()])
    #         robots[rid].random_pose_generator()

    # if num11 == 2: #10이 될 때
    #     navcont.get_multipath_plan(multipath2)  # allocate multi paths
    #     for rid in multipath1.keys():   
    #         robots[rid].insert_plan(['move',navcont.current_command[rid].copy()])
    #         robots[rid].random_pose_generator() #[1,2] => [[[1,1],0],..,[[2,2],1]]
        #print(robots[key].status,1111)
    #current pose받는 딕션어리
        #print(robots.random_list)    

    num11 = num11 + 1
    #여기서부터는 while loop 반복
    #1번move가 안됨 / 
    for rid in robots.keys():
        if robots[rid].status == 'ing':
            robots[rid].move() #move 
        elif robots[rid].status == 'done':
            robots[rid].status = 'none'
        elif robots[rid].status == 'none':
            pass

    #current pose받는 딕션어리
    #현재 pose 계속 전달
    robot_pose_cur = {}
    for rid in AMR_LIFT_IDs:
        robot_pose_cur[rid] = robots[rid].current_state.copy()

    
    #vertex to pose
    for rid, pose in robot_pose_cur.items():
        robots[rid].x = MAP.VertexPos[pose[0][0]][0]
        robots[rid].y = MAP.VertexPos[pose[0][0]][2]


    # for rid, robot in robots.items():
    #     robot.x = MAP.VertexPos[robots[rid].current_state[0][0]]
    #     robot.y = MAP.VertexPos[robots[rid].current_state[0][0]]

    print('current_pose:',robot_pose_cur)
    
        #print(MAP.VertexPos[pose[0][0]][0],111) #MAP.VertexPos[node][0]
        #robots[rid].y = MAP.VertexPos[pose[0][0]][2]

    navcont.update_T_node(robot_pose_cur.copy())
    TM_update_list = navcont.update_robot_command(robot_pose_cur.copy()) #=>goal_check할때 pose[1] = 1이면 goal check
    
    for rid in TM_update_list:
        robots[rid].insert_plan(['move', navcont.current_command[rid].copy()])
        robots[rid].random_pose_generator() #[123] 

    MAP.draw_map()
    for rid, robot in robots.items():
        plt.plot(robot.x,robot.y, draw_set[rid],markersize=15)
        plt.pause(0.05)
        
    #plt.show()
    plt.cla()
    # if num11 == 1:
    #     break   
    print('------------------------------------------')

plt.show()

