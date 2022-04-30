import copy
multipaths = {'AMRLIFT0': [205,206,207,208,209,210,211], 'AMRLIFT1': [223,214,212,211,210,5,210]}#,'AMRTOW0': [233,234,235,237,238,239]}#'AMRTOW0':[234,235,236,237,238,239]}
pathlist = [[1,2,3],[2,3,4]]
T_node = {}
not_overlap_path = []
check_node = []
check_node1 = []
path_list = list(multipaths.values())
rid_list = list(multipaths.keys())


a = copy.copy(multipaths)
for rid,path in a.items():
    a[rid] = pathlist[0]
    pathlist.pop(0)