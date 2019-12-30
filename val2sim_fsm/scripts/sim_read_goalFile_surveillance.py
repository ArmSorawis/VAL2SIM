#!/usr/bin/env python

# Import necessary package
from os.path import expanduser

# Initialize home directory
home = expanduser("~")

# Read goal from /val2sim_ws/src/val2sim_navigation/text/goal_simulation_surveillance.txt file and keep goals in list
def sim_read_goal_surveillance():
    goal_path = "{}/val2sim_ws/src/val2sim_navigation/text/".format(home)
    goal_filename = "goal_simulation_surveillance.txt"
    path = goal_path + goal_filename
    fil_goalList = []
    num_goal = 6

    with open(path) as f:
        goal_list = f.read().splitlines()
    
    for index in range(len(goal_list)):
        if index < num_goal:
            string = goal_list[index]
            goal_list[index] = string.split(", ")
            fil_goalList.append(goal_list[index])
        if index >= num_goal:
            pass
    for index in range(len(fil_goalList)):
        for index_sub in range(len(fil_goalList[index])):
            fil_goalList[index][index_sub] = float(fil_goalList[index][index_sub])

    goal_data = {"station1": fil_goalList[0], 
                 "station2": fil_goalList[1],
                 "station3": fil_goalList[2],
                 "station4": fil_goalList[3],
                 "base_station": fil_goalList[4]}

    return goal_data

# goal = sim_read_goal_surveillance()
# print(goal)
# print(goal['station1'])
