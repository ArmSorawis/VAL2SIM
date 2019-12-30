#!/usr/bin/env python

# Import necessary package
from findCurrent_angle import find_currentAngle

# Select action for the robot 
def sim_read_action_surveillance(current_station, next_station, state):
    station_list = ["station1", "station2", "station3", "station4", "base_station", "charging_station"]
    turnDegree_currentStation = 0
    turnDegree_nextStation = 0

    if current_station in station_list and next_station in station_list and state == "turn2nextgoal":
        
        if current_station == "base_station":
            if next_station == "station1":
                turnDegree_nextStation = 0
            elif next_station == "station2":
                turnDegree_nextStation = 0
            elif next_station == "station3":
                turnDegree_nextStation = 0
            elif next_station == "station4":
                turnDegree_nextStation = 0

        elif current_station == "station1":
            if next_station == "station2":
                turnDegree_nextStation = 0
            elif next_station == "station3":
                turnDegree_nextStation = 0
            elif next_station == "station4":
                turnDegree_nextStation = 0
            elif next_station == "base_station":
                turnDegree_nextStation = -180

        elif current_station == "station2":
            if next_station == "station1":
                turnDegree_nextStation = -100
            elif next_station == "station3":
                turnDegree_nextStation = 90
            elif next_station == "station4":
                turnDegree_nextStation = 90
            elif next_station == "base_station":
                turnDegree_nextStation = -100
        
        elif current_station == "station3":
            if next_station == "station1":
                turnDegree_nextStation = -180
            elif next_station == "station2":
                turnDegree_nextStation = -180
            elif next_station == "station4":
                turnDegree_nextStation = -90
            elif next_station == "base_station":
                turnDegree_nextStation = -180

        elif current_station == "station4":
            if next_station == "station1":
                turnDegree_nextStation = 180
            elif next_station == "station2":
                turnDegree_nextStation = 180
            elif next_station == "station3":
                turnDegree_nextStation = 100
            elif next_station == "base_station":
                turnDegree_nextStation = 180

        sim_read_action_surveillance.rotate2nextStation = turnDegree_nextStation
    
    elif current_station in station_list and state == "turn2currentgoal":
        if current_station == "station1" or current_station == "station2" or current_station == "charging_station":
            turnDegree_currentStation = find_currentAngle(current_station)
        sim_read_action_surveillance.rotate2currentStation = turnDegree_currentStation

    elif current_station not in station_list or next_station not in station_list:
        sim_read_action_surveillance.rotate2currentStation = 0
        sim_read_action_surveillance.rotate2nextStation = 0
