#!/usr/bin/env python

from findNext_angle import find_nextAngle
from findCurrent_angle import find_currentAngle
from os.path import expanduser

home = expanduser("~")

def sim_read_action(current_station, next_station, state):

    station_list = ["initial_station","station1", "station2", "base_station", "charging_station"]

    turnDegree_currentStation = 0
    turnDegree_nextStation = 0

    soundPath = None
    soundFilename = None
    sound = None
    sound_cycleTime = 0

    if next_station in station_list and state == "turn2nextgoal":
        turnDegree_nextStation = find_nextAngle(next_station)
        sim_read_action.rotate2nextStation = turnDegree_nextStation

    elif current_station in station_list and state == "play_sound":
        if current_station == "initial_station":
            soundPath = '{}/val2sim_ws/src/val2sim_sound/sound/'.format(home)
            soundFilename = "system_checking.wav" 
            sound_cycleTime = 6

        elif current_station == "station1":
            soundPath = '{}/val2sim_ws/src/val2sim_sound/sound/'.format(home)
            soundFilename = "station1.wav" 
            sound_cycleTime = 2
            

        elif current_station == "station2":
            soundPath = '{}/val2sim_ws/src/val2sim_sound/sound/'.format(home)
            soundFilename = "station2.wav" 
            sound_cycleTime = 2

        elif current_station == "base_station":
            soundPath = '{}/val2sim_ws/src/val2sim_sound/sound/'.format(home)
            soundFilename = "base_station.wav" 
            sound_cycleTime = 4

        sim_read_action.sound = soundPath + soundFilename
        sim_read_action.sound_cycleTime = sound_cycleTime
    
    elif current_station in station_list and state == "turn2currentgoal":
        if current_station == "station1":
            turnDegree_currentStation = find_currentAngle(current_station)
        elif current_station == "station2":
            turnDegree_currentStation = find_currentAngle(current_station)
        elif current_station == "charging_station":
            turnDegree_currentStation = find_currentAngle(current_station)
        sim_read_action.rotate2currentStation = turnDegree_currentStation

    elif current_station not in station_list or next_station not in station_list:
        sim_read_action.rotate2currentStation = 0
        sim_read_action.rotate2nextStation = 0
        sim_read_action.sound = None
        sim_read_action.sound_cycleTime = 0
        
# sim_read_action("initial_station", "station1")
# print(sim_read_action.rotate2nextStation)