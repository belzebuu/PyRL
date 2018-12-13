#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci


# Start implementation here

class Agent:

    agents = []

    def __init__(self, junction_id):  # initialize an agent
        self.jid = junction_id
        self.c1 = {}  # Dictionary: state,action -> times observed
        self.c2 = {}  # Dictionary: state,action,newstate -> times oberved
        self.incoming = get_incoming_lanes(junction_id)
        self.agents.append(self)
        pass

    def update_c1(self):

        pass

    def update_c2(self):
        pass

    def get_local_state(self):
        phase = traci.trafficlight.getPhase(self.jid)
        vehicles = traci.vehicle.getIDList()
        localstate = []
        for vehicle in vehicles:
            lane = traci.vehicle.getLaneID(vehicle)
            if lane not in self.incoming:
                vehicles.remove(vehicle)
            else:
                position = traci.vehicle.getLanePosition(vehicle) // 5
                localstate.append((vehicle, lane, position))
                pass
        return (phase, localstate)

    def transition_model(self):

        pass

    def expected_reward(self):  # TODO: relies on state being implementated
        r = 0
        return r

    def getBestAction(self):  # returns the best action (greedy?)
        pass

    def getAction(self):  # returns actual action to be taken
        pass

    def update(self):  # update stuff?
        pass
    pass


# Very inefficient way of getting incoming lanes for traffic lights. Unfortuantely needed.
def get_incoming_lanes(trafficLightID):
    # TODO Consider grouping the lanes such that each group correspond to the lanes linked with one controlled lane
    # Preparation
    all_lanes = traci.lane.getIDList()
    all_lights = traci.trafficlight.getIDList()
    other_lights = all_lights.remove(trafficLightID)
    direct_incoming = traci.trafficlight.getControlledLanes(trafficLightID)
    other_lights_direct_incoming = []
    if other_lights:
        for tlsID in other_lights:
            other_lights_direct_incoming = list(
                set(other_lights_direct_incoming + traci.trafficlight.getControlledLanes(tlsID)))
    # Main logic
    incoming = direct_incoming
    changed = True
    while changed:
        changed = False
        for lane in all_lanes:
            goes_to = [x[0] for x in traci.lane.getLinks(lane)]
            for toLane in goes_to:
                if toLane in incoming and toLane not in other_lights_direct_incoming:
                    if lane in incoming:
                        #raise Exception("Lane already in incoming list", lane)
                        break
                    incoming.append(lane)
                    changed = True
    return incoming

# Very inefficient way of getting incoming lanes for traffic lights. Unfortuantely needed.
def get_incoming_lanes_by_jumps(trafficLightID):
    # TODO Consider grouping the lanes such that each group correspond to the lanes linked with one controlled lane
    # Preparation
    all_lanes = traci.lane.getIDList()
    all_lights = traci.trafficlight.getIDList()
    other_lights = all_lights.remove(trafficLightID)
    direct_incoming = traci.trafficlight.getControlledLanes(trafficLightID)
    other_lights_direct_incoming = []
    if other_lights:
        for tlsID in other_lights:
            other_lights_direct_incoming = list(
                set(other_lights_direct_incoming + traci.trafficlight.getControlledLanes(tlsID)))
    # Main logic
    incoming = direct_incoming
    levels = dict.fromkeys(incoming, 0)
    changed = True
    while changed:
        changed = False
        for lane in all_lanes:
            goes_to = [x[0] for x in traci.lane.getLinks(lane)]
            for toLane in goes_to:
                if toLane in incoming and toLane not in other_lights_direct_incoming:
                    if lane in incoming:
                        #raise Exception("Lane already in incoming list", lane)
                        break
                    levels[lane] = levels[toLane] + 1
                    incoming.append(lane)
                    changed = True
    tmp = {}
    for lane in incoming:
        l = levels[lane]
        try:
            tmp[l].append(lane)
        except (KeyError):
            tmp[l] = [lane]
    return tmp


def get_value(lane_map):
    """ Value of the current state of the simulation (Mostly for benchmarking)
    
    :rtype Integer: cumulative waiting time of cars currently in the network
    """
    # if traci.vehicle.getIDCount() == 0:
        # return 0.0
    # assert(traci.simulation.getCollidingVehiclesNumber == 0)
    # assert(traci.simulation.getStartingTeleportNumber == 0)
    v = 0
    for i in range(0, max(lane_map.keys())):
        prev = v
        for lane in lane_map[i]:
            v -= traci.lane.getWaitingTime(lane)
        if prev == v:
            break
    return v


def run():
    junction_id = "gneJ6"
    incoming = get_incoming_lanes_by_jumps(junction_id)
    """execute the TraCI control loop"""
    step = 0
    # we start with phase 2 where EW has green
    traci.trafficlight.setPhase(junction_id, 2)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        value = get_value(incoming)
        print(value)
        """ if traci.trafficlight.getPhase(junction_id) == 2:
            # we are not already switching
            if traci.inductionloop.getLastStepVehicleNumber("FIXME") > 0:
                # there is a vehicle from the north, switch
                traci.trafficlight.setPhase(junction_id, 3)
            else:
                # otherwise try to keep green for EW
                traci.trafficlight.setPhase(junction_id, 2) """
        step += 1
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, _ = optParser.parse_args()  # _ --> Arguments
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/anl427.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
