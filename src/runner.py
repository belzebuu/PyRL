#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import logging
from pprint import pformat
from logging.handlers import RotatingFileHandler
from collections import defaultdict

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

formatter = logging.Formatter("(%(levelname)s) PID%(process)d:%(pathname)s:%(funcName)s:%(lineno)d::\n%(message)s")

file_handler = RotatingFileHandler("runner.log", mode='a', maxBytes=5*1024*1024, 
                                   backupCount=1, encoding=None, delay=0)
file_handler.setFormatter(formatter)

logger.addHandler(file_handler)


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

"""
The default traffic light phase configuration is:
<phase state="GGGgGrrrrrGGgGgrrrr" duration="62"/>
<phase state="GGGgGrrrrryygGgrrrr" duration="4"/>
<phase state="GGGgGrrrrrrrgGgrrrr" duration="12"/>
<phase state="GGGgGrrrrrrryyyrrrr" duration="4"/>
<phase state="GGGgGrrrrrrrrrrrrrr" duration="1"/>
<phase state="GGGgGuuurrrrrrrrrrr" duration="2"/>
<phase state="GGGgGguurrrrrrrrrrr" duration="1"/>
<phase state="GGGgGgGgrrrrrrrrrrr" duration="15"/>
<phase state="yyyyyyGgrrrrrrrrrrr" duration="3"/>
<phase state="yyyyyyyyrrrrrrrrrrr" duration="1"/>
<phase state="rrrrrryyrrrrrrrrrrr" duration="3"/>
<phase state="rrrrrrrruurrrrruuuu" duration="2"/>
<phase state="rrrrrrrrGgrrrrrGgGg" duration="7"/>
<phase state="rrrrrrrryyrrrrryyyy" duration="4"/>
<phase state="rrrrrrrrrrrrrrrrrrr" duration="1"/>
<phase state="uuuuurrrrruuuuurrrr" duration="2"/>

16 phases total, changes should be made through the function do_phase_transition()
"""

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

class PhaseController(object):
    """
    Used instead of directly calling traci
    """
    def __init__(self, connection, junction_id):
        self.connection = connection
        self.junction_id = junction_id
        self.last_change_time = 0
        self.program = connection.trafficlight.getCompleteRedYellowGreenDefinition(junction_id)
        self.phases = self.program[0].getPhases()
        self.num_phases = len(self.phases)
        self.step = 0
        self.step_in_cycle = 0
        self.step_in_phase = 0
        self.__generate_phase_transition_timings__()
        self.stat_objects = []

    def __generate_phase_transition_timings__(self):
        keys = []
        res = defaultdict(int)
        transition_index_dict = defaultdict(list)
        transition_index_link = {}
        for index, phase in enumerate(self.phases):
            if int(phase._duration) == 3600:
                keys.append(index)
        self.decision_phases = keys
        for index, phase in enumerate(self.phases):
            if int(phase._duration) < 3600:
                if index > max(keys):
                    res[min(keys)] += phase._duration
                    transition_index_dict[min(keys)].append(index)
                    transition_index_link[index] = min(keys)
                else:
                    for key in keys:
                        if key > index:
                            res[key] += phase._duration
                            transition_index_dict[key].append(index)
                            transition_index_link[index] = key
                            break
        self.phase_to_transition_time = res
        self.decision_phase_index_to_transition_phases = transition_index_dict
        self.transition_to_successive_decision_phase_index = transition_index_link

    def __can_initiate_phase_transition__(self):
        return not self.is_transitioning() and self.step_in_phase > 10

    def get_time_in_cycle(self):
        return self.step_in_cycle

    def get_time_in_phase(self):
        return self.step_in_phase

    def get_time(self):
        return self.step

    def start_phase_transition(self):
        self.last_change_time = self.step
        current_phase = self.connection.trafficlight.getPhase(self.junction_id)
        if self.__can_initiate_phase_transition__():
            self.connection.trafficlight.setPhase(self.junction_id, current_phase + 1 % self.num_phases)
    
    def register_statistic_gatherer(self, stat_object):
        self.stat_objects.append(stat_object)

    def simulation_step(self):
        if self.connection.simulation.getMinExpectedNumber() <= 0:
            return False
        previous_phase = self.connection.trafficlight.getPhase(self.junction_id)
        self.connection.simulationStep()
        current_phase = self.connection.trafficlight.getPhase(self.junction_id)
        if previous_phase != current_phase and current_phase == 0:
            self.step_in_cycle = 0
        else:
            self.step_in_cycle += 1
        if previous_phase != current_phase:
            self.step_in_phase = 0
        else:
            self.step_in_phase += 1
        self.step += 1
        for stat_obj in self.stat_objects:
            stat_obj.on_step(self.connection)
        return True

    def get_available_actions(self):
        if self.__can_initiate_phase_transition__():
            return ["change", "noop"]
        else:
            return ["noop"]

    def close(self):
        self.connection.close()

    def is_transitioning(self):
        return self.connection.trafficlight.getPhase(self.junction_id) not in self.phase_to_transition_time.keys()

class Statistics(object):
    def __init__(self):
        pass
    
    def on_step(self, connection):
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
    #logger.info(pformat(all_lights))
    #logger.info(pformat(all_lanes))
    #other_lights = all_lights.remove(trafficLightID)
    #direct_incoming = traci.trafficlight.getControlledLanes(trafficLightID)
    #other_lights_direct_incoming = []
    #if other_lights:
    #    for tlsID in other_lights:
    #        other_lights_direct_incoming = list(
    #            set(other_lights_direct_incoming + traci.trafficlight.getControlledLanes(tlsID)))
    ## Main logic
    #incoming = direct_incoming
    #levels = dict.fromkeys(incoming, 0)
    #changed = True
    #while changed:
    #    changed = False
    #    for lane in all_lanes:
    #        goes_to = [x[0] for x in traci.lane.getLinks(lane)]
    #        for toLane in goes_to:
    #            if toLane in incoming and toLane not in other_lights_direct_incoming:
    #                if lane in incoming:
    #                    #raise Exception("Lane already in incoming list", lane)
    #                    break
    #                levels[lane] = levels[toLane] + 1
    #                incoming.append(lane)
    #                changed = True
    #tmp = {}
    #for lane in incoming:
    #    l = levels[lane]
    #    try:
    #        tmp[l].append(lane)
    #    except (KeyError):
    #        tmp[l] = [lane]
    return 0


def get_value(lane_map):
    """ Value of the current state of the simulation (Mostly for benchmarking)
    
    :rtype Integer: cumulative waiting time of cars currently in the network
    """
    # if traci.vehicle.getIDCount() == 0:
        # return 0.0
    # assert(traci.simulation.getCollidingVehiclesNumber == 0)
    # assert(traci.simulation.getStartingTeleportNumber == 0)
    v = 0
    #for i in range(0, max(lane_map.keys())):
    #    prev = v
    #    for lane in lane_map[i]:
    #        v -= traci.lane.getWaitingTime(lane)
    #    if prev == v:
    #        break
    return v



def run(connection):
    junction_id = "gneJ6"
    incoming = get_incoming_lanes_by_jumps(junction_id)
    # Get TLS Program information
    controller = PhaseController(connection, junction_id)
    """execute the TraCI control loop"""
    while controller.simulation_step():
        logger.debug(controller.get_time())
        if "change" in controller.get_available_actions():
            controller.start_phase_transition()
        #logger.debug(f"step: {step}; nextSwitch: {connection.trafficlight.getNextSwitch(junction_id)}; config: {connection.trafficlight.getRedYellowGreenState(junction_id)}")
        #print(value)
    controller.close()
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
                             "--tripinfo-output", "tripinfo.xml"], label="a")
    #traci.start([sumoBinary, "-c", "data/anl427.sumocfg",
    #                         "--tripinfo-output", "tripinfo.xml"], label="b")
    conn1 = traci.getConnection("a")
    #conn2 = traci.getConnection("b")
    import time
    input("Logs can be attached now. Press Enter to continue")
    run(conn1)
    #run(conn2)
