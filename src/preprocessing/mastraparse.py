import sys
import datetime
import time
import numpy as np
import math
from pprint import pprint
import operator
import itertools
import functools
import random
from ortools.linear_solver import pywraplp
import multiprocessing as mp
import os
from collections import defaultdict

class mastra():
    """
    Parses .mastra notation traffic detector data, and gives options for output.
    WARNING: Certain assumptions are made and hence may not be of use to everyone.
    """

    def __init__(self, input_file):
        self.periods = {}
        self.__parse__(input_file)
        pass

    def __parse__(self, file):
        with open(file, "r") as f_in:
            for line in f_in.readlines():
                data = line.rstrip().split(" ")
                if any(x in ["*BEGIN", "*END"] for x in data):
                    continue
                ckey = (data[0], data[1])
                if self.periods.get(ckey, None) is None:
                    self.periods[ckey] = [int(x) for x in data[2:]]
                else:
                    self.periods[ckey].extend([int(x) for x in data[2:]])

    def get_periods(self):
        return self.periods

    def __ub_solver__(self, route_vec_dict, capacity_vector):
        A = capacity_vector
        B = []
        route_to_int = {}
        c = 0
        for route, bin_repr in route_vec_dict.items():
            route_to_int[route] = c
            B.append(bin_repr)
            c += 1
        n_detectors = len(A)
        n_routes = len(B)
        solver = pywraplp.Solver(
            'solver', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
        # Variables
        X = [
            solver.IntVar(
                0,
                solver.infinity(),
                "x[{}]".format(route_to_int[route])
            )
            for route in route_vec_dict.keys()
        ]
        # Constraints
        limits = {}
        for i in range(n_detectors):
            limits[i] = solver.Add(
                solver.Sum(
                    B[route_to_int[j]][i] * X[route_to_int[j]] for j in route_vec_dict.keys()
                ) <= A[i]
            )
        cost = solver.Sum(
            solver.Sum(
                B[route_to_int[j]][i] * X[route_to_int[j]]
                for j in route_vec_dict.keys()
            ) * -1 + A[i]
            for i in range(n_detectors))
        # print(cost)
        # Objective
        solver.Minimize(cost)
        if solver.Solve() == solver.INFEASIBLE:
            raise Exception("INFEASIBLE")
        # print("Final cost: ", solver.Objective().Value() )
        route_count = {}
        remainder = capacity_vector
        for route, index in route_to_int.items():
            route_count[route] = int(X[index].SolutionValue())
            for i in range(n_detectors):
                remainder[i] -= B[index][i] * int(X[index].SolutionValue())
        return {"route_count": route_count, "remainder": remainder}

    def __lb_solver__(self, route_vec_dict, capacity_vector):
        # Filter out detectors not covered by any route
        for route, bin_repr in route_vec_dict.items():
            try:
                accu = accu | np.array(bin_repr)
            except Exception:
                accu = np.array(bin_repr)
        for i in range(len(capacity_vector)):
            if accu[i] == 0:
                capacity_vector[i] = 0
        A = capacity_vector
        B = []
        route_to_int = {}
        c = 0
        for route, bin_repr in route_vec_dict.items():
            route_to_int[route] = c
            B.append(bin_repr)
            c += 1
        n_detectors = len(A)
        solver = pywraplp.Solver(
            'solver', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
        # Variables
        X = [
            solver.IntVar(
                0,
                solver.infinity(),
                "x[{}]".format(route_to_int[route])
            )
            for route in route_vec_dict.keys()
        ]
        # Constraints
        limits = {}
        for i in range(n_detectors):
            limits[i] = solver.Add(
                solver.Sum(
                    B[route_to_int[j]][i] * X[route_to_int[j]] for j in route_vec_dict.keys()
                ) >= A[i]
            )
        cost = solver.Sum(
            solver.Sum(
                B[route_to_int[j]][i] * X[route_to_int[j]]
                for i in range(n_detectors)
            ) for j in route_vec_dict.keys())
        # print(cost)
        # Objective
        solver.Minimize(cost)
        if solver.Solve() == solver.INFEASIBLE:
            raise Exception("INFEASIBLE")
        # print("Final cost: ", solver.Objective().Value())
        route_count = {}
        for route, index in route_to_int.items():
            route_count[route] = int(X[index].SolutionValue())
        return {"route_count": route_count}

    def get_route_vehicle_counts(self, routes, granularity=5, oidx=False, ub_solver=False):
        """
        Checks how many vehicles follow each route
            :param routes: List of integer lists/tuples representing detectors visited in order by a route
            :param granularity: aggregate all readings into intervals of _granularity_ intervals
            :param oidx=False: Are the detectors one-indexed, rather than 0?
        """
        readings = self.aggregate_readings(granularity)
        route_vec_map = {}
        for route in routes:
            route_vec_map[route] = self.get_route_vector(
                self.__oidxfix__(route, oidx), len(readings[0]))
        route_count = []
        if ub_solver:
            # Initialize array of 0's with correct length
            remainder = readings[0] - readings[0]
            for reading in readings:
                reading += remainder
                # print(["{}:{}".format(index+1, count) for (index, count) in enumerate(reading) if count != 0])
                result = self.__ub_solver__(route_vec_map, reading)
                remainder = result["remainder"]
                route_count.append(
                    {route: count for route, count in result["route_count"].items() if count != 0})
                # print(["{}:{}".format(detector+1, count)
                #    for (detector, count) in enumerate(remainder) if count != 0])
        else:
            l = [(route_vec_map, reading) for reading in readings]
            sol = []
            try:
                with mp.Pool(mp.cpu_count()) as p:
                    sol = p.starmap(self.__lb_solver__, l)
                    p.close()
                    p.join()
            except NotImplementedError:
                with mp.Pool(4) as p:
                    sol = p.starmap(self.__lb_solver__, l)
                    p.close()
                    p.join()
            route_count = [
                    {
                        route: count 
                        for route, count in result["route_count"].items() 
                        if count != 0
                    } 
                    for result in sol
                ]
        return(route_count)

    def __oidxfix__(self, tup, mode):
        """
        Meant for getting around one-indexing of detector ids
            :param tup: The iterable containing detector ids, tuples recommended
            :param mode: Is the tuple currently one-indexed?
                If mode=True  -> go from 1-index to 0-index
                else -> Leave as is
        """
        if mode:
            result = []
            for i in tup:
                result.append(i-1)
            return(tuple(result))
        else:
            return(tup)

    def get_route_vector(self, route, reading_length):
        result = []
        for i in range(reading_length):
            if i in route:
                result.append(1)
            else:
                result.append(0)
        return np.array(result)

    def aggregate_readings(self, period):
        """
        Sums up the detector outputs by groups defined by the period parameter. 
        Not useful if readings are less frequent than period.
            :param period: How many minutes is considered a group? e.g. 60 will group vehicles on an hourly basis. 
        """
        result = []
        dates = []
        for d, _ in self.periods.keys():
            if d not in dates:
                dates.append(d)
        d = 0
        t = 0
        t_e = 0
        d_e = 0
        while d < len(dates):
            done = False
            while not done:
                # Get start and stop date/time
                t_e += period
                if t_e >= 1440:
                    t_e %= 1440
                    d_e += 1
                    if not len(dates) > d_e:
                        done = True
                        break
                vehicles = self.__get_vehicles_by_interval(int(dates[d]), int(
                    dates[d_e]), get_time_from_minutes(t), get_time_from_minutes(t_e))
                if vehicles is not None:
                    result.append(vehicles)
                t += period
                if t >= 1440:
                    t %= 1440
                    done = True
            d += 1
            d_e = d
        return(result)

    def __get_vehicles_by_interval(self, start_d, end_d, start_t, end_t):
        keys = []
        result = None
        for a, b in self.periods.keys():
            if int(a) >= start_d and int(a) <= end_d:
                if (int(b) > start_t or int(a) > start_d) and (int(b) <= end_t or int(a) < end_d):
                    keys.append((a, b))
        for key in keys:
            a = np.array(self.periods[key])
            if result is None:
                result = a
            else:
                result = result + a
        return(result)


def get_time_from_minutes(t):
    h = str(int(t / 60))
    m = str(int(t % 60))
    if len(h) < 2:
        h = "0"+h
    if len(m) < 2:
        m = "0"+m
    return int(h+m)


def str_tuple_to_int_tuple(tup):
    result = []
    for s in tup:
        result.append(int(s))
    return tuple(result)


def str_tuple_list_to_int_tuple_list(tuple_list):
    result = []
    for tup in tuple_list:
        result.append(str_tuple_to_int_tuple(tup))
    return result


def generate_vehicles(detector_to_route_id, detector_route_counts, interval, multiplier=1.0, out=sys.stdout):
    """
    Generates a .rou.xml file for use in the traffic simulator SUMO
    """
    pprint(detector_to_route_id)
    pprint(detector_route_counts)
    pprint(interval)
    pprint(multiplier)
    def id_num_to_route_id(route_number):
        if type(route_number) == int:
            return "r" + str(route_number)
        elif type(route_number) == str:
            return "r" + route_number
        else:
            return "r" + str(int(route_number))
    tmp = {}
    for k, v in detector_to_route_id.items():
        tmp[str_tuple_to_int_tuple(k)] = v
    detector_to_route_id = tmp
    try:
        assert(len(detector_route_counts) == int(1440/interval))
    except AssertionError:
        print("Number of route counts not as expected, continuing.")
    # Get info and generate vehicles
    v_id = 0
    group = 0
    veh_str = "<vehicle id=\"{}\" type=\"type1\" route=\"{}\" depart=\"{}\"/>"
    veh_str_alt = "<vehicle id=\"{}\" route=\"{}\" depart=\"{}\"/>"
    rc = filter(None, detector_route_counts)
    departure_dict = defaultdict(list)
    for route_counts in rc:
        group += 1
        for route, count in route_counts.items():
            count = int(count * multiplier)
            for _ in range(count):
                time = int(random.uniform((group-1) * interval * 60, group * interval * 60))
                departure_dict[time].append(random.choice(detector_to_route_id[route]))
    for time, departures in departure_dict.items():
        v_id = 0
        for r in departures:
            r_id = id_num_to_route_id(r)
            print(veh_str_alt.format(v_id, r_id, time), file=out)
            v_id += 1
    # group = 0
    # for route_counts in rc:
    #     group += 1
    #     departure_intervals = {}
    #     for route, count in route_counts.items():
    #         minutes_between_departures = float(interval)/float(count)*0.7
    #         seconds_between_departures = datetime.timedelta(
    #             minutes=minutes_between_departures)
    #         departure_intervals[route] = int(
    #             seconds_between_departures.total_seconds())
    #         # if count == 1:
    #         # departure_intervals[route] = random.randint(interval*60/2+1, interval*60-1)
    #         departure_tracker = departure_intervals.copy()
    #     for i in range((group-1) * interval * 60, group * interval * 60):
    #         for k, v in departure_tracker.items():
    #             if v == 0:
    #                 r_id = id_num_to_route_id(
    #                     random.choice(detector_to_route_id[k]))
    #                 print(veh_str_alt.format(v_id, r_id, i), file=out)
    #                 v_id += 1
    #                 departure_tracker[k] = departure_intervals[k]
    #             else:
    #                 departure_tracker[k] -= 1


# Each tuple is an order of detectors which a route follows. Should be converted to int tuples before use
detector_routes = [('1', '2'),
                   ('1', '2', '3', '23', '22'),
                   ('1', '2', '5', '24'),
                   ('1', '2', '5', '24', '20'),
                   ('1', '4', '3', '23', '22'),
                   ('3', '23', '22'),
                   ('5', '24'),
                   ('5', '24', '20'),
                   ('7', '8', '9', '21'),
                   ('7', '8', '10', '11', '22'),
                   ('7', '8', '12'),
                   ('14', '15'),
                   ('14', '15', '20'),
                   ('14', '19'),
                   ('16', '20'),
                   ('16', '21'),
                   ('16', '22')]

# Dict mapping detector ordering -> arbitrary string list, each identifying a route.
# This is used as different routes could pass the same detector ordering.
# Single member lists used if there is only one route passing that detector ordering.
# Only used for vehicle generation.
detector_to_routeid = {('0',): ['12', '13', '15', '24'],
                       ('1', '2'): ['18', '27'],
                       ('1', '2', '3', '23', '22'): ['31'],
                       ('1', '2', '5', '24'): ['2', '17'],
                       ('1', '2', '5', '24', '20'): ['20', '26'],
                       ('1', '4', '3', '23', '22'): ['6', '21'],
                       ('14', '15'): ['4'],
                       ('14', '15', '20'): ['1', '22'],
                       ('14', '19'): ['5', '8', '14', '19'],
                       ('16', '20'): ['11'],
                       ('16', '21'): ['28', '32'],
                       ('16', '22'): ['7', '29'],
                       ('3', '23', '22'): ['0', '25'],
                       ('5', '24'): ['9'],
                       ('5', '24', '20'): ['10'],
                       ('7', '8', '10', '11', '22'): ['3'],
                       ('7', '8', '12'): ['23'],
                       ('7', '8', '9', '21'): ['16', '30']}


def main(input):
    random.seed(0)
    interval = 5
    m = mastra(input)
    # pprint(m.get_periods())
    # pprint(m.aggregate_readings(interval))
    # counts = m.get_route_vehicle_counts(str_tuple_list_to_int_tuple_list(detector_routes), oidx=True)
    # pprint(counts)
    dr = str_tuple_list_to_int_tuple_list(detector_routes)
    rc = m.get_route_vehicle_counts(
        dr, granularity=interval, oidx=True, ub_solver=False)
    date = datetime.datetime.now()
    with open("{}-{}-{}-{}-{}.rou.xml".format(date.year, date.month, date.day, date.hour, date.minute), "w") as f_out:
        generate_vehicles(detector_to_routeid, rc, interval, out=f_out)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("please include a file to parse")
        exit(0)
    print("Running mastra parser on " + sys.argv[1])
    main(sys.argv[1])
