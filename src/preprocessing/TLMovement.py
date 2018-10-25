from pprint import pprint
from functools import reduce
from functools import partial

""" 
    File which captures the rules of traffic junction Anl427
"""

# Global constants

bin_format = '019b'  # replace 19 with number of controlled lanes
phase_format = "<phase duration=\"{}\" state=\"{}\"/>"

movement_to_connectionlist = {
    "A1": [0, 1, 2, 3, 4],
    "A1v": [5],
    "B1h": [6, 7],
    "B1": [8, 9],
    "A2Cy": [10, 11],
    "A2": [12, 13, 14],
    "B2": [15, 16, 17, 18]
}

movement_configuration = {
    "A1": ["G", "G", "G", "g", "G"],
    "A1v": ["g"],
    "B1h": ["G", "g"],
    "B1": ["G", "g"],
    "A2Cy": ["G", "G"],
    "A2": ["g", "G", "g"],
    "B2": ["G", "g", "G", "g"]
}

movement_blocks = [
    ("A1", "A2", "A2Cy"),
    ("A1", "A1v", "B1h"),
    ("B1", "B2")
]

yellow_time = 4  # The amount of time a traffic light is yellow after green

red_clearance_time = 1  # Only used when two adjacent phases have no green lanes in common

# The amount of time a traffic light is orange (red+yellow) before green
orange_time = 2

green_times = {  # Format: (offset, min, max)
    ("A1", "A2", "A2Cy"): ((0, 8, 78), (0, 8, 78), (0, 8, 62)),
    ("A1", "A1v", "B1h"): ((0, 4, 16), (0, 4, 16), (1, 6, 18)),
    ("B1", "B2"): ((0, 7, 16), (0, 7, 16))
}

green_times_new = {  # Format: (offset, min, max)
    ("A1", "A2", "A2Cy"): {"A1": (0, 8, 78), "A2": (0, 8, 78), "A2Cy": (0, 8, 62)},
    ("A1", "A1v", "B1h"): {"A1": (0, 4, 16), "A1v": (0, 4, 16), "B1h": (1, 6, 18)},
    ("B1", "B2"): {"B1": (0, 7, 16), "B2": (0, 7, 16)}
}

block_order = (movement_blocks[0], movement_blocks[1], movement_blocks[2])

num_connections = max(
    list(map(lambda x: max(x), movement_to_connectionlist.values())))+1


def int_to_bin_repr(int_dict):
    max_idx = 0
    for key in int_dict.keys():
        max_idx = max(max_idx, max(int_dict[key]))
    result = {}
    for key, value in int_dict.items():
        for i in range(0, max_idx+1):
            if i in value:
                try:
                    bin_string = bin_string << 1
                    bin_string += 1
                except Exception:
                    bin_string = 1
            else:
                try:
                    bin_string = bin_string << 1
                except Exception:
                    bin_string = 0
        result[key] = bin_string
        bin_string = None
    return(result)

def connection_id_to_movement_id(connection):
    for movement, connections in movement_to_connectionlist.items():
        if connection in connections:
            return movement


def get_phases_helper(block_bin_repr, block_lane_priority_map):
    # Function for comparing two triples element-wise. Equal if 0.
    def eq_triple(a, b):
        assert(len(a) == len(b) == 3)
        x = a[0] ^ b[0]
        y = a[1] ^ b[1]
        z = a[2] ^ b[2]
        return (x | y | z)

    def generic_string(block, true, false):
        string = ""
        k = 1
        n = 0
        for i in range(num_connections-1, -1, -1):
            k = 1 << i
            current_movement = connection_id_to_movement_id(n)
            if block_bin_repr[block] & k > 0:
                string += true(block, current_movement, n, k, i)
            else: 
                string += false(block, current_movement, n, k, i)
            n += 1
        return string

    def do_simple(current_block, next_block):
        # We assume that orange is defined by previous phase (circularly)
        # Generate Green, since all lanes have the same timing, we need not worry about duration as much here.
        # Offsets are also a non-factor as they are relative to the earliest green lane, but all lanes are equal here
        # In the simple case, the phases are as follows: green -> yellow -> red -> orange(next phase preparation)
        # Only tricky part is that we want to keep movements green if they exist in both current and next block
        keep_active = set.intersection(set(current_block), set(next_block))
        phases = []
        # Green
        f = lambda b, cm, n, k, i: list(filter(lambda x: x[0] == n, block_lane_priority_map[b]))[0][1]
        g = lambda b, cm, n,k,i : 'r'
        phases.append(generic_string(current_block, f, g))
        # Yellow
        f = lambda b, cm, n, k, i: 'y' if len(set.intersection(keep_active, {cm})) == 0 else list(filter(lambda x: x[0] == n, block_lane_priority_map[b]))[0][1]
        g = lambda b, cm, n, k, i: 'r'
        phases.append(generic_string(current_block, f, g))
        # Red
        f = lambda b, cm, n, k, i: 'r' if len(set.intersection(keep_active, {cm})) == 0 else list(filter(lambda x: x[0] == n, block_lane_priority_map[b]))[0][1]
        g = lambda b, cm, n, k, i: 'r'
        phases.append(generic_string(current_block, f, g))
        # Orange
        f = lambda b, cm, n, k, i: 'u' if len(set.intersection(keep_active, {cm})) == 0 else list(filter(lambda x: x[0] == n, block_lane_priority_map[b]))[0][1]
        g = lambda b, cm, n, k, i: 'r'
        phases.append(generic_string(next_block, f, g))
        return(phases)


    def get_state_at_time(block, next_block, active_set, t, max_t):
        string = ""
        for i in range(num_connections):
            current_movement = connection_id_to_movement_id(i)
            if current_movement in block:  # connection belongs to current block
                offset = green_times_new[block][current_movement][0]
                maximum = green_times_new[block][current_movement][2] + offset
                if t >= offset and (t < maximum or current_movement in active_set):
                    string += list(filter(lambda x: x[0] == i, block_lane_priority_map[block]))[0][1]
                elif t >= maximum and t < maximum + yellow_time:
                    string += 'y'
                elif t >= maximum + yellow_time:
                    string += 'r'
                elif t < offset:
                    string += 'u'
                else:
                    print("U MISSED SOMETHING BRUH", t, offset, maximum)
            elif current_movement in next_block:  # connection belongs to next block
                if t >= max_t + yellow_time + red_clearance_time:
                    string += 'u'
                else: 
                    string += 'r'
            else:
                string += 'r'
        return string

    def do_advanced(current_block, next_block):
        keep_active = set.intersection(set(current_block), set(next_block))
        max_t = max(map(lambda x: x[2], green_times[current_block]))
        print("max_t", max_t)
        phases = []
        prev_string = ""
        duration_map = {}
        for i in range(max_t + yellow_time + red_clearance_time + orange_time):
            new_string = get_state_at_time(current_block, next_block, keep_active, i, max_t)
            if new_string == prev_string:
                try:
                    duration_map[prev_string] += 1
                except KeyError:
                    duration_map[prev_string] = 1
                continue
            else:
                duration_map[new_string] = 1
                prev_string = new_string
                phases.append(new_string)
        return(phases, duration_map)

    # Get some time information
    all_eq = {}
    for i in range(len(block_order)):
        current_block = block_order[i]
        next_block = block_order[(i+1) % len(block_order)]
        difference = 0
        for i in range(len(green_times[current_block])-1):
            difference += eq_triple(green_times[current_block]
                                    [i], green_times[current_block][i+1])
        all_eq[current_block] = True if difference == 0 else False
    block_info = []
    for i in range(len(block_order)):
        current_block = block_order[i]
        next_block = block_order[(i+1) % len(block_order)]
        phases, duration_map = do_advanced(current_block, next_block)
        block_info.append((phases, duration_map))
    phase_ordering = []
    phase_duration_dict = {}
    for i in range(len(block_info)):
        phase_ordering.extend(block_info[i][0])
        phase_duration_dict = {**phase_duration_dict, **block_info[i][1]}
    for phase in phase_ordering:
        print(phase_format.format(phase_duration_dict[phase], phase))


def get_phases(bin_repr_map, lane_priority_map):
    block_lane_priority_map = {}
    for block in movement_blocks:
        tmp = []
        for movement in block:
            tmp.extend(lane_priority_map[movement])
        block_lane_priority_map[block] = tmp
    # Combine movements into blocks
    block_bin_repr = {}
    for block in movement_blocks:
        _block_bin_repr = 0
        for movement in block:
            # binary or between movements to combine them into blocks
            _block_bin_repr = _block_bin_repr | bin_repr_map[movement]
        block_bin_repr[block] = _block_bin_repr
    # Generate phases
    get_phases_helper(block_bin_repr, block_lane_priority_map)


def main():
    print("Number of connections:", num_connections)
    bin_repr = int_to_bin_repr(movement_to_connectionlist)
    lane_priority_map = reduce(lambda x, y: {**x, **y}, list(map(lambda conn, conf: {conn[0]: list(zip(conn[1], conf[1]))} if (
        conn[0] == conf[0]) else None,
        movement_to_connectionlist.items(),
        movement_configuration.items())))
    get_phases(bin_repr, lane_priority_map)

    pass


if __name__ == '__main__':
    main()
