import os
import numpy as np
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import pandas as pd

LOG_DIRECTORY = "../logs/"
SAMPLING_TIME = 0.1 # [s]
N_STATES_PER_AGENT = 6
N_AGENTS = 2


def parse_txt(filename):
    items = []
    
    f = open(filename,'r')
    for row in f:
        row = row.split(' ')
        row = [item for item in row if item]    # filter out empty strings
        row[-1] = row[-1][:-1]                  # remove trailing "\n" from the last element of the row

        for item in row:
            items.append(item)

    items = np.asarray(items, dtype=float)
    items = np.reshape(items, (-1, len(row)))

    return items


def add_element_to_dict(dictionary, key, value):
    if key not in dictionary:
        dictionary[key] = []
    dictionary[key].append(value)


# Recursively sort all entries of a dictionary
def sort_dict(dictionary):
    keys = list(dictionary.keys())
    keys.sort()
    sorted_dictionary = {i : dictionary[i] for i in keys}

    for entry in dictionary.items():
        element = entry[1][0]
        if isinstance(element, dict):
            sorted_element = sort_dict(element)
            sorted_dictionary[entry[0]] = sorted_element

    return sorted_dictionary


def array2df(array):
    times = pd.DataFrame(np.arange(0, array[0].shape[0]*SAMPLING_TIME, SAMPLING_TIME)) # create time vector
    df = pd.DataFrame(np.reshape(array[0], (array[0].shape[1], -1)).T) # reshape to (n_rows, n_columns)
    df.insert(0, 'time', times)
    return df


def plot_states(states):
    for operating_point in states.keys():
        df = array2df(states[operating_point])
        df.columns = ['time', 'x_0', 'vx_0', 'y_0', 'vy_0', 'z_0', 'vz_0','x_1', 'vx_1', 'y_1', 'vy_1', 'z_1', 'vz_1']

    fig = px.line(df, x="time", y=df.columns[1:], title='States')
    fig.show()  
    fig3d_agent0 = px.line_3d(df, x="x_0", y="y_0", z="z_0", title='Trajectory of agent 0 in cartesian space')
    fig3d_agent0.show()
    fig3d_agent1 = px.line_3d(df, x="x_1", y="y_1", z="z_1", title='Trajectory of agent 1 in cartesian space')
    fig3d_agent1.show() 


def main():
    example_logdir = "hand_tcp_point3D"
    path_to_logfile = os.path.join(LOG_DIRECTORY, example_logdir)

    gen_dir = os.walk(path_to_logfile)
    dirs = [x[0] for x in gen_dir][1:]

    costs = {}
    states = {}
    control_inputs = {}
    t0s = {}
    runtimes = {}

    for dir in dirs:
        operating_point = int(dir.split('/')[-1])
        add_element_to_dict(control_inputs, operating_point, {})
                            
        gen_files = os.walk(dir)
        files = [x[2] for x in gen_files][0]

        for file in files:
            full_path_file = os.path.join(dir,file)
            data = parse_txt(full_path_file)

            if file == 'xs.txt':
                add_element_to_dict(states, operating_point, data)
            elif file[0] == 'u':
                agent_number = file[1:-4]
                add_element_to_dict(control_inputs[operating_point][0], ('agent_' + agent_number), data)
            elif file == 't0.txt':
                add_element_to_dict(t0s, operating_point, data)
            elif file == 'cumulative_runtimes.txt':
                add_element_to_dict(runtimes, operating_point, data)
            elif file == 'costs.txt':
                add_element_to_dict(costs, operating_point, data)

    costs = sort_dict(costs)
    states = sort_dict(states)
    control_inputs = sort_dict(control_inputs)
    t0s = sort_dict(t0s)
    runtimes = sort_dict(runtimes)

    plot_states(states)

    pass # for debug


if __name__ == "__main__":
    main()






    
