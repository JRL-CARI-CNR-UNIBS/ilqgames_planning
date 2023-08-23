import os
import numpy as np
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import pandas as pd

LOG_DIRECTORY = "../logs/"
EXAMPLE_LOGDIR = "hand_tcp_point3D_14"
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
    df = pd.DataFrame(array[0]) # reshape to (n_rows, n_columns)
    df.insert(0, 'time', times)
    return df


def plot_states(states):
    figs = {}
    for operating_point in states.keys():
        if operating_point == max(states.keys()): # plot only the last operating point
            df = array2df(states[operating_point])
            df.columns = ['time', 'x_0', 'vx_0', 'y_0', 'vy_0', 'z_0', 'vz_0','x_1', 'vx_1', 'y_1', 'vy_1', 'z_1', 'vz_1']

            start_state_agent0 = df.iloc[0, 1:7]
            start_state_agent1 = df.iloc[0, 7:13]

            fig3d_agent_traj = go.Figure()
            fig3d_agent_traj.add_scatter3d(x=df['x_0'], y=df['y_0'], z=df['z_0'], mode='lines', name='agent0')
            fig3d_agent_traj.add_scatter3d(x=df['x_1'], y=df['y_1'], z=df['z_1'], mode='lines', name='agent1')
            fig3d_agent_traj.add_scatter3d(x=np.array(start_state_agent0['x_0']), y=np.array(start_state_agent0['y_0']), z=np.array(start_state_agent0['z_0']), mode='markers', name='agent0_start')
            fig3d_agent_traj.add_scatter3d(x=np.array(start_state_agent1['x_1']), y=np.array(start_state_agent1['y_1']), z=np.array(start_state_agent1['z_1']), mode='markers', name='agent1_start')
            figs['OP_' + str(operating_point) + '_trajectories'] = fig3d_agent_traj

            fig_states = px.line(df, x="time", y=df.columns[1:], title='States [OP: ' + str(operating_point) + ']')
            figs['OP_' + str(operating_point) + '_states'] = fig_states

    for fig in figs.values():
        fig.show()


def main():
    path_to_logfile = os.path.join(LOG_DIRECTORY, EXAMPLE_LOGDIR)

    gen_dir = os.walk(path_to_logfile)
    dirs = [x[0] for x in gen_dir][1:]

    costs = {}
    states = {}
    control_inputs = {}
    t0s = {}
    runtimes = {}

    if not dirs:
        print("No log files found in directory " + path_to_logfile + ". Directory must contain at least one subdirectory with log files.")
        return

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






    
