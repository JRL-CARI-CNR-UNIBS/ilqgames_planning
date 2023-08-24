import os
import numpy as np
import pandas as pd
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

sns.set_theme(context="paper", style="whitegrid")

LOG_DIRECTORY = "../logs/"
EXAMPLE_LOGDIR = "hand_tcp_point3D_0"
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
    df.set_index('time', inplace=True)
    return df


def plot_states(states):
    max_operating_point = max(states.keys()) # plot only the last operating point
    
    fig = plt.figure('States')
    ax = fig.add_subplot()
    ax.set_title("States [OP: " + str(max_operating_point) + "]")

    df = array2df(states[max_operating_point])
    df.columns = ['x_0', 'vx_0', 'y_0', 'vy_0', 'z_0', 'vz_0','x_1', 'vx_1', 'y_1', 'vy_1', 'z_1', 'vz_1']
    
    sns.lineplot(ax=ax, data=df)
    plt.legend(loc='upper right')

    plt.draw()


def plot_traj_3d(states):
    max_operating_point = max(states.keys()) # plot only the last operating point
    
    fig = plt.figure('3D trajectories')
    ax = fig.add_subplot(projection='3d')

    # Agent 1 (Human Hand)
    ax.plot(xs=states[max_operating_point][0][:,0],
                     ys=states[max_operating_point][0][:,2],
                     zs=states[max_operating_point][0][:,4], marker='o', markersize=1)
    ax.scatter(xs=states[max_operating_point][0][0,0],
               ys=states[max_operating_point][0][0,2],
               zs=states[max_operating_point][0][0,4], c='r', marker='o', label='Human hand')
    
    # Agent 2 (Robot Tcp)
    ax.plot(xs=states[max_operating_point][0][:,6],
                        ys=states[max_operating_point][0][:,8],
                        zs=states[max_operating_point][0][:,10], marker='o', markersize=0.75)
    ax.scatter(xs=states[max_operating_point][0][0,6],
               ys=states[max_operating_point][0][0,8],
               zs=states[max_operating_point][0][0,10], c='g', marker='o', label='Robot TCP')
    
    # setting title and labels
    ax.set_title("States [OP: " + str(max_operating_point) + "]")
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlabel('x')
    ax.legend()
    plt.legend(loc='upper right')

    plt.draw()


def plot_control_inputs(control_inputs):
    max_operating_point = max(control_inputs.keys()) # plot only the last operating point

    fig, axs = plt.subplots(nrows=N_AGENTS, ncols=1, sharex=True)
    fig.suptitle('Control inputs [OP: ' + str(max_operating_point) + ']')
    fig.canvas.manager.set_window_title('Control inputs')

    i = 0
    for agent in control_inputs[max_operating_point].keys():
        df = array2df(control_inputs[max_operating_point][agent])
        df.columns = ['ax', 'ay', 'az']
        
        axs[i].set_title('Agent: ' + agent)
        sns.lineplot(ax=axs[i], data=df)
                     
        i += 1

    plt.legend(loc='upper right')
    plt.draw()


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

    plot_traj_3d(states)
    plot_states(states)
    plot_control_inputs(control_inputs)

    plt.show()
    pass # for debug


if __name__ == "__main__":
    main()






    
