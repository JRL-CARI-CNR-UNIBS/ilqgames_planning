import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from IPython.display import display, clear_output

sns.set_theme(context="paper", style="whitegrid")

LOG_DIRECTORY = "../logs/"
EXAMPLE_LOGDIR = "hand_tcp_point3D_50" # to test single iteration
EXAMPLE_LOGDIR_RECEDING = "hand_tcp_point3D_receding_5" # to test full trajectory along receding horizon
SAMPLING_TIME = 0.1 # [s]
N_STATES_PER_AGENT = 6
N_AGENTS = 2
RECEDING_HORIZON = True


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
        if RECEDING_HORIZON:
            element = entry[1]
        else:
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
    column_names = []
    for i in range(N_AGENTS):
        column_names += ['x_'+str(i), 'vx_'+str(i), 'y_'+str(i), 'vy_'+str(i), 'z_'+str(i), 'vz_'+str(i)]

    df.columns = column_names
    df.columns = df.columns.map('_'.join)
    
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
    
    if N_AGENTS > 1:
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

    if N_AGENTS == 1:
        fig = plt.figure()
    else:
        fig, axs = plt.subplots(nrows=N_AGENTS, ncols=1, sharex=True)

    fig.suptitle('Control inputs [OP: ' + str(max_operating_point) + ']')
    fig.canvas.manager.set_window_title('Control inputs')

    i = 0
    for agent in control_inputs[max_operating_point].keys():
        df = array2df(control_inputs[max_operating_point][agent])
        df.columns = ['ax', 'ay', 'az']
        
        if N_AGENTS == 1:
            ax = fig.add_subplot()
            ax.set_title('Agent: ' + agent)
            sns.lineplot(ax=ax, data=df)
        else:
            axs[i].set_title('Agent: ' + agent)
            sns.lineplot(ax=axs[i], data=df)
                     
        i += 1

    plt.legend(loc='upper right')
    plt.draw()


def plot_traj_3d_receding(states):
    actual_states = []
    for entry in states.values():
        actual_states.append(entry[0][0].tolist())
    actual_states = np.asarray(actual_states)
    
    fig = plt.figure('3D trajectories - Receding horizon')
    ax = fig.add_subplot(projection='3d')

    # Agent 1 (Human Hand)
    ax.plot(xs=actual_states[:,0], ys=actual_states[:,2], zs=actual_states[:,4], marker='o', markersize=1)
    ax.scatter(xs=actual_states[0,0], ys=actual_states[0,2], zs=actual_states[0,4], c='r', marker='o', label='Human hand')
    
    if N_AGENTS > 1:
        # Agent 2 (Robot Tcp)
        ax.plot(xs=actual_states[:,6], ys=actual_states[:,8], zs=actual_states[:,10], marker='o', markersize=0.75, alpha=0.5)
        ax.scatter(xs=actual_states[0,6], ys=actual_states[0,8], zs=actual_states[0,10], c='g', marker='o', label='Robot TCP')
    
    # setting title and labels
    ax.set_title("Evolution of the state")
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlabel('x')
    ax.legend()
    plt.legend(loc='upper right')

    plt.draw()


def plot_predictions_3d_receding(states, t0s): # plot future optimal trajectory of each agent for the given receding horizon invocation               
    final_time = list(t0s.values())[-1][0][0][0]

    # Update axes limits just at initialization
    x0 = states[0][0][0][[0,6]]
    y0 = states[0][0][0][[2,8]]
    z0 = states[0][0][0][[4,10]]

    # Create and configure axes
    fig = plt.figure('3D trajectories - Receding horizon')
    fig.canvas.manager.window.attributes('-topmost', 1) # bring the figure window to the front
    ax = fig.add_subplot(projection='3d')
    ax.set_title("Evolution of the state")
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlabel('x')
    
    i = 0
    for entry in states.values():
        actual_states = np.asarray(entry[0].tolist())
        actual_time = t0s[i][0][0][0]

        ax.cla()

        # Agent 1 (Human Hand)
        ax.plot(xs=actual_states[:,0], ys=actual_states[:,2], zs=actual_states[:,4], marker='o', markersize=1)
        ax.scatter(xs=actual_states[0,0], ys=actual_states[0,2], zs=actual_states[0,4], c='r', marker='o', label='Human hand')
        
        if N_AGENTS > 1:
            # Agent 2 (Robot Tcp)
            ax.plot(xs=actual_states[:,6], ys=actual_states[:,8], zs=actual_states[:,10], marker='o', markersize=0.75, alpha=0.5)
            ax.scatter(xs=actual_states[0,6], ys=actual_states[0,8], zs=actual_states[0,10], c='g', marker='o', label='Robot TCP')
        
        # Update axes limits for each iteration
        # xs = np.append(actual_states[:,0], actual_states[:,6])
        # ys = np.append(actual_states[:,2], actual_states[:,8])
        # zs = np.append(actual_states[:,4], actual_states[:,10])
    
        # Update axes limits for each iteration
        # ax.set_xlim(min(xs), max(xs))
        # ax.set_ylim(min(ys), max(ys))
        # ax.set_zlim(min(zs), max(zs))

        # Update axes limits just based on initial state
        ax.set_xlim(min(x0), max(x0))
        ax.set_ylim(min(y0), max(y0))
        ax.set_zlim(min(z0), max(z0))

        # Draw legend
        ax.legend()
        plt.legend(loc='upper right')
        
        display(fig)
        clear_output(wait = True) # Clear the previous graph
        
        i += 1

        # Wait for key press to advance to the next iteration
        input("Press Enter to display next receding horizon invocation... \
              [iter " + str(i) + " out of " + str(len(states.values())) + \
              " | time: " + str(actual_time) + " s / " + str(final_time) + " s]")
        plt.pause(0.01)
        
        # # Advance automatically to the next iteration
        # print("iter " + str(i) + " out of " + str(len(states.values())) + \
        #       " | time: " + str(actual_time) + " s / " + str(final_time) + " s")
        # plt.pause(1)

    plt.close(fig)


def add_iteration_data(dir, control_inputs, states, t0s, runtimes, costs, operating_point):
    add_element_to_dict(control_inputs, operating_point, {}) # initialize control inputs dictionary for the current operating point
                        
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


def main():
    if RECEDING_HORIZON:
        path_to_logfile = os.path.join(LOG_DIRECTORY, EXAMPLE_LOGDIR_RECEDING)
        gen_dir = os.walk(path_to_logfile)
        dirs = [x[0] for x in gen_dir if ("iter" in x[0].split('/')[-1])]
    else:
        path_to_logfile = os.path.join(LOG_DIRECTORY, EXAMPLE_LOGDIR)
        gen_dir = os.walk(path_to_logfile)
        dirs = [x[0] for x in gen_dir][1:]

    if not dirs:
        print("No log files found in directory " + path_to_logfile + ". Directory must contain at least one subdirectory with log files.")
        return

    costs = {}
    states = {}
    control_inputs = {}
    t0s = {}
    runtimes = {}

    if RECEDING_HORIZON:
        for dir in dirs:
            gen_subdir = os.walk(dir)
            subdirs = [x[0] for x in gen_subdir][1:]

            operating_point = int(dir.split('_')[-1])

            if subdirs:
                latest_operating_point = 0
                for subdir in subdirs:
                    latest_subdir = int(subdir.split('/')[-1])
                    if latest_subdir > latest_operating_point:
                        latest_operating_point = latest_subdir
                
                
                selected_dir = dir + '/' + str(latest_operating_point)
                add_iteration_data(selected_dir, control_inputs, states, t0s, runtimes, costs, operating_point)

            else:
                add_iteration_data(dir, control_inputs, states, t0s, runtimes, costs, operating_point)

        costs = sort_dict(costs)
        states = sort_dict(states)
        control_inputs = sort_dict(control_inputs)
        t0s = sort_dict(t0s)
        runtimes = sort_dict(runtimes)

        plot_predictions_3d_receding(states, t0s)
        # plot_traj_3d_receding(states)

    else:
        for dir in dirs:
            operating_point = int(dir.split('/')[-1])
            add_iteration_data(dir, control_inputs, states, t0s, runtimes, costs, operating_point)

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






    
