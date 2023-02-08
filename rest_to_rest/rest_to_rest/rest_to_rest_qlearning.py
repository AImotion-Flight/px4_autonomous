import sys
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker
import numpy as np
from util import *
from environment import GridEnvironment
from agent import DynamicalSystem
from qlearning import *

plt.rcParams.update({
    "text.usetex": True,
    "font.size": 16,
    "text.latex.preamble" : r"\usepackage{amsmath}"
})

if __name__ == '__main__':
    map = np.ones((10, 15))
    map[7, 0] = 0
    map[3, 2] = 0
    map[9, 4] = 0
    map[4, 6] = 0
    map[8, 8] = 0
    map[0:3, 4:9] = 0
    map[7:10, 12:15] = 0
    map[2:4, 11] = 0
    map[2, 9:11] = 0

    initial_state = (0, 0, 0, 0)
    final_state = (11, 0, 0, 0)
    
    vstates = generate_states_vector(map, [-2, -1, 0, 1, 2])
    print(vstates)
    vactions = generate_actions_vector([-1, 0, 1])
    print(vactions)

    env = GridEnvironment(map)
    agent = DynamicalSystem(vstates, vactions, -2, 2)
    algorithm = {
        'Q': QLearning(initial_state, final_state, agent, env, 100000, 1, 0.95, 0.1, False),
        'SARSA': SARSA(initial_state, final_state, agent, env, 100000, 1, 0.95, 0.1, 0.99, False)
    }

    try:
        while 1:
            cmd = input('cmd> ').split()
            if cmd[0] == 'map':
                size = np.shape(map)
                fig, ax = plt.subplots()
                fig.canvas.manager.set_window_title('Occuopancy Grid Map')
                fig.set_size_inches((10, 10))
    
                ax.set_xlabel(r'$x$ [m]', fontsize=22)
                ax.set_ylabel(r'$y$ [m]', fontsize=22)
    
                ax.set_xticks(np.arange(0, size[1], 1))
                ax.set_yticks(np.arange(0, size[0], 1))
    
                ax.set_xticks(np.arange(-0.5, size[1], 1), minor=True)
                ax.set_yticks(np.arange(-0.5, size[0], 1), minor=True)

                ax.tick_params(which='minor', bottom=False, left=False)

                ax.grid(which='minor')
                ax.imshow(map, cmap='gray', origin='lower')
                ax.plot(initial_state[0], initial_state[1], 'go', markersize=20)
                ax.plot(final_state[0], final_state[1], 'r*', markersize=20)
                fig.show()
            elif cmd[0] == 'compare':
                convgQ, _ = algorithm['Q'].learn()
                algorithm['Q'].save('models/Q.npy')
                convgSARSA, _ = algorithm['SARSA'].learn()
                algorithm['SARSA'].save('models/SARSA.npy')
 
                fig, ax = plt.subplots()
                ax.set_xlabel(r'Episode', fontsize=22)
                ax.set_ylabel(r'$$\max_a Q(s, a)$$', fontsize=22)
                ax.plot(np.arange(np.shape(convgQ)[0]), convgQ, label='Q Learning')
                ax.plot(np.arange(np.shape(convgQ)[0]), convgSARSA, label=r'SARSA')
                ax.legend()
                fig.savefig("img/convergence.svg", dpi=300, bbox_inches = "tight")
                fig.show()
            elif cmd[0] == 'comparepaths':
                q, _ = algorithm['Q'].get_policy()
                sarsa, _ = algorithm['SARSA'].get_policy()

                fig = plt.figure('Summary', figsize=(15, 15), tight_layout=True)
                gs = gridspec.GridSpec(1, 2)

                size = np.shape(map)
                ax = fig.add_subplot(gs[0, 0])
                ax.set_xlabel('x [m]')
                ax.set_ylabel('y [m]')
                ax.set_xlabel('x [m]')
                ax.set_ylabel('y [m]')
                ax.set_xticks(np.arange(0, size[1], 1))
                ax.set_yticks(np.arange(0, size[0], 1))
                ax.set_xticks(np.arange(-0.5, size[1], 1), minor=True)
                ax.set_yticks(np.arange(-0.5, size[0], 1), minor=True)
                ax.tick_params(which='minor', bottom=False, left=False)
                ax.grid(which='minor')
                ax.imshow(map, cmap='gray', origin='lower')
                ax.plot(q[:, 0], q[:, 1], 'b--')
                ax.plot(q[0, 0], q[0, 1], 'go')
                ax.plot(q[-1, 0], q[-1, 1], 'r*')

                ax = fig.add_subplot(gs[0, 1])
                ax.set_xlabel('x [m]')
                ax.set_ylabel('y [m]')
                ax.set_xlabel('x [m]')
                ax.set_ylabel('y [m]')
                ax.set_xticks(np.arange(0, size[1], 1))
                ax.set_yticks(np.arange(0, size[0], 1))
                ax.set_xticks(np.arange(-0.5, size[1], 1), minor=True)
                ax.set_yticks(np.arange(-0.5, size[0], 1), minor=True)
                ax.tick_params(which='minor', bottom=False, left=False)
                ax.grid(which='minor')
                ax.imshow(map, cmap='gray', origin='lower')
                ax.plot(sarsa[:, 0], sarsa[:, 1], 'b--')
                ax.plot(sarsa[0, 0], sarsa[0, 1], 'go')
                ax.plot(sarsa[-1, 0], sarsa[-1, 1], 'r*')

                fig.show()
            elif cmd[0] == 'learn':
                algorithm[cmd[1]].learn()
                algorithm[cmd[1]].save('models/' + cmd[1])
            elif cmd[0] == 'load':
                algorithm[cmd[1]].load(cmd[1] + '.npy')
            elif cmd[0] == 'plot':
                path, aseq = algorithm[cmd[1]].get_policy()
                print(path)
                print(aseq)
                fig = plt.figure('Summary', figsize=(15, 15), tight_layout=True)
                gs = gridspec.GridSpec(4, 2)

                size = np.shape(map)
                ax = fig.add_subplot(gs[0, :])
                ax.set_xlabel('x [m]')
                ax.set_ylabel('y [m]')
                ax.set_xlabel('x [m]')
                ax.set_ylabel('y [m]')
                ax.set_xticks(np.arange(0, size[1], 1))
                ax.set_yticks(np.arange(0, size[0], 1))
                ax.set_xticks(np.arange(-0.5, size[1], 1), minor=True)
                ax.set_yticks(np.arange(-0.5, size[0], 1), minor=True)
                ax.tick_params(which='minor', bottom=False, left=False)
                ax.grid(which='minor')
                ax.imshow(map, cmap='gray', origin='lower')
                ax.plot(path[:, 0], path[:, 1], 'b--')
                ax.plot(path[0, 0], path[0, 1], 'go')
                ax.plot(path[-1, 0], path[-1, 1], 'r*')
                
                ax = fig.add_subplot(gs[1, 0])
                ax.set_xlabel('t [step]')
                ax.set_ylabel('x [m]')
                ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.plot(np.arange(np.shape(path)[0]), path[:, 0])
                
                ax = fig.add_subplot(gs[1, 1])
                ax.set_xlabel('t [step]')
                ax.set_ylabel('y [m]')
                ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.plot(np.arange(np.shape(path)[0]), path[:, 1])
                
                ax = fig.add_subplot(gs[2, 0])
                ax.set_xlabel('t [step]')
                ax.set_ylabel('vx [m/step]')
                ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.plot(np.arange(np.shape(path)[0]), path[:, 2])

                ax = fig.add_subplot(gs[2, 1])
                ax.set_xlabel('t [step]')
                ax.set_ylabel('vy [m/step]')
                ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.plot(np.arange(np.shape(path)[0]), path[:, 3])

                ax = fig.add_subplot(gs[3, 0])
                ax.set_xlabel('t [step]')
                ax.set_ylabel('ax [m/step]')
                ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.plot(np.arange(np.shape(aseq)[0]), aseq[:, 0])

                ax = fig.add_subplot(gs[3, 1])
                ax.set_xlabel('t [step]')
                ax.set_ylabel('ay [m/step]')
                ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
                ax.plot(np.arange(np.shape(aseq)[0]), aseq[:, 1])

                fig.show()
            elif cmd[0] == "exit":
                raise KeyboardInterrupt
            else:
                print('unknown command')
    except KeyboardInterrupt:
        sys.exit()
