import matplotlib.pyplot as plt
import numpy as np
from itertools import cycle


class Evaluator(object):

    def __init__(self):
        self.eval_data = {'Nf':{}}

    def plot_evaluation(self, iterations, actions_per_iter, reward_per_iter):
        fig, ax1 = plt.subplots()

        ax1.set_xlabel('iterations')
        ax1.set_ylabel('cumulative_reward', color='b')
        ax1.plot(iterations, reward_per_iter)

        ax2 = ax1.twinx()
        ax2.set_ylabel('actions', color='r')
        ax2.plot(iterations, actions_per_iter, color='r', linestyle=':')

        fig.tight_layout()
        plt.show()

    def plot_comparison(self, iterations, data_to_compare):
        jet = plt.get_cmap('jet')
        kinds = len(data_to_compare.items())
        colors = iter(('r', 'g', 'b', 'k'))
        lines = ["-", "--", "-.", ":"]
        linecycler = cycle(lines)

        fig, ax1 = plt.subplots()
        ax1.set_xlabel('iterations')
        ax1.set_ylabel('actions')
        i = 1
        for key, data in data_to_compare.items():
            ax1.plot(iterations, data['actions_per_iter'], label='NF=%s, %.2f' % (key, 1./(1+int(key))), color=next(colors),linestyle=next(linecycler))
            i += 1

        fig.tight_layout()
        plt.legend(loc='best')
        plt.title('Actions pet iteration based on learning factor')
        plt.show()