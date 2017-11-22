import matplotlib.pyplot as plt
import numpy as np
from itertools import cycle
import matplotlib.lines as mlines


class Evaluator(object):

    def __init__(self):
        self.eval_data = {'Nf':{}, 'Ne':{}, 'small':{}, 'medium':{}}

    def plot_evaluation(self, iterations, actions_per_iter, reward_per_iter):
        fig, ax1 = plt.subplots()

        ax1.set_xlabel('iterations')
        ax1.set_ylabel('cumulative_reward')
        ax1.plot(iterations, reward_per_iter)

        ax2 = ax1.twinx()
        ax2.set_ylabel('actions')
        ax2.plot(iterations, actions_per_iter, linestyle=':')

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

    def plot_trainings(self, iterations, small_data, medium_data):

        # fig, ax = plt.subplots(ncols=2)
        plt.rcParams.update({'font.size': 15})

        # create all axes we need
        ax0 = plt.subplot(121)
        ax1 = ax0.twinx()
        ax2 = plt.subplot(122)
        ax3 = ax2.twinx()

        # share the secondary axes
        ax1.get_shared_y_axes().join(ax1, ax3)

        ax0.set_xlabel('iterations')
        ax0.set_ylabel('cumulative_reward')
        ax0.set_title('Small sized mission')
        ax0.plot(iterations, small_data['reward_per_iter'], 'r', label='reward')
        ax1.set_ylabel('actions')
        ax1.plot(iterations, small_data['actions_per_iter'], 'g', label='actions', linestyle='--')
        ax2.plot(iterations, medium_data['reward_per_iter'], 'r', label='reward')
        ax2.set_xlabel('iterations')
        ax2.set_ylabel('cumulative_reward')
        ax2.set_title('Medium sized mission')

        ax3.plot(iterations, medium_data['actions_per_iter'], 'g', label='actions', linestyle='--')
        ax3.set_ylabel('actions')

        blue_line = mlines.Line2D([], [], color='r', label='reward')
        reds_line = mlines.Line2D([], [], color='g', label='actions', linestyle='--')


        handles = [blue_line, reds_line]
        labels = [h.get_label() for h in handles]
        plt.subplots_adjust(bottom=0.2)
        plt.subplots_adjust(wspace=0.5)

        plt.legend(handles, labels, loc='upper center',  bbox_to_anchor=(0.5,-0.05), shadow=True, ncol=4)

        # plt.tight_layout()
        plt.show()