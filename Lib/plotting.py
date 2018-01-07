import numpy as np
import matplotlib.pyplot as plt
import os
import copy



################################################################
def finalize_plot(fig, ax, xmin=None, xmax=None, ymin=None, ymax=None, xlabel='', ylabel='', fontsize=10, export_dir='', output_file='', \
                  show_legend=True, legend_type='basic', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None):

    if tick_fontsize is None:
        tick_fontsize = fontsize

    if xmin is None or xmax is None:
        if not (ymin is None or ymax is None):
            ax.set_ylim([ymin, ymax])
    elif ymin is None or ymax is None:
        if not (xmin is None or xmax is None):
            ax.set_xlim([xmin, xmax])
    else:
        ax.axis([xmin, xmax, ymin, ymax])

    if show_legend:
        handles, labels = ax.get_legend_handles_labels()
        if legend_type == 'outer_top':
            ax.legend(handles, labels, prop={'size': fontsize}, \
            bbox_to_anchor=(0., 1.02, 1., .102), loc=3, mode="expand", frameon=False)
        elif legend_type == 'outer_right':
            ax.legend(handles, labels, prop={'size': fontsize}, \
                bbox_to_anchor=(1.02, 1., 1., .102), loc=2, mode="expand", frameon=False)
        elif legend_type == 'outer_left':
            ax.legend(handles, labels, prop={'size': fontsize}, \
                bbox_to_anchor=(0., 1.02, 1., .102), loc='upper left', mode="expand", frameon = False)
        elif legend_type == 'basic':
            ax.legend(handles, labels, prop={'size': fontsize}, loc=0)
        else:
            print ('no legend shown')

    if logscale_x is True:
        ax.set_xscale('log')

    if logscale_y is True:
        ax.set_yscale('log')

    ax.set_xlabel(xlabel, fontsize=fontsize)
    ax.set_ylabel(ylabel, fontsize=fontsize)

    gridlines = ax.get_xgridlines()
    gridlines.extend(ax.get_ygridlines())
    for line in gridlines:
        line.set_linestyle('--')

    ticklabels = ax.get_xticklabels()
    ticklabels.extend(ax.get_yticklabels())
    for label in ticklabels:
        label.set_color('k')
        label.set_fontsize(tick_fontsize)

    if not (export_dir == '' or output_file == ''):
        fig.savefig(os.path.join(export_dir, output_file), dpi=400)

    if show:
        plt.show()
