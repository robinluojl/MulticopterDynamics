import numpy as np
import matplotlib.pyplot as plt
import os
import copy



class Plot_xy():

    def __init__(self, fontsize):
        self.fontsize = fontsize
        self.fig = None
        self.ax = None
        self.xlabel = ''
        self.ylabel = ''
        self.curves = []

    def createPlot(self, title, xlabel, ylabel):

        self.fig, (self.ax) = plt.subplots(1, 1, sharex=True)
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.fig.subplots_adjust(hspace=0.5)
        self.ax.set_title(title, fontweight='bold', fontsize=self.fontsize)

    def add(self, curve):
        self.curves.append(curve)

    def plot(self):
        for c in self.curves:
            c.plot(self.ax)
        self.finalizePlot()

    def finalizePlot(self, xmin=None, xmax=None, ymin=None, ymax=None,
        export_dir='', output_file='', show_legend=True, legend_type='outer_left',
        logscale_x=False, logscale_y=False, show=True, tick_fontsize=None):

        if tick_fontsize is None:
            tick_fontsize = self.fontsize

        if xmin is None or xmax is None:
            if not (ymin is None or ymax is None):
                self.ax.set_ylim([ymin, ymax])
        elif ymin is None or ymax is None:
            if not (xmin is None or xmax is None):
                self.ax.set_xlim([xmin, xmax])
        else:
            self.ax.axis([xmin, xmax, ymin, ymax])

        if show_legend:
            handles, labels = self.ax.get_legend_handles_labels()
            if legend_type == 'outer_top':
                self.ax.legend(handles, labels, prop={'size': self.fontsize}, \
                bbox_to_anchor=(0., 1.02, 1., .102), loc=3, mode="expand", frameon=False)
            elif legend_type == 'outer_right':
                self.ax.legend(handles, labels, prop={'size': self.fontsize}, \
                    bbox_to_anchor=(1.02, 1., 1., .102), loc=2, mode="expand", frameon=False)
            elif legend_type == 'outer_left':
                self.ax.legend(handles, labels, prop={'size': self.fontsize}, \
                    bbox_to_anchor=(0., 1.02, 1., .102), loc='upper left', mode="expand", frameon = False)
            elif legend_type == 'basic':
                self.ax.legend(handles, labels, prop={'size': self.fontsize}, loc=0)
            else:
                print ('no legend shown')

        if logscale_x is True:
            self.ax.set_xscale('log')

        if logscale_y is True:
            self.ax.set_yscale('log')

        self.ax.set_xlabel(self.xlabel, fontsize=self.fontsize)
        self.ax.set_ylabel(self.ylabel, fontsize=self.fontsize)

        gridlines = self.ax.get_xgridlines()
        gridlines.extend(self.ax.get_ygridlines())
        for line in gridlines:
            line.set_linestyle('--')

        ticklabels = self.ax.get_xticklabels()
        ticklabels.extend(self.ax.get_yticklabels())
        for label in ticklabels:
            label.set_color('k')
            label.set_fontsize(tick_fontsize)

        if not (export_dir == '' or output_file == ''):
            self.fig.savefig(os.path.join(export_dir, output_file), dpi=400)

        if show:
            plt.show()


class Curve_xy():

    def __init__(self, initial_data):
        for key in initial_data:
            setattr(self, key, initial_data[key])

        self.x = np.empty(self.nbIte)
        self.y = np.empty(self.nbIte)

    def plot(self, ax):
        ax.plot(self.x, self.y / self.yAdim,
             self.style, label=self.label)