import numpy as np
import matplotlib.pyplot as plt
import os
import copy
from datetime import datetime
from matplotlib.animation import FuncAnimation
from random import randrange
from matplotlib import animation
from matplotlib import colors as c



class Plot_xy():

    def __init__(self, fontsize):
        self.fontsize = fontsize
        self.fig = None
        self.ax = None
        self.xlabel = ''
        self.ylabel = ''
        self.curves = []

    def createPlot(self, title, xlabel, ylabel, nbSubplotVert):

        self.fig, (self.ax) = plt.subplots(nbSubplotVert, 1, sharex=True)
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.fig.subplots_adjust(hspace=0.5)
        for sub in self.ax:
            sub.set_title(title, fontweight='bold', fontsize=self.fontsize)

    def add(self, curve):
        self.curves.append(curve)

    def set(self):
        for c in self.curves:
            c.set()

    def plot(self):
        for c in self.curves:
            for i, sub in enumerate(self.ax):
                if c.subplotIndex == i:
                    c.plot(sub)

        for i in range(len(self.ax)-1):
            self.finalizePlot(self.fig, self.ax[i], self.fontsize, self.xlabel, self.ylabel, show=False)
        self.finalizePlot(self.fig, self.ax[-1], self.fontsize, self.xlabel, self.ylabel, show=True)

    def finalizePlot(self, fig, ax, fontsize, xlabel, ylabel, xmin=None, xmax=None, ymin=None, ymax=None,
        export_dir='', output_file='', show_legend=True, legend_type='outer_left',
        logscale_x=False, logscale_y=False, show=True, tick_fontsize=None):

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


class Curve_xy():

    def __init__(self, result_xy, style, subplotIndex):
        self.result_xy = result_xy
        self.style = style
        self.subplotIndex = subplotIndex
        self.x = []
        self.y = []

    def set(self):
        self.x.append(self.result_xy.x)
        self.y.append(self.result_xy.y)

    def plot(self, ax):
        ax.plot(np.array(self.x), np.array(self.y) / self.result_xy.adim,
             self.style, label=self.result_xy.label)





# x_data, y_data = [], []

# figure = pyplot.figure()
# line, = pyplot.plot_date(x_data, y_data, '-')

# def update(frame):
#     x_data.append(datetime.now())
#     y_data.append(randrange(0, 100))
#     line.set_data(x_data, y_data)
#     figure.gca().relim()
#     figure.gca().autoscale_view()
#     return line,



# import numpy as np
# import time
# import matplotlib
# matplotlib.use('GTKAgg')


# def randomwalk(dims=(256, 256), n=20, sigma=5, alpha=0.95, seed=1):
#     """ A simple random walk with memory """

#     r, c = dims
#     gen = np.random.RandomState(seed)
#     pos = gen.rand(2, n) * ((r,), (c,))
#     old_delta = gen.randn(2, n) * sigma

#     while True:
#         delta = (1. - alpha) * gen.randn(2, n) * sigma + alpha * old_delta
#         pos += delta
#         for ii in range(n):
#             if not (0. <= pos[0, ii] < r):
#                 pos[0, ii] = abs(pos[0, ii] % r)
#             if not (0. <= pos[1, ii] < c):
#                 pos[1, ii] = abs(pos[1, ii] % c)
#         old_delta = delta
#         yield pos


# def run(niter=1000, doblit=True):
#     """
#     Display the simulation using matplotlib, optionally using blit for speed
#     """

#     fig, ax = plt.subplots(1, 1)
#     ax.set_aspect('equal')
#     ax.set_xlim(0, 255)
#     ax.set_ylim(0, 255)
#     rw = randomwalk()
#     x, y = rw.__next__()

#     plt.show(False)
#     plt.draw()

#     if doblit:
#         # cache the background
#         background = fig.canvas.copy_from_bbox(ax.bbox)

#     points = ax.plot(x, y, 'o')[0]
#     tic = time.time()

#     for ii in range(niter):

#         # update the xy data
#         x, y = rw.__next__()
#         points.set_data(x, y)

#         if doblit:
#             # restore background
#             fig.canvas.restore_region(background)

#             # redraw just the points
#             ax.draw_artist(points)

#             # fill in the axes rectangle
#             fig.canvas.blit(ax.bbox)

#         else:
#             # redraw everything
#             fig.canvas.draw()

#     plt.close(fig)
#     print ("Blit = %s, average FPS: %.2f" % (str(doblit), niter / (time.time() - tic)))




# world_size = 80 # The circles appear on a 59 X 59 grid
# nbFrames = 100
# circle_colors = np.empty((world_size, world_size, nbFrames), 'str') #And this is loading a 3D array
# for n in range(nbFrames):
#     for i in range(circle_colors.shape[1]):
#         for j in range(circle_colors.shape[0]):
#             circle_colors[j, i, n] = '0.5'

# #Each XY coordinate contains a list of RGB colors indicating the series
# #of colors the circle should take on in each frame (so the length of
# #these lists is equal to the number of frames)


# #Create figure to do plotting on
# fig = plt.figure(figsize=(20,20))

# world = np.zeros((world_size, world_size, 3), 'uint8') #Don't worry about this it's just
# #loading a 2D array of numbers to pass to imshow() to make the background

# #Create list of circles, one for every grid cell in the environment
# patches = []

# for i in range(world_size):
#     for j in range(world_size):
#         patches.append(plt.Circle((j,i), radius=.3, lw=2, ec="black", facecolor=None))
#         #(j,i) are the (x,y) coordinates, lw is the line width, ec is the color of the
#         #outline, setting facecolor to None makes these circles invisible for now

# def init():
#     #Draw background
#     plt.imshow(world, interpolation="none", hold=True)
#     axes = plt.gca()
#     axes.autoscale(False)

#     #Add patches to axes
#     for p in patches:
#         axes.add_patch(p)

#     return patches

# def animate(n):
#     #Recolor circles
#     #Note that this needs to be in the same scope as circle_colors

#     for i in range(world_size):
#         for j in range(world_size):

#             if circle_colors[i][j][n] == 0:
#                 #Here, I'm using 0 as code for non-existent
#                 #So I make these circles invisible
#                 patches[i * world_size + j].set_visible(False)

#             else: #Otherwise we set the color to the one indicated by circle_colors
#                 #and make sure the circle is set to visible
#                 patches[i*world_size + j].set_facecolor(circle_colors[i][j][n])
#                 patches[i*world_size + j].set_visible(True)

#     #We haven't actually changed patches, but FuncAnimation is
#     #still expecting us to return it. Don't forget the comma.
#     return patches,



#remember to set blit=True

# anim.save(filename, writer="mencoder", fps=2) #You can also save your animation!
#Appropriate writer will vary based on your computer


"""
A simple example of an animated plot
"""

# fig, ax = plt.subplots()

# x = np.arange(0, 2*np.pi, 0.01)
# line, = ax.plot(x, np.sin(x))


# def animate(i):
#     line.set_ydata(np.sin(x + i/10.0))  # update the data
#     return line,


# # Init only required for blitting to give a clean slate.
# def init():
#     line.set_ydata(np.ma.array(x, mask=True))
#     return line,

# ani = animation.FuncAnimation(fig, animate, np.arange(1, 200), init_func=init,
#                               interval=25, blit=True)
# # plt.show()