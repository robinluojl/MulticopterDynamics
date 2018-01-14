from Lib.plotting import *

class Post():

	def __init__(self, fontsize):
		self.fontsize = fontsize
		self.plt = None

	def createXYPlot(self, nbSubplot, styles, subPlotIndex, results):
		self.plt = Plot_xy(self.fontsize)
		self.plt.createPlot('', '', '', nbSubplot)
		for i, r in enumerate(results):
			curve = Curve_xy(r, styles[i], subPlotIndex[i])
			self.plt.add(curve)