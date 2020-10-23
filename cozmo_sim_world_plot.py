


from cozmo_sim_world import *
from map import *

from matplotlib import pyplot as plt


def plotRobot(pos:Frame2D, color="orange", existingPlot=None):
	xy = np.array([[30, 35,  35,  30, -30, -30, 30],
	               [20, 15, -15, -20, -20,  20, 20],
	               [ 1,  1,   1,   1,   1,   1,  1]])
	xy = np.matmul(pos.mat, xy)
	if existingPlot is not None:
		existingPlot.set_xdata(xy[0,:])
		existingPlot.set_ydata(xy[1,:])
		existingPlot.set_color(color)
		return existingPlot
	else:
		line = plt.plot(xy[0,:],xy[1,:], color)
		return line[0]

def plotLandmark(pos:Frame2D, color="orange", existingPlot=None):
	xy = np.array([[25, -25, -25,  25, 25],
	               [25,  25, -25, -25, 25],
	               [ 1,  1,   1,   1,   1]])
	if pos.x() == 0 and pos.y() == 0:
		xy = np.matmul(Frame2D.fromXYA(-1000,-1000,0).mat, xy)
	else:
		xy = np.matmul(pos.mat, xy)
	if existingPlot is not None:
		existingPlot.set_xdata(xy[0,:])
		existingPlot.set_ydata(xy[1,:])
		existingPlot.set_color(color)
		return existingPlot
	else:
		line = plt.plot(xy[0,:],xy[1,:], color)
		return line[0]


def runPlotLoop(m: CozmoMap, simWorld: CozmoSimWorld, finished):

	# create plot
	plt.ion()
	plt.show()
	fig = plt.figure(figsize=(8, 8))
	ax = fig.add_subplot(1, 1, 1, aspect=1)

	ax.set_xlim(m.grid.minX(), m.grid.maxX())
	ax.set_ylim(m.grid.minY(), m.grid.maxY())

	plotMap(ax,m)
	
	robotPlot = plotRobot(simWorld.dont_touch__pos())
	cube1Plot = plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube1Id))
	cube2Plot = plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube2Id))
	cube3Plot = plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube3Id))
 
	# main loop
	t = 0
	while not finished.is_set():

		plotRobot(simWorld.dont_touch__pos(), existingPlot=robotPlot)
		plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube1Id), existingPlot=cube1Plot)
		plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube2Id), existingPlot=cube2Plot)
		plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube3Id), existingPlot=cube3Plot)

		plt.draw()
		plt.pause(0.001)
		
		time.sleep(0.01)


