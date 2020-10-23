#!/usr/bin/env python3

# Copyright (c) 2019 Matthias Rolf, Oxford Brookes University

'''

'''

import numpy as np
from math import pi, cos, sin
from matplotlib import pyplot as plt

class Gaussian:
	# Create Gaussian from mean vector and covariance matrix
	def __init__(self, meanP: np.array, varP: np.array):
		self.mean = meanP
		self.var = varP

	# Estimate empirical Gaussian from data (stored in rows of a matrix)
	@classmethod
	def fromData(cls,rowData: np.matrix):
		m = np.mean(rowData,axis=0)
		if np.size(rowData,0) > 1:
			v = np.cov(rowData,rowvar=False)
		else:
			v = np.zeros([np.size(rowData,1),np.size(rowData,1)])
		f = cls(m,v)
		return f

	def mult(self, other):
		innerVar = np.add(self.var, other.var)
		innerVar = np.linalg.inv(innerVar)
		newVar = np.matmul(np.matmul(self.var, innerVar), other.var)
		leftMean = np.matmul(np.matmul(self.var, innerVar), other.mean)
		rightMean = np.matmul(np.matmul(other.var, innerVar), self.mean)
		newMean =  np.add(leftMean, rightMean)
		return Gaussian(newMean,newVar)

	def add(self, other):
		return Gaussian(np.add(self.mean,other.mean),np.add(self.var,other.var))
        
	# create 'size' new random samples from distribution
	def sample(self, size=1):
		return np.random.multivariate_normal(self.mean,self.var,size)

	def __str__(self):
		return "Gaussian: \nMean:\n"+ str(self.mean) + "\nVariance:\n"+ str(self.var) 

class GaussianTable:
	# Create Gaussian from mean vector and covariance matrix
	def __init__(self, meanP: np.array, varP: np.array, numEntriesP):
		self.mean = meanP
		self.var = varP
		self.numEntries = numEntriesP
		self.table = np.zeros([meanP.size, numEntriesP])
		for i in range(0,numEntriesP):
			self.table[:,i] = np.random.multivariate_normal(self.mean,self.var)
		self.currentIndex = 0
        
	# create 'size' new random samples from distribution
	def sample(self):
		self.currentIndex = self.currentIndex+1
		if self.numEntries <= self.currentIndex:
			self.currentIndex = 0
		return self.table[:,self.currentIndex]

	def __str__(self):
		return "Gaussian look-up table: \nMean:\n"+ str(self.mean) + "\nVariance:\n"+ str(self.var) 

# plot gaussian as ellipse on a matplotlib plot
def plotGaussian(g:Gaussian, color="red", existingPlot=None):
	x = g.mean.reshape(-1)[0]
	y = g.mean.reshape(-1)[1]
	var = g.var[0:2,0:2]
	try:
		L = np.linalg.cholesky(var)
	except np.linalg.LinAlgError:
		L = np.zeros([2,2])
	num=100
	t = np.linspace(0, 2*pi, num)
	xy=np.zeros([num,2])
	for i in range(0,num):
		pos = np.array([cos(t[i]), sin(t[i])])
		pos = np.matmul(L,pos) + np.array([x,y])
		xy[i,0] = pos[0]
		xy[i,1] = pos[1]
	if existingPlot is not None:
		existingPlot.set_xdata(xy[:,0])
		existingPlot.set_ydata(xy[:,1])
		existingPlot.set_color(color)
		return existingPlot
	else:
		line = plt.plot(xy[:,0],xy[:,1], color)
		return line[0]


def testGaussian():
	m = np.array([0.0, 10.0])
	v = np.array([[1.0, 1], [1, 2.0]])

	g = Gaussian(m,v)

	#print(g.sample(10))

	zero = np.array([0.0, 0.0])
	x1 = np.array([1.0, 0.0])
	y1 = np.array([0.0, 1.0])

	vNorm = np.array([[1.0, 0], [0, 1.0]])
	vX = np.array([[10.0, 0], [0, 0.1]])
	vY = np.array([[0.1, 0], [0, 10.0]])
	covPos = np.array([[1.0, 0.7], [0.7, 1.0]])
	covNeg = np.array([[1.0, -0.7], [-0.7, 1.0]])

	g1 = Gaussian(zero,vNorm)

	g = g1.mult(g1)
	print(g.mean)
	print(g.var)

	g = Gaussian(x1,vY).mult(Gaussian(y1,vX))
	print(g.mean)
	print(g.var)

	g = Gaussian(zero,covPos).mult(Gaussian(zero,covNeg))
	print(g.mean)
	print(g.var)

	g = Gaussian(zero,covPos).mult(Gaussian(x1,vY))
	print(g.mean)
	print(g.var)


