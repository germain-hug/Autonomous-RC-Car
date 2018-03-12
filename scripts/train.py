#! /usr/bin/env python

import os
import sys
import glob
import numpy as np

from utils import network
from numpy import genfromtxt
from PIL import Image

class Trainer(object):
	""" Train a Keras model on all captured data
	"""
	def __init__(self):
        # Data path
		self.data_path = 'data/'
		self.batch_size = 64
		self.no_of_epochs = 1
		self.img_shape = (480, 640, 3)
		self.model = network(self.img_shape)

	def prepare_data(self):
		fnames = glob.glob("../data/*/*.csv")
		print(fnames)
		self.X, self.Y = [], []
		for f in fnames:
			annotations = genfromtxt(f, delimiter=',', dtype="|U5")
			print(annotations.shape)
			print(annotations[0:10,0:])
			x = np.array([np.array(Image.open(f[:-15] + '_img' + fname)) for fname in annotations[:,0]])
			y = [np.array(label) for label in annotations[:,1:2]]
			print(y)
			self.X.append(x)
			self.Y.append(y)

	def train(self):
		self.prepare_data()
		self.model.fit(self.X, self.Y, batch_size=self.batch_size, nb_epoch=self.no_of_epochs)
		self.model.save("../models/model.h5")
