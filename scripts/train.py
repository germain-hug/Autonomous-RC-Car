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
		self.Y = []
		for f in fnames:
			annotations = genfromtxt(f, delimiter=',', dtype=object)
			self.Y.append([np.array(label, dtype=np.float32) for label in annotations[:,1:]])
		print(np.asarray(self.Y))
		print(np.asarray(self.Y).shape)

	def train(self):
		self.prepare_data()
		self.model.fit(self.X, self.Y, batch_size=self.batch_size, nb_epoch=self.no_of_epochs)
		self.model.save("../models/model.h5")
