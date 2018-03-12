#! /usr/bin/env python

import os
import sys
import glob
import numpy as np

from utils import network
from numpy import genfromtxt
from PIL import Image
from tqdm import tqdm
from keras.optimizers import Adam

class Trainer(object):
	""" Train a Keras model on all captured data
	"""
	def __init__(self):
        # Data path
		self.data_path = 'data/'
		self.bs = 64
		self.no_of_epochs = 1
		self.img_shape = (480, 640, 3)
		self.model = network(self.img_shape)

	def prepare_data(self):
		fnames = glob.glob("../data/*/*.csv")
		self.Y = np.array([]).reshape(0,2) # Steering, Throttle
		self.X = np.array([]).reshape(0,1) # Filenames
		for f in fnames:
			annotations = genfromtxt(f, delimiter=',', dtype=object)
			labels = [np.array(label, dtype=np.float32)  for label in annotations[:,1:]]
			images = [np.array(f[:-15] + 'img_' + fname) for fname in annotations[:,0]]
			self.Y = np.vstack([self.Y, (np.reshape(np.array(labels),(-1, 2)))])
			self.X = np.vstack([self.X, (np.reshape(np.array(images),(-1, 1)))])

	def train(self):
		self.prepare_data()
		self.model.compile(Adam(0.01), 'mean_squared_error')
		# Shuffle training samples
		perms = np.random.permutation(self.X.shape[0])
		self.X = self.X[perms]
		self.Y = self.Y[perms]
		# Begin training
		for e in range(self.no_of_epochs):
			for i in tqdm(range(self.X.shape[0]//self.bs)):
				images = [np.array(Image.open(f[0])) for f in self.X[i*self.bs:(i+1)*self.bs]]
				images = np.divide(np.array(images, dtype=np.float32), 255.0)
				labels = self.Y[i*self.bs:(i+1)*self.bs]
				self.model.fit(images, labels, batch_size=self.bs, nb_epoch=1, verbose=0)
		self.model.save("../models/model.h5")
