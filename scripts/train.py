#! /usr/bin/env python

import os
import sys
import glob

from utils import network
from numpy import genfromtxt

class Trainer(object):
	""" Train a Keras model on all captured data
	"""
	def __init__(self):
        # Data path
		self.data_path = 'data/'
        self.bs = 64
        self.no_of_epochs = 1
        self.img_shape = (224, 224, 3)
        self.model = network(self.img_shape)

	def train(self):
        self.prepare_data()
        self.model.fit(self.X, self.Y, batch_size=self.bs, nb_epoch=self.no_of_epochs)
        self.model.save("models/model.h5")

	def prepare_data(self):
		fnames = glob.glob("/data/*/*.csv")
        self.X = [], self.Y = []
        for f in fnames:
            annotations = genfromtxt(f, delimiter=',')
            x = np.array([np.array(Image.open(fname)) for fname in annotations[:,0]])
            y = [np.array(label) for label in annotations[:,1:2]])
            self.X.append(x)
            self.Y.append(y)
