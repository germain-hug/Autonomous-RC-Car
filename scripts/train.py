#! /usr/bin/env python

"""
Keras training script
"""

import os
import sys
import glob

class KerasTrainer(object):
	def __init__(self):
        # Data path
		self.data_path = '/data/'
        self.bs=64
        self.no_of_epochs=1
        self.build_network()
        self.prepare_batches()
        self.train()

	def train(self):

    def build_network(self):

    def prepare_batches(self):
        # Create Image Generator
        self.gen = image.ImageDataGenerator(
            height_shift_range=0.05,
            shear_range=0.1,
            channel_shift_range=20)
        # Get data
        self.X = self.get_data(self.data_path, self.gen, bs=self.bs, shuffle=False)
        # Read annotations
        fnames = glob.glob("/data/*/*.csv")

        


    def get_data(self, dirname, gen, shuffle, bs, size=(224,224)):
        batches = gen.flow_from_directory(dirname,
            target_size = size,
            class_mode = None,
            shuffle = shuffle,
            batch_size = batch_size)
	    return np.concatenate([batches.next() for i in range(batches.nb_sample)])
