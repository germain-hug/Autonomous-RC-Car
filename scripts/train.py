#! /usr/bin/env python

"""
Keras training script
"""

import os
import sys
import glob

from keras.layers import *
from numpy import genfromtxt

class KerasTrainer(object):
	def __init__(self):
        # Data path
		self.data_path = 'data/'
        self.bs=64
        self.no_of_epochs=1
        self.prepare_data()
        self.build_network()
        self.train()

	def train(self):
        self.model.fit(self.X, self.Y, batch_size=self.bs, nb_epoch=self.no_of_epochs)
        self.model.save("models/model.h5")

    def build_network(self):
        img_input = Input(self.img_shape)
        x = BatchNormalization(axis=1)(img_input)
        x = incep_block(x)
        x = incep_block(x)
        x = incep_block(x)
        x = Dropout(0.6)(x)
        x = Convolution2D(8,3,3, border_mode='same')(x)
        output = Dense(2, activation='tanh')(x)
        self.model = Model(img_input, output)

    def prepare_data(self):
        fnames = glob.glob("/data/*/*.csv")
        self.X = [], self.Y = []
        for f in fnames:
            annotations = genfromtxt(f, delimiter=',')
            x = np.array([np.array(Image.open(fname)) for fname in annotations[:,0]])
            y = [np.array(label) for label in annotations[:,1:2]])
            self.X.append(x)
            self.Y.append(y)

    def conv_block(self, x, nb_filter, nb_row, nb_col, subsample=(1, 1)):
        """ 2D Convolution + ReLU + BatchNorm block
        """
        x = Convolution2D(nb_filter, nb_row, nb_col, subsample=subsample, activation='relu', border_mode='same')(x)
        return BatchNormalization(axis=1)(x)

    def incep_block(self, x):
        """ Inception Block
        """
        branch1x1 = conv_block(x, 32, 1, 1, subsample=(2, 2))
        branch5x5 = conv_block(x, 24, 1, 1)
        branch5x5 = conv_block(branch5x5, 32, 5, 5, subsample=(2, 2))
        branch3x3dbl = conv_block(x, 32, 1, 1)
        branch3x3dbl = conv_block(branch3x3dbl, 48, 3, 3)
        branch3x3dbl = conv_block(branch3x3dbl, 48, 3, 3, subsample=(2, 2))
        branch_pool = AveragePooling2D((3, 3), strides=(2, 2), border_mode='same')(x)
        branch_pool = conv_block(branch_pool, 16, 1, 1)
        return merge([branch1x1, branch5x5, branch3x3dbl, branch_pool], mode='concat', concat_axis=1)
