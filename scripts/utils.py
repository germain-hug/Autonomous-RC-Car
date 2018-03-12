from keras.layers import *
from keras.models import Model

def network(img_shape):
    """ Small Keras Inception Network
    """
    img_input = Input(img_shape)
    x = BatchNormalization(axis=1)(img_input)
    x = incep_block(x)
    x = incep_block(x)
    x = Dropout(0.6)(x)
    x = Flatten()(x)
    output = Dense(2, activation='tanh')(x)
    return Model(img_input, output)

def conv_block(x, nb_filter, nb_row, nb_col, subsample=(1, 1)):
    """ 2D Convolution + ReLU + BatchNorm block
    """
    x = Convolution2D(nb_filter, nb_row, nb_col, subsample=subsample, activation='relu', border_mode='same')(x)
    return BatchNormalization(axis=1)(x)

def incep_block(x):
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
    return merge([branch1x1, branch5x5, branch3x3dbl, branch_pool], mode='concat', concat_axis=3)
