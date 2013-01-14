#!/usr/bin/env python

'''
classes for sensor noise
'''

import random

class GaussianNoise(object):
    
    def __init__(self, mean, variance):
        self.mean = mean
        self.variance = variance
 
    def get_noise(self):
        return random.gauss(self.mean, self.variance)

    def __add__(self, x):
        return x + self.get_noise()

    def __radd__(self, x):
        return x + self.get_noise()
