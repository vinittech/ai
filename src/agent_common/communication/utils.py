"""
Utility function for communication.py to use for general purpose.

__author__ = '{Vinit Jain}'
__project__='{AAIP Project SS19}'
"""
import numpy as np

def numpy_array_to_csv_string(a):
    '''
    a: numpy array
    returns comma separated string of values in a
    '''
    return ",".join([str(el) for el in a.flatten()])