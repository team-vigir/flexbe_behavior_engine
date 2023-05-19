#!/usr/bin/env python

from flexbe_core import EventState, Logger

'''
Created on 14-Feb-2018

@author: David Conner
'''

class UserDataState(EventState):
    '''
    Implements a state that defines user data

    -- data    type        Data for given user data

    #> data                User data

    <= done                Created the user data
    '''


    def __init__(self, data ):
        '''
        Constructor
        '''
        super(UserDataState, self).__init__( output_keys=["data"], outcomes=["done"])

        self._my_data = data
        self._return_code = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        return self._return_code


    def on_enter(self, userdata):

        try:
          # Add the user data
          userdata.data = self._my_data
          self._return_code = 'done'
        except:
            raise ValueError('UserDataState %s - invalid data ' % self.name)
