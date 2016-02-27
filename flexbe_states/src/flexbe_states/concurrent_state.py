#!/usr/bin/env python

import rospy
import smach

from flexbe_core import EventState, Logger, Behavior

'''
Created on 03/09/2015

@author: Philipp Schillinger
'''

class ConcurrentState(EventState):
    '''
    DEPRECATED.
    Implements concurrent execution of states.
    Depending on the outcomes of the single states, the outcome of this state is determined according to the given set of rules.

    -- states           dict    A dictionary containing all states to be executed.
                                Key is any unique name of the state as string, value is the state instantiation such as WaitState(wait_time=2).

    -- outcomes         list    A list of strings of all possible outcomes of this state.

    -- outcome_mapping  list    Rules for mapping the inner outcomes to one outcome of this state.
                                Each list entry is a dictionary containing two entries: 'outcome' refers to one of the outcomes, 'condition' is a dictionary.
                                Each condition dictionary contains one entry for each relevant state, mapping the name of the state to one of its outcomes.
                                States which are not relevant for this condition can be skipped and may be preempted if a condition is fulfilled.
                                Outcomes can be represented multiple times in the mapping.
    '''


    def __init__(self, states, outcomes, outcome_mapping):
        '''
        Constructor
        '''
        for name, state in states.items():
            if isinstance(state, Behavior):
                states[name] = state._get_muted_state_machine()
            else:
                state._is_controlled = False
            state.name = name

        super(ConcurrentState, self).__init__(outcomes=outcomes,
                                               input_keys=sum(map(lambda n: list(n + "_" + ik for ik in states[n]._input_keys), states.keys()), []),
                                               output_keys=sum(map(lambda n: list(n + "_" + ok for ok in states[n]._output_keys), states.keys()), []))
        
        if isinstance(outcome_mapping, basestring):
            mapping = list()
            outcome_mapping = outcome_mapping.replace(" ", "")
            for oc,cond in [el.split("=>")[0:2] for el in outcome_mapping.split(",")]:
                for conj in cond.split("||"):
                    mapping_entry = dict()
                    mapping_entry['outcome'] = oc
                    mapping_entry['condition'] = dict()
                    for el in conj.split("&&"):
                        pair = el.split(":")
                        mapping_entry['condition'][pair[0]] = pair[1]
                    mapping.append(mapping_entry)
            outcome_mapping = mapping

        self._states = states
        self._outcomes = outcomes
        self._outcome_mapping = outcome_mapping
        self._returned_outcomes = dict()

        if not all(element['outcome'] in outcomes for element in outcome_mapping):
            raise Exception("Not all outcomes are available in %s" % (self.name))

        rospy.loginfo(str(self._input_keys))
        rospy.loginfo(str(self._output_keys))
        
        
    def execute(self, userdata):
        # execute all active states
        for name, state in self._states.items():
            if self._returned_outcomes.has_key(name): continue
            outcome = None
            if isinstance(state, smach.StateMachine):
                rmp_ud = smach.Remapper(
                            userdata,
                            state.get_registered_input_keys(),
                            state.get_registered_output_keys(),
                            {key: name + '_' + key for key in state.get_registered_input_keys() + state.get_registered_output_keys()})
                state.userdata.merge(rmp_ud, rmp_ud.keys(), dict())
                with state._state_transitioning_lock:
                    outcome = state._update_once()
            else:
                outcome = state.execute(smach.Remapper(
                            userdata,
                            state.get_registered_input_keys(),
                            state.get_registered_output_keys(),
                            {key: name + '_' + key for key in state.get_registered_input_keys() + state.get_registered_output_keys()}))
            if outcome is not None and outcome != "loopback":
                Logger.loginfo("Concurrency: %s > %s" % (name, outcome))
                self._returned_outcomes[name] = outcome

        # determine outcome
        for element in self._outcome_mapping:
            state_outcome = element['outcome']
            mapping = element['condition']
            if all(name in self._returned_outcomes and self._returned_outcomes[name] is outcome for name, outcome in mapping.items()):
                return state_outcome



    def on_enter(self, userdata):
        self._returned_outcomes = dict()

        for name, state in self._states.items():
            if isinstance(state, smach.StateMachine): continue
            state.on_enter(smach.Remapper(
                        userdata,
                        state.get_registered_input_keys(),
                        state.get_registered_output_keys(),
                        {key: name + '_' + key for key in state.get_registered_input_keys() + state.get_registered_output_keys()}))
            state._entering = False


    def on_exit(self, userdata):
        for name, state in self._states.items():
            if isinstance(state, smach.StateMachine): continue
            state.on_exit(smach.Remapper(
                        userdata,
                        state.get_registered_input_keys(),
                        state.get_registered_output_keys(),
                        {key: name + '_' + key for key in state.get_registered_input_keys() + state.get_registered_output_keys()}))
            state._entering = True

    def on_start(self):
        for state in self._states.values():
            if isinstance(state, smach.StateMachine):
                state._set_current_state(state._initial_state_label)
                state._notify_start()
            else:
                state.on_start()

    def on_stop(self):
        for state in self._states.values():
            if isinstance(state, smach.StateMachine):
                state._notify_stop()
            else:
                state.on_stop()


    # deprecated, use smach.Remapper instead
    def _convert_userdata(self, state, userdata):
        inner_userdata = smach.UserData()
        for key in list(self._input_keys):
            if key.startswith(state.name + "_"):
                inner_userdata[key.replace(state.name + "_", "")] = userdata[key]
        return inner_userdata
    # deprecated, use smach.Remapper instead
    def _merge_userdata_back(self, state, inner_userdata, userdata):
        output_userdata = smach.UserData()
        for key in list(self._output_keys):
            if key.startswith(state.name + "_"):
                output_userdata[key] = inner_userdata[key.replace(state.name + "_", "")]
        userdata.update(output_userdata)