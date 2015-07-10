# Describes a single instantiation of a state

# Path of this state inside the behavior
# Last segment of the path is the name of this state
# e.g. "/Inner_Statemachine/This_State"
string state_path

# Specifies the class implementing this state
# Use one of the provided CLASS constants if this is not a primitive state
# e.g. "CalculationState"
string CLASS_STATEMACHINE 	= :STATEMACHINE
string CLASS_BEHAVIOR 		= :BEHAVIOR
string state_class



# Only relevant for state machines
# Name of the initial state
string initial_state_name

# Only relevant for behaviors
# Name of the class implementing this state
string behavior_class

# Only relevant for primitive states
# List of parameter names and values of this state
# Order of names and values has to match
string[] parameter_names
string[] parameter_values



# Outcomes of this state
# Defines the order of the lists: transitions, autonomy
string[] outcomes
# Transition targets (state names or container outcomes) of the outcomes
string[] transitions
# Required autonomy level of the outcomes
uint8[] autonomy

# Userdata of this state
# Merges own input_keys and output_keys, no duplicates
string[] userdata_keys
# Remapping of the corresponding own key (= key used by container)
string[] userdata_remapping