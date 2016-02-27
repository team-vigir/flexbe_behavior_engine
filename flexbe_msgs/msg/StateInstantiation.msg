# Describes a single instantiation of a state

# Path of this state inside the behavior
# Last segment of the path is the name of this state
# e.g. "/Inner_Statemachine/This_State"
string state_path

# Specifies the class implementing this state
# Use one of the provided CLASS constants if this is not a primitive state
# e.g. "CalculationState"
string CLASS_STATEMACHINE 	= :STATEMACHINE
string CLASS_CONCURRENCY 	= :CONCURRENCY
string CLASS_PRIORITY 		= :PRIORITY
string CLASS_BEHAVIOR 		= :BEHAVIOR
string state_class



# Only relevant for state machine, priority, and concurrency
# Name of the initial state / state to be monitored
string initial_state_name
# Input and output keys (can be remapped below)
string[] input_keys
string[] output_keys

# Only relevant for concurrency
# Outcome conditions
string[] cond_outcome
OutcomeCondition[] cond_transition

# Only relevant for behavior
# Name of the class implementing this state
string behavior_class

# Only relevant for primitive state
# List of parameter names and values of this state
# Order of names and values has to match
string[] parameter_names
string[] parameter_values



# Position of this state in the editor [x,y]
float32[2] position

# Outcomes of this state
# Defines the order of the lists: transitions, autonomy
string[] outcomes
# Transition targets (state names or container outcomes) of the outcomes
string[] transitions
# Required autonomy level of the outcomes
int8[] autonomy

# Userdata of this state
# Merges own input_keys and output_keys, no duplicates
string[] userdata_keys
# Remapping of the corresponding own key (= key used by container)
string[] userdata_remapping