# Minimal template for the request data provided by the FlexBE synthesis interface.
# Either refer this message as 'request' in the goal part of your synthesis action
# or implement your own by at least supporting the fields listed below.

# Identifier of this synthesis query, can for example be used to set the name of the resulting state machine
string name

# Identifier of the system (i.e., configuration data set) to be used by the synthesis tool
string system

# Goal to be achieved by the synthesized behavior
string goal

# Initial conditions from which the synthesized behavior will start
string initial_condition

# Available outcomes of the generated state machine,
# i.e., possible results of the synthesized behavior
string[] sm_outcomes