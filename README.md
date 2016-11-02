# SOLA

Surface-based Object Learning Architecture

This system is a pipeline concerned with robot object perception, and ties together large amounts of the STRANDS system in order to produce SOMA objects. It is dependent on SOMA, strands_3d_mapping, v4r, the initial_surface_view_evaluation package, as well as the python library shapely. The general workflow of the system is that it receives views from a 3D sensor, typically via some view planning algorithm (such as VIPER), segments these views into objects, aligns these views in the case of multiple views (and tracks objects between views), merges multiple views of objects into single point clouds, sees if any object recognition can be run on the learned objects, and handles the creation and updating of both high level and low-level SOMA objects, all the while ensuring any learned objects are within defined SOMA ROIs. This all runs without any input from anywhere else, and the pipeline is trigged via a single service call.

There is a single launch file `surface_based_object_learning.launch` and a test script `do_manual_views.py` so that the system can be run manually, without the need of a view planning algorithm.

More detailed documentation and instructions coming soon.
