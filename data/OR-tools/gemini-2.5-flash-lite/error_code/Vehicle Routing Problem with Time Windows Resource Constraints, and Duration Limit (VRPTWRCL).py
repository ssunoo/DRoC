
# Vehicle Routing Problem with Time Windows Resource Constraints, and Duration Limit (VRPTWRCL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Create the routing index manager.
manager = pywrapcp.RoutingIndexManager(
    len(time_matrix), num_vehicle, depot
)

# Create Routing Model.
routing = pywrapcp.RoutingModel(manager)

# Create and register a transit callback.
def time_callback(from_index, to_index):
    """Returns the travel time between the two nodes."""
    # Convert from routing variable Index to time matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return time_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(time_callback)

# Define cost of each arc.
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Add Time Windows constraint and Duration Limit.
# The 'capacity' parameter in AddDimension sets the maximum cumul value for each vehicle,
# which corresponds to the duration limit for each route.
# 'slack_max' allows waiting time at nodes. Set to duration_limit to allow waiting within the route limit.
time = "Time"
routing.AddDimension(
    transit_callback_index,
    duration_limit,  # allow waiting time (slack_max)
    duration_limit,  # maximum time per vehicle (capacity)
    False,  # Don't force start cumul to zero.
    time,
)
time_dimension = routing.GetDimensionOrDie(time)

# Add time window constraints for each location.
for location_idx, time_window in enumerate(time_windows):
    index = manager.NodeToIndex(location_idx)
    time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

# Add time window constraints for each vehicle start node (at the depot).
# This is crucial to ensure vehicles start within the depot's operating hours.
# The depot's time window is time_windows[depot].
for vehicle_id in range(num_vehicle):
    start_index = routing.Start(vehicle_id)
    time_dimension.CumulVar(start_index).SetRange(
        time_windows[depot][0], time_windows[depot][1]
    )

# Instantiate route start and end times to produce feasible times.
# This is important for the solver to find a solution.
for i in range(num_vehicle):
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.Start(i))
    )
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.End(i))
    )

# Add Resource Constraints (Depot Capacity).
# This uses the Cumulative constraint to limit simultaneous loading/unloading at the depot.
solver = routing.solver()
all_depot_intervals = []
depot_usage_list = []

for vehicle_id in range(num_vehicle):
    # Loading interval at depot (start of route)
    loading_start_time_var = time_dimension.CumulVar(routing.Start(vehicle_id))
    loading_interval = solver.MakeFixedDurationIntervalVar(
        loading_start_time_var, vehicle_load_time, f"loading_v{vehicle_id}"
    )
    all_depot_intervals.append(loading_interval)
    depot_usage_list.append(1) # Each vehicle uses 1 unit of capacity

    # Unloading interval at depot (end of route)
    unloading_start_time_var = time_dimension.CumulVar(routing.End(vehicle_id))
    unloading_interval = solver.MakeFixedDurationIntervalVar(
        unloading_start_time_var, vehicle_unload_time, f"unloading_v{vehicle_id}"
    )
    all_depot_intervals.append(unloading_interval)
    depot_usage_list.append(1) # Each vehicle uses 1 unit of capacity

# Apply the cumulative constraint for depot capacity
solver.Add(
    solver.Cumulative(all_depot_intervals, depot_usage_list, depot_capacity, "depot_resource")
)

# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)
# Add a search time limit to prevent indefinite running
search_parameters.time_limit.seconds = 30 # Set a reasonable time limit, e.g., 30 seconds

# Solve the problem.
solution = routing.SolveWithParameters(search_parameters)

# Return the objective value.
obj = -1 # Default value if no solution is found
if solution:
    obj = solution.ObjectiveValue()

return obj
