
# Capacitated Vehicle Routing Problem with Time Windows and Resource Constraints (CVRPTWRC)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Create the routing index manager.
manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicles, depot)

# Create Routing Model.
routing = pywrapcp.RoutingModel(manager)

# Create and register a transit callback for time.
def time_callback(from_index, to_index):
    """Returns the travel time between the two nodes."""
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return time_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(time_callback)

# Define cost of each arc.
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Add Capacity constraint.
def demand_callback(from_index):
    """Returns the demand of the node."""
    from_node = manager.IndexToNode(from_index)
    return demands[from_node]

demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,  # null capacity slack
    vehicle_capacities,  # vehicle capacities
    True,  # start cumul to zero
    "Capacity",
)

# Add Time Windows constraint.
time = "Time"
# Determine a sufficiently large maximum time for vehicles.
# This should be at least the maximum possible route duration including service times.
# A large constant is used for robustness.
# The original value of 1,000,000 was excessively large, leading to a vast search space
# and potential timeouts. A more realistic upper bound is used here.
max_time_per_vehicle = 2000  # A more reasonable upper bound for route duration and waiting time

routing.AddDimension(
    transit_callback_index,
    max_time_per_vehicle,  # allow waiting time (slack_max)
    max_time_per_vehicle,  # maximum time per vehicle (capacity)
    False,  # Don't force start cumul to zero.
    time,
)
time_dimension = routing.GetDimensionOrDie(time)

# Add time window constraints for each location.
for location_idx, time_window in enumerate(time_windows):
    index = manager.NodeToIndex(location_idx)
    time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

# Ensure vehicles have enough time to load before starting their route.
# The start time of a vehicle's route (time_dimension.CumulVar(routing.Start(i)))
# is also the end time of its loading operation. To ensure loading_start_var
# (which is loading_end_var - vehicle_load_time) is non-negative,
# loading_end_var must be at least vehicle_load_time.
for i in range(num_vehicles):
    start_node_index = routing.Start(i)
    time_dimension.CumulVar(start_node_index).SetMin(vehicle_load_time)

# Instantiate route start and end times to produce feasible times.
for i in range(num_vehicles):
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

# Add Resource Constraints (Depot Capacity).
solver = routing.solver()
intervals = []
depot_usage = []

for vehicle_id in range(num_vehicles):
    is_used_var = routing.IsVehicleUsed(vehicle_id)

    # Loading operation at the depot (before departure for the first customer).
    # The vehicle departs at time_dimension.CumulVar(routing.Start(vehicle_id)).
    loading_end_var = time_dimension.CumulVar(routing.Start(vehicle_id))
    # loading_start_var must be non-negative. Its upper bound should allow for vehicle_load_time.
    loading_start_var = solver.MakeIntVar(0, max_time_per_vehicle - vehicle_load_time, f"loading_start_v{vehicle_id}")

    # Create the loading interval. It's optional, and its performance is linked to vehicle usage.
    loading_interval = solver.MakeIntervalVar(
        loading_start_var,
        loading_end_var,
        vehicle_load_time,  # Fixed duration
        True,  # Is optional
        f"loading_interval_v{vehicle_id}"
    )
    intervals.append(loading_interval)
    depot_usage.append(1)

    # Link the interval's performance to vehicle usage.
    # If vehicle is used, loading interval must be performed.
    solver.Add(solver.IsEqual(is_used_var, 1).Implies(loading_interval.IsPerformed()))
    # If vehicle is not used, loading interval must be absent.
    solver.Add(solver.IsEqual(is_used_var, 0).Implies(loading_interval.IsAbsent()))

    # Unloading operation at the depot (after arrival from the last customer).
    # The vehicle arrives at time_dimension.CumulVar(routing.End(vehicle_id)).
    unloading_start_var = time_dimension.CumulVar(routing.End(vehicle_id))
    # unloading_end_var must be at least vehicle_unload_time and within bounds.
    unloading_end_var = solver.MakeIntVar(vehicle_unload_time, max_time_per_vehicle + vehicle_unload_time, f"unloading_end_v{vehicle_id}")

    # Create the unloading interval. It's optional, and its performance is linked to vehicle usage.
    unloading_interval = solver.MakeIntervalVar(
        unloading_start_var,
        unloading_end_var,
        vehicle_unload_time,  # Fixed duration
        True,  # Is optional
        f"unloading_interval_v{vehicle_id}"
    )
    intervals.append(unloading_interval)
    depot_usage.append(1)

    # Link the interval's performance to vehicle usage.
    # If vehicle is used, unloading interval must be performed.
    solver.Add(solver.IsEqual(is_used_var, 1).Implies(unloading_interval.IsPerformed()))
    # If vehicle is not used, unloading interval must be absent.
    solver.Add(solver.IsEqual(is_used_var, 0).Implies(unloading_interval.IsAbsent()))

# Add the cumulative constraint for depot capacity.
solver.Add(solver.Cumulative(intervals, depot_usage, depot_capacity, "depot_resource"))

# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)

# Solve the problem.
solution = routing.SolveWithParameters(search_parameters)

obj = -1
if solution:
    obj = solution.ObjectiveValue()
return obj
