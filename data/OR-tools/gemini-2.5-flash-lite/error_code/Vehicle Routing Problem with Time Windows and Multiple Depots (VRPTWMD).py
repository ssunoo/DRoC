
# Vehicle Routing Problem with Time Windows and Multiple Depots (VRPTWMD)
# [('user', 'The solution failed the code execution test: Error: Wrong number or type of arguments for overloaded function \'new_RoutingIndexManager\'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmp3l4xjpx7.py", line 111, in <module>\n    result = solve(time_matrix, time_windows, starts, ends, num_vehicles)\n  File "/tmp/tmp3l4xjpx7.py", line 11, in solve\n    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, starts, ends)\n  File "/disks/local/r.2t_6/r13922049/miniconda3/envs/AI/lib/python3.10/site-packages/ortools/constraint_solver/pywrapcp.py", line 4749, in __init__\n    _pywrapcp.RoutingIndexManager_swiginit(self, _pywrapcp.new_RoutingIndexManager(*args))\nTypeError: Wrong number or type of arguments for overloaded function \'new_RoutingIndexManager\'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\n\n')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix, num_vehicles, starts, ends, time_windows):
    # time_matrix: 2D array representing travel times between locations.
    # num_vehicles: The number of vehicles available.
    # starts: A list of depot node indices where each vehicle starts.
    # ends: A list of depot node indices where each vehicle ends.
    # time_windows: A list of time windows for each location.
    #               Each element is a tuple (start_time, end_time).
    num_locations = len(time_matrix)

    # Create the routing index manager.
    # For multiple depots, starts and ends are lists of node indices.
    # The error message suggests an issue with the arguments passed to RoutingIndexManager.
    # It expects either (num_locations, num_vehicles, depot_index) or (num_locations, num_vehicles, starts, ends).
    # The current code uses the latter, which is correct for multiple depots.
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, starts, ends)

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

    # Add Time Windows constraint.
    time = "Time"

    # A sufficiently large constant for max_time_per_vehicle (capacity).
    # This should be large enough to cover any possible route time.
    max_time_per_vehicle = 100000

    # Calculate a reasonable slack_max based on time windows.
    # This should be the maximum possible waiting time at any node.
    max_span = 0
    if time_windows:
        for tw in time_windows:
            max_span = max(max_span, tw[1] - tw[0])

    # If time_windows is empty or all time windows have zero span,
    # set slack_max_value to a sufficiently large value (e.g., max_time_per_vehicle)
    # to allow waiting, preventing immediate infeasibility.
    if not time_windows or max_span == 0:
        slack_max_value = max_time_per_vehicle  # Allow maximum possible waiting
    else:
        slack_max_value = max_span  # Use the maximum span as a guide for allowed waiting

    routing.AddDimension(
        transit_callback_index,
        slack_max_value,       # allow waiting time (slack_max)
        max_time_per_vehicle,  # maximum time per vehicle (capacity)
        False,                 # Don't force start cumul to zero. (Vehicles can start at different times/depots)
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for all locations (customers and depots).
    # This assumes time_windows is a list of (start, end) tuples, where time_windows[i]
    # corresponds to the time window for NodeIndex i.
    for location_idx in range(num_locations):
        index = manager.NodeToIndex(location_idx)
        if location_idx < len(time_windows):
            time_window = time_windows[location_idx]
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        # If location_idx is out of bounds for time_windows, no explicit time window is set for it.
        # It will be constrained by the dimension's overall capacity (max_time_per_vehicle).

    # Minimize the total time of all routes.
    # This is done by adding the cumul variables of the end nodes to the solution finalizer.
    for i in range(num_vehicles):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Optional: Add a local search metaheuristic for better solutions
    search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # Optional: Add a time limit for solving
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    obj = None
    if solution:
        obj = solution.ObjectiveValue()

    return obj
