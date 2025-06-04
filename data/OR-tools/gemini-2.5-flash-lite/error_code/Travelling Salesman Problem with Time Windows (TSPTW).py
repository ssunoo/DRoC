
# Travelling Salesman Problem with Time Windows (TSPTW)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    num_locations = len(time_matrix)
    num_vehicles = 1 # For TSP, we use one vehicle

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot)

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
    # A sufficiently large value for slack_max (waiting time) and capacity (max time per vehicle).
    # Using a large constant for robustness, but not excessively large.
    # The original 1,000,000 might be too large, making the search space too wide.
    # A more moderate large value is used to potentially improve solver performance.
    max_dimension_value = 100000

    routing.AddDimension(
        transit_callback_index,
        max_dimension_value,  # allow waiting time (slack_max)
        max_dimension_value,  # maximum time per vehicle (capacity)
        False,           # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location.
    # Apply time windows to all locations except the depot as regular stops.
    # The depot's time window will be applied specifically to the start and end nodes.
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == depot: # Skip depot for this general loop
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for the vehicle's start and end nodes (depot).
    # For TSP, the start and end nodes are both the depot.
    start_index = routing.Start(0) # For the single vehicle (vehicle_id = 0)
    end_index = routing.End(0)     # For the single vehicle (vehicle_id = 0)

    # Ensure the depot's time window is applied to the start and end of the route.
    depot_time_window = time_windows[depot]
    time_dimension.CumulVar(start_index).SetRange(depot_time_window[0], depot_time_window[1])
    time_dimension.CumulVar(end_index).SetRange(depot_time_window[0], depot_time_window[1])

    # Instantiate route start and end times to produce feasible times.
    # This helps the solver find a solution by minimizing the start/end times,
    # which are part of the cumulative time variables.
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(start_index))
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(end_index))

    # Setting first solution heuristic and local search metaheuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    # Add a time limit to prevent excessively long run times for large problems.
    search_parameters.time_limit.FromSeconds(30) # 30 seconds time limit

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    obj = -1
    if solution:
        obj = solution.ObjectiveValue()

    return obj
