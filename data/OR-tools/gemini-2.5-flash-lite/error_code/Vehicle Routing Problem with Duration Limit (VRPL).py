
# Vehicle Routing Problem with Duration Limit (VRPL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    # The slack_max is 0, meaning no waiting time at nodes.
    # The capacity is duration_limit for each vehicle.
    # fix_start_cumul_to_zero=True ensures the time starts at 0 for each vehicle at the depot.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        duration_limit,  # vehicle maximum duration
        True,  # start cumul to zero
        "Time")
    time_dimension = routing.GetDimensionOrDie("Time")

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(30) # Optional: set a time limit for the search

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    obj = -1
    if solution:
        obj = solution.ObjectiveValue()

    return obj
