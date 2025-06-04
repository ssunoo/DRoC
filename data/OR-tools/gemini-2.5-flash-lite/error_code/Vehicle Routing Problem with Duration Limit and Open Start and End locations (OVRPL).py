
# Vehicle Routing Problem with Duration Limit and Open Start and End locations (OVRPL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicle, starts, ends
    )

    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        """Returns the time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    dimension_name = "Time"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        duration_limit,  # vehicle maximum travel time
        True,  # start cumul to zero
        dimension_name,
    )
    time_dimension = routing.GetDimensionOrDie(dimension_name)

    # Add a cost to the global span of the time dimension. This encourages balanced routes
    # and can help the solver find solutions faster, especially if the problem is large or complex.
    time_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic and search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    # Set a time limit for the search to prevent timeout.
    search_parameters.time_limit.seconds = 30  # You can adjust this value

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        return solution.ObjectiveValue()
    else:
        return None
