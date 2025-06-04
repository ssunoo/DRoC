
# Vehicle Routing Problem with Open Start and End locations (OVRP)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    """Solves the Vehicle Routing Problem with Open Start and End locations (OVRP) using OR-Tools.

    Args:
        distance_matrix: A list of lists representing the distance matrix.
        num_vehicle: The number of vehicles.

    Returns:
        The objective value of the solution.
    """
    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(distance_matrix),
        num_vehicle,
        [0] * num_vehicle,  # Open start locations (e.g., depot 0)
        [0] * num_vehicle,  # Open end locations (e.g., depot 0)
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy =
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1
