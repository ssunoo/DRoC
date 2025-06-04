
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix, depot, max_distance, prizes):
    """Solves the VRP with node disjunctions and returns the objective value.

    Args:
        distance_matrix: A 2D list representing the distance matrix.
        depot: The index of the depot node.
        max_distance: The maximum allowed distance for a route.
        prizes: A list of prizes for each node.

    Returns:
        The objective value of the solution, or -1 if no solution is found.
    """
    num_nodes = len(distance_matrix)
    num_vehicles = 1

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        num_nodes, num_vehicles, depot
    )

    # Create routing model.
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

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_distance,
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Allow to drop nodes.
    for node in range(num_nodes):
        if node != depot:
            routing.AddDisjunction(
                [manager.NodeToIndex(node)], prizes[node]
            )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy =
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    obj = -1
    if assignment:
        obj = routing.ObjectiveValue()

    return obj
