
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def solve_pctsptw(time_matrix, time_windows, prizes, max_duration, depot):
    """Solves the Prize Collecting Traveling Salesman Problem with Time Windows (PCTSPTW).

    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        prizes: the value of prize that a vehicle can collect at each node
        max_duration: maximum duration that a vehicle can travel
        depot: the index of the depot node

    Returns:
        obj: a number representing the objective value of the solution
    """
    num_locations = len(time_matrix)
    num_vehicles = 1

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        num_locations, num_vehicles, depot
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time_dimension_name = "Time"
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        max_duration,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time_dimension_name,
    )
    time_dimension = routing.GetDimensionOrDie(time_dimension_name)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Allow to drop nodes and collect prizes.
    for node in range(num_locations):
        if node == depot:
            continue
        penalty = prizes[node]
        routing.AddDisjunction(
            [manager.NodeToIndex(node)], penalty
        )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy =
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Extract the objective value.
    if solution:
        obj = solution.ObjectiveValue()
    else:
        obj = -1

    return obj
