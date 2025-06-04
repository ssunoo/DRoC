# Prize Collecting Travelling Salesman Problem with Time Windows (PCTSPTW)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, depot: int, prizes: list, max_duration: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        prizes: the value of prize that a vehicle can collect at each node
        max_duration: maximum duration that a vehicle can travel
        depot: the index of the depot node

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), 1, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        max_duration,  # maximum travel time
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each node.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for depot start nodes.
    for vehicle_id in range(1):  # assuming single vehicle for prize collecting
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            time_windows[depot][0], time_windows[depot][1]
        )

    # Add disjunctions for prize collection with penalties.
    for node in range(len(time_matrix)):
        if node != depot:
            routing.AddDisjunction([manager.NodeToIndex(node)], prizes[node])

    # Add maximum duration constraint.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_duration,  # maximum total travel time
        True,  # start cumul to zero
        "Duration",
    )
    duration_dimension = routing.GetDimensionOrDie("Duration")

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Compute the total prize collected.
    total_prize = 0
    if assignment:
        for node in range(len(time_matrix)):
            index = manager.NodeToIndex(node)
            if assignment.Value(time_dimension.CumulVar(index)) is not None:
                total_prize += prizes[node]
        # Subtract penalties for dropped nodes.
        total_prize -= sum(
            prizes[node] for node in range(len(time_matrix))
            if assignment.Value(time_dimension.CumulVar(manager.NodeToIndex(node))) is None and node != depot
        )
        return total_prize
    return -1
