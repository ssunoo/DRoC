# Vehicle Routing Problem with Service Time and Duration Limit (VRPSL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        num_vehicle: the number of the vehicle
        depot: the index of the depot node
        service_time: service time for each customer node
        duration_limit: the time duration of each route is upper bounded by the duration limit 

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node] + service_time[from_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        duration_limit,  # maximum route duration
        True,  # force start cumul to zero
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Add service time to each node's cumulative time.
    for node in range(len(time_matrix)):
        index = manager.NodeToIndex(node)
        time_dimension.CumulVar(index).SetValue(service_time[node])

    # Enforce maximum route duration by setting the start cumul variables to zero.
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        # Set the start cumul variable to zero.
        time_dimension.CumulVar(index).SetValue(0)
        # Add a constraint for route duration.
        routing.AddDimension(
            transit_callback_index,
            0,  # slack
            duration_limit,  # vehicle maximum travel time
            True,  # start cumul to zero
            f"duration_{vehicle_id}")

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Set a time limit for the search.
    search_parameters.time_limit.seconds = 30

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1