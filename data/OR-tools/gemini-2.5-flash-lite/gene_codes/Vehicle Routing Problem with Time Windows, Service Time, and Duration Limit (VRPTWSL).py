
# Vehicle Routing Problem with Time Windows, Service Time, and Duration Limit (VRPTWSL)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        num_vehicle: the number of the vehicle
        depot: the index of the depot node
        service_time: service time for each customer node
        duration_limit: the time duration of each route is upper bounded by the duration limit

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicle, depot
    )

    # Create Routing Model.
    model = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for travel time (for arc cost).
    def travel_time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    travel_time_callback_index = model.RegisterTransitCallback(travel_time_callback)

    # Define cost of each arc.
    model.SetArcCostEvaluatorOfAllVehicles(travel_time_callback_index)

    # Add Time dimension.
    # The transit callback for the time dimension should include service time.
    def time_callback(from_index, to_index):
        """Returns the total time (travel + service) between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # The service time is incurred at the 'from_node' before traveling to 'to_node'.
        return time_matrix[from_node][to_node] + service_time[from_node]

    time_callback_index = model.RegisterTransitCallback(time_callback)

    # Define a large constant for max time to allow flexibility for slack and capacity.
    # The actual route duration limit is enforced by SetSpanUpperBoundForVehicle.
    large_time_constant = 1000000

    model.AddDimension(
        time_callback_index,
        large_time_constant,  # slack_max: maximum waiting time at a node.
        large_time_constant,  # capacity: maximum value of the cumul variable for any node.
        True,                 # fix_start_cumul_to_zero: vehicles start at time 0 from depot.
        "Time",
    )
    time_dimension = model.GetDimensionOrDie("Time")

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add duration limit for each vehicle.
    for vehicle_id in range(num_vehicle):
        # The span of the time dimension for each vehicle must be within duration_limit.
        time_dimension.SetSpanUpperBoundForVehicle(duration_limit, vehicle_id)

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(30) # Set a reasonable time limit for solving.

    # Solve the problem.
    assignment = model.SolveWithParameters(search_parameters)

    # Get the objective value.
    if assignment:
        obj = assignment.ObjectiveValue()
    else:
        obj = -1 # Indicate no solution found

    return obj
