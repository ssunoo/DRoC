
# Vehicle Routing with Pickups and Deliveries, Time Windows, and Multiple Depots (PDPTWMD)
# [('user', 'The solution failed the code execution test: Error: 'bool' object has no attribute 'CumulVar'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpxal0ufsy.py", line 99, in <module>\n    result = solve(time_matrix, time_windows, pickups_deliveries, num_vehicle, starts, ends)\n  File "/tmp/tmpxal0ufsy.py", line 58, in solve\n    time_dimension.CumulVar(index).SetRange(start_time, end_time)\nAttributeError: 'bool' object has no attribute 'CumulVar'\n\n')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, starts: list, ends: list):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        pickups_deliveries: a list of pairs of pickup and delivery locations
        num_vehicle: the number of the vehicle
        starts: the index of the starting depot for vehicles
        ends: the index of the ending depot for vehicles

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1

    num_locations = len(time_matrix)

    # Create the routing index manager.
    # The manager maps actual node indices to internal solver indices and handles multiple depots.
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicle, starts, ends)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for time.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    # The objective is to minimize the total travel time.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    # A sufficiently large number for maximum time and slack to allow for waiting.
    # This value should be greater than any possible cumulative time or waiting time.
    large_time_value = 1000000000  # 1 billion, assuming time units are reasonable

    time_dimension = routing.AddDimension(
        transit_callback_index,
        large_time_value,  # allow waiting time (slack_max)
        large_time_value,  # maximum time per vehicle (capacity)
        True,  # fix start cumul to zero (vehicles start at time 0 from depots)
        "Time")

    # Add time window constraints for each location.
    # Iterate through all possible locations (nodes).
    for location_idx in range(num_locations):
        index = manager.NodeToIndex(location_idx)
        start_time, end_time = time_windows[location_idx]
        time_dimension.CumulVar(index).SetRange(start_time, end_time)

    # Add pickup and delivery requests.
    # For each pair, ensure the delivery happens after the pickup and by the same vehicle.
    for pickup_node, delivery_node in pickups_deliveries:
        routing.AddPickupAndDelivery(manager.NodeToIndex(pickup_node),
                                     manager.NodeToIndex(delivery_node))

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(30) # Set a time limit for solving

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value.
    if solution:
        obj = solution.ObjectiveValue()
    else:
        obj = -1  # Indicate no solution found

    return obj
