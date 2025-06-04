# Travelling Salesman Problem with Time Windows and Service Time (TSPTWS)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, depot: int, service_time: list):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        depot: the index of the depot node
        service_time: service time for each customer node

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager, which manages the conversion between the problem's node indices and the solver's internal indices
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), 1, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Convert from routing variable Index to time matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc as the travel time between nodes
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint
    def service_time_callback(from_index):
        # Convert from routing variable Index to node index
        from_node = manager.IndexToNode(from_index)
        return service_time[from_node]

    # Register callback for service time at each node
    service_time_callback_index = routing.RegisterUnaryTransitCallback(service_time_callback)

    # Add dimension for time to model the cumulative travel time
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        1440,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        'Time')

    # Retrieve the time dimension to set constraints
    time_dimension = routing.GetDimensionOrDie('Time')

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for the depot
    depot_idx = manager.NodeToIndex(depot)
    time_dimension.CumulVar(depot_idx).SetRange(time_windows[depot][0], time_windows[depot][1])

    # Setting first solution heuristic to guide the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1