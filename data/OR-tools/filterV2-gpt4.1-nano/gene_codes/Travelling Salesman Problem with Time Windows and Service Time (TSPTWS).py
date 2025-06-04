# Travelling Salesman Problem with Time Windows and Service Time (TSPTWS)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

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
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), 1, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Include service time at the from_node.
        return time_matrix[from_node][to_node] + (service_time[from_node] if from_node != depot else 0)

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of travel for each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        10000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == depot:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set time window constraints for depot.
    depot_time_window = time_windows[depot]
    for vehicle_id in range(1):  # assuming one vehicle
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(depot_time_window[0], depot_time_window[1])

    # Add service time as constraints within the time dimension.
    # Already incorporated in the transit callback.

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1