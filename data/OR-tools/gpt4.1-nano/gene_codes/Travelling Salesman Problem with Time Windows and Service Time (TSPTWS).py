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
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), 1, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node] + service_time[from_node]  # Calculate travel time plus service time at the from_node

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator to use the transit callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension to model to handle time windows and service times
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time at nodes
        10000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        time)  # Dimension name
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for each location
    for location_idx, (start, end) in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(start, end)  # Set time window for each node

    # Set time window constraints for depot (start location)
    for vehicle_id in range(1):  # Loop over vehicles, here only one vehicle
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(time_windows[depot][0], time_windows[depot][1])  # Set depot time window

    # Add service time to each node's transit time (already included in time_callback)
    for node in range(len(time_matrix)):
        index = manager.NodeToIndex(node)
        # Adjust the transit callback to include service time
        # Already included in time_callback
        pass  # No additional code needed as service time is already added in time_callback

    # Setting first solution heuristic to guide the search
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1