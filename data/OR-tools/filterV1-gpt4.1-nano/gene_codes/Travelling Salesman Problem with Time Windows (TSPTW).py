# Travelling Salesman Problem with Time Windows (TSPTW)
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, depot: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        depot: the index of the depot node

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager, which manages the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), 1, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Returns the travel time between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define the cost of each arc (edge) in the route using the transit callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint to ensure the route respects customer time windows
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time at nodes (in minutes)
        1440,  # maximum time per vehicle (in minutes)
        False,  # Don't force the start cumul to zero; allow flexible start times
        'Time')  # Dimension name
    # Retrieve the time dimension to set time window constraints
    time_dimension = routing.GetDimensionOrDie('Time')

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        start, end = time_window
        # Set the allowed time window for each location
        time_dimension.CumulVar(index).SetRange(start, end)

    # Add time window constraints for the depot (starting point)
    depot_idx = manager.NodeToIndex(depot)
    start, end = time_windows[depot]
    # Set the allowed time window for the depot
    time_dimension.CumulVar(depot_idx).SetRange(start, end)

    # Set the search parameters for the solver, including the heuristic for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value of the solution if found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1