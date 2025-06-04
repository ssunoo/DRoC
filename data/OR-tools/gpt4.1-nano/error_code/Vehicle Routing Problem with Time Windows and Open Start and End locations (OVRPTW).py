# Vehicle Routing Problem with Time Windows and Open Start and End locations (OVRPTW)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        num_vehicle: the number of the vehicle

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create data model
    data = {}
    data["time_matrix"] = time_matrix
    data["time_windows"] = time_windows
    data["num_vehicles"] = num_vehicle
    # For open start/end, set start and end nodes for each vehicle
    # Here, assuming all vehicles start at node 0 and end at node 0, but can be customized
    # For open start/end, we can set start and end nodes to None or specific nodes
    # For simplicity, assume start and end nodes are the same for all vehicles and are node 0
    data["starts"] = [0] * num_vehicle
    data["ends"] = [0] * num_vehicle
    data["depot"] = 0

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]), data["num_vehicles"], data["starts"], data["ends"])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        30,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data["time_windows"]):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for v in range(data["num_vehicles"]):
        start_index = routing.Start(v)
        # Set the start node's time window to match the depot's time window
        depot_time_window = data["time_windows"][data["starts"][v]]
        time_dimension.CumulVar(start_index).SetRange(depot_time_window[0], depot_time_window[1])

    # Minimize the total time
    for v in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(v)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(v)))

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1