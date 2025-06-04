
# Vehicle Routing Problem with Time Windows and Service Time (VRPTWS)
# [('user', 'The generated code cannot run or time out.')]

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, service_time: list):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        num_vehicle: the number of the vehicle
        depot: the index of the depot node
        service_time: service time for each customer node

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1

    # Create the routing index manager.
    num_locations = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicle, depot)

    # Create Routing Model.
    model = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes, including service time at the from_node."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # The time elapsed includes travel time and service time at the 'from_node'.
        return time_matrix[from_node][to_node] + service_time[from_node]

    transit_callback_index = model.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    # The cost of an arc is the time elapsed (travel time + service time at origin).
    model.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension.
    # A sufficiently large number for slack_max (waiting time) and capacity (max route duration).
    max_time_per_vehicle = 1000000

    time = "Time"
    model.AddDimension(
        transit_callback_index,
        max_time_per_vehicle,  # slack_max: maximum waiting time at a node
        max_time_per_vehicle,  # capacity: maximum total time a vehicle can accumulate
        False,                 # fix_start_cumul_to_zero: False, as depot time window might not start at 0
        time)
    time_dimension = model.GetDimensionOrDie(time)

    # Add time window constraints for each location (customer nodes).
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == depot:
            continue # Depot time window handled separately for vehicle start/end.
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for vehicles at the depot (start and end of routes).
    for vehicle_id in range(num_vehicle):
        # Start node (depot)
        start_index = model.Start(vehicle_id)
        time_dimension.CumulVar(start_index).SetRange(time_windows[depot][0], time_windows[depot][1])
        # End node (depot)
        end_index = model.End(vehicle_id)
        time_dimension.CumulVar(end_index).SetRange(time_windows[depot][0], time_windows[depot][1])

    # Instantiate search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(30) # Set a time limit for solving

    # Solve the problem.
    solution = model.SolveWithParameters(search_parameters)

    # Get the objective value.
    if solution:
        obj = solution.ObjectiveValue()
    else:
        obj = -1 # Indicate no solution found

    return obj

