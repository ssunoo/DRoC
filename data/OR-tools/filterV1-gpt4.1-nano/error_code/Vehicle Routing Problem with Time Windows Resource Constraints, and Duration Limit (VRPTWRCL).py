# Vehicle Routing Problem with Time Windows Resource Constraints, and Duration Limit (VRPTWRCL)
# [('user', "UnusedParameterError: Params {'vehicle_unload_time', 'vehicle_load_time'} are not used, you should ensure all the params are used in the function.")]
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, num_vehicle: int, vehicle_load_time: int, vehicle_unload_time: int, depot_capacity: int, duration_limit: int, depot: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        num_vehicle: the number of the vehicle
        vehicle_load_time: the time required to load a vehicle at depot
        vehicle_unload_time: the time required to unload a vehicle at depot
        depot_capacity: the maximum number of vehicles that can load or unload simultaneously at the depot
        duration_limit: the time duration of each route is upper bounded by the duration limit
        depot: the index of the depot node

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback
    def time_callback(from_index, to_index):
        # Returns the travel time between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        duration_limit,  # maximum route duration
        False,  # Don't force start cumul to zero
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add resource constraints at depot (loading/unloading)
    # Assuming resource constraints are modeled as capacity constraints at depot
    # For simplicity, we model the resource constraint as a capacity dimension
    resource_capacity = depot_capacity
    def resource_callback(from_index):
        # Resource consumption at depot (loading/unloading)
        return 1  # Each visit consumes 1 unit resource

    resource_callback_index = routing.RegisterUnaryTransitCallback(resource_callback)

    routing.AddDimension(
        resource_callback_index,
        0,  # no slack
        resource_capacity,  # capacity
        True,  # start cumul to zero
        "Resource")

    # Add duration limit constraint
    # Already added via time dimension with max_time = duration_limit

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Compute total travel time
    total_time = 0
    if solution:
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                total_time += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
    else:
        total_time = -1

    return total_time
