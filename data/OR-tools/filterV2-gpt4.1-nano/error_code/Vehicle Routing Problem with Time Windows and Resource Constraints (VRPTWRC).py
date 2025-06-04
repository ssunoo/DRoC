# Vehicle Routing Problem with Time Windows and Resource Constraints (VRPTWRC)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, num_vehicle: int, vehicle_load_time: int, vehicle_unload_time: int, depot_capacity: int, depot: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        num_vehicle: the number of the vehicle
        vehicle_load_time: the time required to load a vehicle at depot
        vehicle_unload_time: the time required to unload a vehicle at depot
        depot_capacity: the maximum number of vehicles that can load or unload simultaneously at the depot
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
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        10000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        'Time')
    time_dimension = routing.GetDimensionOrDie('Time')

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add resource constraints at depot (loading/unloading)
    # For simplicity, assume resource constraints are modeled as capacity constraints on the number of vehicles at depot
    # This can be modeled with a custom resource dimension if needed
    # Here, we add a dummy resource dimension to limit the number of vehicles starting at depot
    def demand_callback(from_index):
        # Demand is 1 for all locations except depot
        node = manager.IndexToNode(from_index)
        if node == depot:
            return 0
        return 1

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        [depot_capacity] * num_vehicle,  # vehicle capacities
        True,  # start cumul to zero
        'Resource')

    resource_dimension = routing.GetDimensionOrDie('Resource')

    # Add load and unload times at depot
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        # Add load time
        time_dimension.CumulVar(index).SetValue(vehicle_load_time)
        # Add unload time at end
        end_index = routing.End(vehicle_id)
        time_dimension.CumulVar(end_index).SetValue(vehicle_unload_time)

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Calculate total travel time
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
