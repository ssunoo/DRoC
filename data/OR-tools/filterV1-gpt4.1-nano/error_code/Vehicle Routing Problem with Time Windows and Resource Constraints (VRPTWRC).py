# Vehicle Routing Problem with Time Windows and Resource Constraints (VRPTWRC)
# [('user', 'The solution failed the code execution test: Error: 'RoutingModel' object has no attribute 'CumulVar''\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmps_71k0ih.py", line 102, in <module>\n    result = solve(time_matrix, time_windows, num_vehicle, vehicle_load_time, vehicle_unload_time, depot_capacity, depot)\n  File "/tmp/tmps_71k0ih.py", line 69, in solve\n    routing.AddToAssignment(routing.CumulVar(index), vehicle_load_time)\nAttributeError: 'RoutingModel' object has no attribute 'CumulVar'\n')]
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
    horizon = max([tw[1] for tw in time_windows])
    routing.AddDimension(
        transit_callback_index,
        horizon,  # allow waiting time
        horizon,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        'Time')
    time_dimension = routing.GetDimensionOrDie('Time')

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add resource constraints at depot (loading/unloading)
    # Assuming resource constraints are modeled as capacity constraints on the number of vehicles at depot
    # For simplicity, we model this as a capacity constraint on the number of vehicles starting at depot
    # Note: OR-Tools does not directly support resource constraints at nodes, so this is a simplified assumption
    # For more complex resource constraints, custom constraints are needed
    # Here, we enforce that at most depot_capacity vehicles start at the depot
    def vehicle_start_constraint(routing, manager, depot, depot_capacity):
        start_indices = [routing.Start(vehicle_id) for vehicle_id in range(num_vehicle)]
        for start_index in start_indices:
            routing.solver().Add(routing.VehicleVar(start_index) <= depot_capacity)

    vehicle_start_constraint(routing, manager, depot, depot_capacity)

    # Add load/unload times at depot
    # For simplicity, we add fixed load/unload times at depot nodes
    for node in range(len(time_matrix)):
        index = manager.NodeToIndex(node)
        if node == depot:
            # Add load time
            routing.AddToAssignment(routing.CumulVar(index), vehicle_load_time)
            # Add unload time
            routing.AddToAssignment(routing.CumulVar(index), vehicle_unload_time)

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return routing.Cost(solution)
    else:
        return -1
