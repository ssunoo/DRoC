
# Capacitated Vehicle Routing Problem with Time Windows and Resource Constraints (CVRPTWRC)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def solve(
    time_matrix,
    time_windows,
    demands,
    vehicle_capacities,
    num_vehicles,
    depot,
    vehicle_load_time,
    vehicle_unload_time,
    depot_capacity,
):
    """Solves the Capacitated Vehicle Routing Problem with Time Windows and Resource Constraints (CVRPTWRC) using OR-Tools.

    Args:
        time_matrix: A list of lists representing the travel times between locations.
        time_windows: A list of tuples representing the time windows for each customer.
        demands: A list of integers representing the demand of each customer.
        vehicle_capacities: A list of integers representing the capacity of each vehicle.
        num_vehicles: The number of vehicles available.
        depot: The index of the depot.
        vehicle_load_time: The time required to load a vehicle at the depot.
        vehicle_unload_time: The time required to unload a vehicle at the depot.
        depot_capacity: The maximum number of vehicles that can load or unload at the same time.

    Returns:
        The objective value of the solution.
    """
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicles, depot
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Add Time Windows constraint.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        30,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == depot:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    depot_idx = depot
    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            time_windows[depot_idx][0], time_windows[depot_idx][1]
        )

    # Instantiate route start and end times to produce feasible times.
    for i in range(num_vehicles):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i))
        )
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Add resource constraint for depot loading/unloading.
    intervals = []
    for vehicle_id in range(num_vehicles):
        start_index = routing.Start(vehicle_id)
        end_index = routing.End(vehicle_id)
        intervals.append((time_dimension.CumulVar(start_index), vehicle_load_time, depot_capacity))  # Corrected: Added capacity
        intervals.append((time_dimension.CumulVar(end_index), vehicle_unload_time, depot_capacity))  # Corrected: Added capacity

    solver = routing.solver()
    solver.Add(solver.Cumulative(intervals, [1] * len(intervals), depot_capacity, "depot"))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        return solution.ObjectiveValue()
    return -1
