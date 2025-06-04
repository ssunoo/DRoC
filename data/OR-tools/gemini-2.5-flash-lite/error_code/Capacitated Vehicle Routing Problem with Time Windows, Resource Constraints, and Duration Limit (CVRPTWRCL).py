
# Capacitated Vehicle Routing Problem with Time Windows, Resource Constraints, and Duration Limit (CVRPTWRCL)
# [('user', 'The solution failed the code execution test: Error: solve() missing 1 required positional argument: \'depot_capacity\'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpc76c2828.py", line 120, in <module>\n    result = solve(time_matrix, time_windows, demands, vehicle_capacities, num_vehicles, depot, vehicle_load_time, vehicle_unload_time, depot_capacity)\nTypeError: solve() missing 1 required positional argument: \'depot_capacity\'\n\n')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix, num_vehicles, depot, demands, vehicle_capacities, time_windows, duration_limit, vehicle_load_time, vehicle_unload_time, depot_capacity):
    # Initialize the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicles, depot)
    # Initialize the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Add Capacity constraint
    def demand_callback(from_index):
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacities(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # vehicle capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Add Time Windows and Duration Limit constraint
    def time_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    time = "Time"
    MAX_SLACK = 100000  # A sufficiently large number for maximum waiting time
    routing.AddDimension(
        transit_callback_index,
        MAX_SLACK,  # allow waiting time (max slack)
        duration_limit,  # maximum time per vehicle (duration limit)
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        # Skip the depot in this loop, as its time window will be applied to vehicle start nodes.
        if location_idx == depot:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (at the depot)
    # This ensures that vehicles start from the depot within its specified time window.
    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(time_windows[depot][0], time_windows[depot][1])

    # Add Resource Constraints at the depot
    solver = routing.solver()
    intervals = []
    for i in range(num_vehicles):
        # Add time intervals for vehicle loading at the start of routes
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.Start(i)),
                vehicle_load_time,
                f"depot_load_interval_vehicle_{i}",
            )
        )
        # Add time intervals for vehicle unloading at the end of routes.
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.End(i)),
                vehicle_unload_time,
                f"depot_unload_interval_vehicle_{i}",
            )
        )
    depot_usage = [1 for _ in range(len(intervals))]
    solver.Add(
        solver.Cumulative(intervals, depot_usage, depot_capacity, "depot_resource")
    )

    # Instantiate route start and end times to produce feasible times and minimize total time.
    for i in range(num_vehicles):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i))
        )
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    obj = -1
    if solution:
        obj = solution.ObjectiveValue()
    return obj
