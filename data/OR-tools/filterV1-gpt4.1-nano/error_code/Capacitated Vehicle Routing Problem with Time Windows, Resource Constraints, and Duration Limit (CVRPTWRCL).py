# Capacitated Vehicle Routing Problem with Time Windows, Resource Constraints, and Duration Limit (CVRPTWRCL)
# [('user', 'The solution failed the code execution test: Error: solve() missing 1 required positional argument: 'depot_capacity'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpakvpdi9x.py", line 135, in <module>\n    result = solve(time_matrix, time_windows, demands, vehicle_capacities, num_vehicles, depot, vehicle_load_time, vehicle_unload_time, depot_capacity)\nTypeError: solve() missing 1 required positional argument: 'depot_capacity'\n\n')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicles: int, depot: int, vehicle_load_time: int, vehicle_unload_time: int, duration_limit: int, depot_capacity: int):
    # Create the data model
    data = {}
    data["time_matrix"] = time_matrix
    data["time_windows"] = time_windows
    data["demands"] = demands
    data["vehicle_capacities"] = vehicle_capacities
    data["num_vehicles"] = num_vehicles
    data["depot"] = depot
    data["vehicle_load_time"] = vehicle_load_time
    data["vehicle_unload_time"] = vehicle_unload_time
    data["depot_capacity"] = depot_capacity

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]), data["num_vehicles"], data["depot"])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        60,  # allow waiting time
        duration_limit,  # maximum route duration
        False,  # Don't force start cumul to zero
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][data["depot"]][0],
            data["time_windows"][data["depot"]][1],
        )

    # Add demand dimension for capacity constraints
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Add resource constraints at the depot
    solver = routing.solver()
    intervals = []
    for i in range(data["num_vehicles"]):
        # Load interval
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.Start(i)),
                data["vehicle_load_time"],
                f"load_{i}",
            )
        )
        # Unload interval
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.End(i)),
                data["vehicle_unload_time"],
                f"unload_{i}",
            )
        )
    depot_usage = [1 for _ in range(len(intervals))]
    solver.Add(solver.Cumulative(intervals, depot_usage, data["depot_capacity"], "depot_resource"))

    # Add route duration constraint
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        end_index = routing.End(vehicle_id)
        routing.solver().Add(
            time_dimension.CumulVar(end_index) - time_dimension.CumulVar(index) <= duration_limit
        )

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.seconds = 30  # optional time limit

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1