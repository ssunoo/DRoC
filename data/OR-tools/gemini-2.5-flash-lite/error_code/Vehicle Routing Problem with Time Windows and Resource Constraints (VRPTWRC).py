
# Vehicle Routing Problem with Time Windows and Resource Constraints (VRPTWRC)
# [('user', 'The solution failed the code execution test: Error: Wrong number or type of arguments for overloaded function 'new_RoutingIndexManager'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmp_p2ih8k8.py", line 111, in <module>\n    result = solve(time_matrix, time_windows, num_vehicle, vehicle_load_time, vehicle_unload_time, depot_capacity, depot)\n  File "/tmp/tmp_p2ih8k8.py", line 5, in solve\n    manager = pywrapcp.RoutingIndexManager(\n  File "/disks/local/r.2t_6/r13922049/miniconda3/envs/AI/lib/python3.10/site-packages/ortools/constraint_solver/pywrapcp.py", line 4749, in __init__\n    _pywrapcp.RoutingIndexManager_swiginit(self, _pywrapcp.new_RoutingIndexManager(*args))\nTypeError: Wrong number or type of arguments for overloaded function 'new_RoutingIndexManager'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\n\n')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix, num_vehicle, depot, time_windows, vehicle_load_time, vehicle_unload_time, depot_capacity):
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicle, depot
    )

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time = "Time"
    # The original value of 1,000,000 for slack_max and capacity was excessively large,
    # leading to a very wide search space and potential timeouts. A more reasonable
    # upper bound for the total route time and waiting time is used here (e.g., 1000).
    # This value should be tuned based on the actual scale of the time_matrix and time_windows.
    max_time_dimension_value = 1000

    routing.AddDimension(
        transit_callback_index,
        max_time_dimension_value,  # allow waiting time (slack_max)
        max_time_dimension_value,  # maximum time per vehicle (capacity)
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot).
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            time_windows[depot][0], time_windows[depot][1]
        )

    # Add resource constraints at the depot.
    solver = routing.solver()
    intervals = []
    for i in range(num_vehicle):
        # Add fixed duration interval for vehicle loading at the start of routes.
        # The start time of this interval is the vehicle's departure time from the depot.
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.Start(i)),
                vehicle_load_time,
                "depot_load_interval_vehicle_" + str(i),
            )
        )
        # Add fixed duration interval for vehicle unloading at the end of routes.
        # The start time of this interval is the vehicle's arrival time back at the depot.
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.End(i)),
                vehicle_unload_time,
                "depot_unload_interval_vehicle_" + str(i),
            )
        )
    depot_usage = [1 for _ in range(len(intervals))]
    solver.Add(
        solver.Cumulative(intervals, depot_usage, depot_capacity, "depot_resource")
    )

    # Instantiate route start and end times to produce feasible times.
    for i in range(num_vehicle):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
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
