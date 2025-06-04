# Vehicle Routing Problem with Time Windows Resource Constraints, and Duration Limit (VRPTWRCL)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, vehicle_load_time: int, vehicle_unload_time: int, depot_capacity: int, duration_limit: int, depot: int):
    # Create the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'num_vehicles': num_vehicle,
        'depot': depot,
        'vehicle_load_time': vehicle_load_time,
        'vehicle_unload_time': vehicle_unload_time,
        'depot_capacity': depot_capacity,
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback
    def time_callback(from_index, to_index):
        # Convert from routing variable indices to node indices
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return travel time between nodes
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set cost of each arc to be the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        60,  # allow waiting time
        duration_limit,  # maximum time per route
        False,  # Don't force start cumul to zero
        time
    )
    # Retrieve the time dimension
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue  # Skip depot as it is handled separately
        index = manager.NodeToIndex(location_idx)
        # Set the time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Set the start time window for each vehicle to match depot's time window
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )

    # Add resource constraints at the depot
    solver = routing.solver()
    intervals = []
    for i in range(data['num_vehicles']):
        # Add load interval at start
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.Start(i)),
                data['vehicle_load_time'],
                f'depot_load_{i}'
            )
        )
        # Add unload interval at end
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.End(i)),
                data['vehicle_unload_time'],
                f'depot_unload_{i}'
            )
        )
    # Create a list indicating resource usage (capacity) for each interval
    depot_usage = [1 for _ in range(len(intervals))]
    # Add cumulative resource constraint at the depot
    solver.Add(
        solver.Cumulative(intervals, depot_usage, data['depot_capacity'], 'depot_resource')
    )

    # Add duration limit constraint for each vehicle route
    for vehicle_id in range(data['num_vehicles']):
        index_start = routing.Start(vehicle_id)
        index_end = routing.End(vehicle_id)
        # Ensure total route duration does not exceed the specified limit
        routing.solver().Add(
            time_dimension.CumulVar(index_end) - time_dimension.CumulVar(index_start) <= duration_limit
        )

    # Remove the line that sets the objective to minimize, as arc cost evaluator already minimizes total travel time
    # routing.SetMinimizingObjective(True)

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 30  # optional time limit
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1
