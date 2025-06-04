# Capacitated Vehicle Routing Problem with Time Windows and Resource Constraints (CVRPTWRC)
# [('user', 'The solution failed the code execution test: Error: \'RoutingModel\' object has no attribute \'SetMinimizingObjective\'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpigx39y1c.py", line 148, in <module>\n    result = solve(time_matrix, time_windows, demands, vehicle_capacities, num_vehicles, depot, vehicle_load_time, vehicle_unload_time, depot_capacity)\n  File "/tmp/tmpigx39y1c.py", line 115, in solve\n    routing.SetMinimizingObjective(True)\nAttributeError: \'RoutingModel\' object has no attribute \'SetMinimizingObjective\'

')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicles: int, depot: int, vehicle_load_time: int, vehicle_unload_time: int, depot_capacity: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        demands: the list of integer customer demands
        vehicle_capacities: the capacity of each vehicle
        num_vehicles: the number of the vehicle
        depot: the index of the depot node
        vehicle_load_time: the time required to load a vehicle at depot
        vehicle_unload_time: the time required to unload a vehicle at depot
        depot_capacity: the maximum number of vehicles that can load or unload at the same time 

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'demands': demands,
        'vehicle_capacities': vehicle_capacities,
        'num_vehicles': num_vehicles,
        'depot': depot,
        'vehicle_load_time': vehicle_load_time,
        'vehicle_unload_time': vehicle_unload_time,
        'depot_capacity': depot_capacity,
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        60,  # allow waiting time
        60,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1],
        )

    # Add resource constraints at the depot
    solver = routing.solver()
    intervals = []
    for i in range(data['num_vehicles']):
        # Add load interval
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.Start(i)),
                data['vehicle_load_time'],
                f'depot_load_{i}',
            )
        )
        # Add unload interval
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.End(i)),
                data['vehicle_unload_time'],
                f'depot_unload_{i}',
            )
        )
    depot_usage = [1 for _ in range(len(intervals))]
    solver.Add(
        solver.Cumulative(intervals, depot_usage, data['depot_capacity'], 'depot_resource')
    )

    # Add capacity constraints for demands
    def demand_callback(from_index):
        node = manager.IndexToNode(from_index)
        return data['demands'][node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity',
    )

    # Minimize the total travel time
    # The following line is incorrect because RoutingModel does not have a method 'SetMinimizingObjective'
    # To minimize total travel time, set the objective directly using the appropriate method or parameter
    # routing.SetMinimizingObjective(True)

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1
