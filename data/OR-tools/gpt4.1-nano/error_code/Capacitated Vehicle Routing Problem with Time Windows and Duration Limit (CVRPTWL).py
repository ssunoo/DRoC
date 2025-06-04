# Capacitated Vehicle Routing Problem with Time Windows and Duration Limit (CVRPTWL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicle: int, depot: int, duration_limit: int):
    # Create the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'demands': demands,
        'vehicle_capacities': vehicle_capacities,
        'num_vehicles': num_vehicle,
        'depot': depot,
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

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity dimension to model vehicle demands
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    # Register demand callback
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    # Add capacity dimension with vehicle capacities
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Add Time dimension to model time windows and travel times
    time = 'Time'
    # Determine maximum time window end for setting the time horizon
    max_time = max(max(tw) for tw in data['time_windows']) if data['time_windows'] else 30
    # Add time dimension with waiting time allowance
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        max_time,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        time
    )
    # Retrieve the time dimension for further constraints
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        # Set the time window for each node
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Set the start time window for each vehicle at the depot
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )

    # Add route duration constraint to limit total route time
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        duration_limit,  # maximum route duration
        True,  # force start cumul to zero
        'Duration'
    )
    # Retrieve the duration dimension for further constraints
    duration_dimension = routing.GetDimensionOrDie('Duration')

    # Enforce route duration at the end nodes for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.End(vehicle_id)
        # Set maximum route duration at the end node
        duration_dimension.CumulVar(index).SetRange(0, duration_limit)

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Choose the first solution strategy
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Optional time limit for the solver
    search_parameters.time_limit.seconds = 30

    # Solve the problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1