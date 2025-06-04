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
        'depot': depot
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

    # Set the cost of each arc to the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension to model to handle time windows and route duration
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        duration_limit,  # maximum route duration
        False,  # Don't force start cumul to zero.
        time
    )
    # Retrieve the time dimension
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        # Set the time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Set the start time window for each vehicle
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )

    # Add capacity dimension to handle demands and vehicle capacities
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
        'Capacity'
    )

    # Add route duration dimension to limit total route duration
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        duration_limit,  # maximum route duration
        True,  # start cumul to zero
        'RouteDuration'
    )
    # Retrieve the route duration dimension
    route_duration_dimension = routing.GetDimensionOrDie('RouteDuration')

    # Enforce route duration limit for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        # Set the maximum route duration for each vehicle
        route_duration_dimension.CumulVar(routing.End(vehicle_id)).SetRange(0, duration_limit)

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC as the first solution strategy
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Set a time limit for the search
    search_parameters.time_limit.seconds = 30  # optional time limit
    # Enable logging of the search process
    search_parameters.log_search = True

    # Solve the problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1
