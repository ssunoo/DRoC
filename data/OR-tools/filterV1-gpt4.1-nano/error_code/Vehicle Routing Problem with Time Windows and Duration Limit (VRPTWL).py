# Vehicle Routing Problem with Time Windows and Duration Limit (VRPTWL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, duration_limit: int):
    # Create the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'num_vehicles': num_vehicle,
        'depot': depot
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback
    def time_callback(from_index, to_index):
        # Convert from routing variable indices to node indices
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return travel time between nodes
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension to model to handle time windows and waiting times
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        30,  # maximum time per vehicle (initially set, will be constrained by duration limit)
        False,  # Don't force start cumul to zero
        time
    )
    # Retrieve the time dimension
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue
        # Convert node index to routing index
        index = manager.NodeToIndex(location_idx)
        # Set the time window for the node
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Set the start node's time window to match depot's time window
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )

    # Add route duration limit constraint
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        duration_limit,  # maximum route duration
        True,  # start cumul to zero
        'Duration'
    )
    # Retrieve the duration dimension
    duration_dimension = routing.GetDimensionOrDie('Duration')

    # Enforce maximum route duration for each vehicle at start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Set the maximum route duration for the start node
        duration_dimension.CumulVar(index).SetRange(0, duration_limit)

    # Minimize the final time for each vehicle's route
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Set search parameters with a time limit for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.seconds = 60  # Increased time limit for better solutions

    # Solve the problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found
    if solution:
        return solution.ObjectiveValue()
    # Return -1 if no solution is found
    return -1