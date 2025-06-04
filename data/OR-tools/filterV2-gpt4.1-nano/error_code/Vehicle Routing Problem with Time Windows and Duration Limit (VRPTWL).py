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

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        30,  # maximum time per vehicle (initially, can be adjusted)
        False,  # Don't force start cumul to zero
        time
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
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
    duration_dimension = routing.GetDimensionOrDie('Duration')
    # Set maximum route duration for each vehicle at the end node
    for vehicle_id in range(data['num_vehicles']):
        end_index = routing.End(vehicle_id)
        duration_dimension.CumulVar(end_index).SetRange(0, duration_limit)

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Optional: set a time limit for the search
    # search_parameters.time_limit.seconds = 30

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if solution exists
    if solution:
        return solution.ObjectiveValue()
    return -1
