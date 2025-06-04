# Vehicle Routing Problem with Time Windows, Service Time, and Duration Limit (VRPTWSL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    # Create the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'num_vehicles': num_vehicle,
        'depot': depot,
        'service_time': service_time,
        'duration_limit': duration_limit
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback that includes service times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert to node index
        travel_time = data['time_matrix'][from_node][to_node]  # Get travel time between nodes
        # Add service time at the from_node
        return travel_time + data['service_time'][from_node]

    # Register the transit callback with the routing model
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension with the combined callback
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        data['duration_limit'],  # maximum route duration
        False,  # Don't force start cumul to zero
        time
    )
    # Retrieve the time dimension
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node index to routing index
        # Set the time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Get start node index for each vehicle
        # Set the time window for the start node of each vehicle
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )

    # Add route duration limit at the end nodes using a separate dimension
    routing.AddDimension(
        transit_callback_index,
        0,  # no waiting time
        data['duration_limit'],  # maximum route duration
        True,  # force start cumul to zero
        'Duration'
    )
    # Retrieve the duration dimension
    duration_dimension = routing.GetDimensionOrDie('Duration')

    # Link the 'Time' dimension to the 'Duration' dimension to enforce total route duration
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)  # Get start index for each vehicle
        end_index = routing.End(vehicle_id)  # Get end index for each vehicle
        # Set the maximum total duration for each route at the end node
        duration_dimension.CumulVar(end_index).SetRange(0, data['duration_limit'])

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 30  # Set a time limit for the search
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1