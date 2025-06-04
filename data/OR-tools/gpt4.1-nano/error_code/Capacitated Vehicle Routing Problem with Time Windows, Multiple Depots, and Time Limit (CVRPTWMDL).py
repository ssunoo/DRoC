# Capacitated Vehicle Routing Problem with Time Windows, Multiple Depots, and Time Limit (CVRPTWMDL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicle: int, starts: list, ends: list, duration_limit: int):
    # Create the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'demands': demands,
        'vehicle_capacities': vehicle_capacities,
        'num_vehicles': num_vehicle,
        'starts': starts,
        'ends': ends,
        'duration_limit': duration_limit,
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity'
    )

    # Add Time Windows constraint
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        data['duration_limit'],  # maximum route duration
        False,  # Don't force start cumul to zero
        time
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        start_node = data['starts'][vehicle_id]
        time_window = data['time_windows'][start_node]
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add maximum route duration constraint for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.End(vehicle_id)
        time_dimension.CumulVar(index).SetRange(0, data['duration_limit'])

    # Create and add a dimension for total route time to explicitly constrain route durations
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        data['duration_limit'],  # maximum route duration
        True,  # enforce start cumul to zero
        'TotalTime'
    )
    total_time_dimension = routing.GetDimensionOrDie('TotalTime')

    # Set the maximum total route time for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.End(vehicle_id)
        total_time_dimension.CumulVar(index).SetRange(0, data['duration_limit'])

    # Set search parameters with a time limit
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.seconds = 30  # Set time limit as needed

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1