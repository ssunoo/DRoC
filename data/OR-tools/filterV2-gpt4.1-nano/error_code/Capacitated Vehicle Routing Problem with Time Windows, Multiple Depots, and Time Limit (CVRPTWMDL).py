# Capacitated Vehicle Routing Problem with Time Windows, Multiple Depots, and Time Limit (CVRPTWMDL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicle: int, starts: list, ends: list, duration_limit: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        demands: the list of integer customer demands
        vehicle_capacities: the capacity of each vehicle
        num_vehicle: the number of the vehicle
        starts: the index of the starting depot for vehicles
        ends: the index of the ending depot for vehicles
        duration_limit: the time duration of each route is upper bounded by the duration limit

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'demands': demands,
        'vehicle_capacities': vehicle_capacities,
        'num_vehicles': num_vehicle,
        'starts': starts,
        'ends': ends,
        'depot': starts[0],  # assuming all vehicles start from their respective starts
        'duration_limit': duration_limit
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension with duration limit
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        data['duration_limit'],  # maximum time per vehicle
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
        start_index = routing.Start(vehicle_id)
        start_node = manager.IndexToNode(start_index)
        # Set the time window for the start node based on the start location
        start_time_window = data['time_windows'][start_node]
        time_dimension.CumulVar(start_index).SetRange(start_time_window[0], start_time_window[1])

    # Register demand callback
    def demand_callback(from_index):
        node = manager.IndexToNode(from_index)
        return data['demands'][node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    # Add capacity dimension
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity'
    )

    # Set search parameters with a time limit
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 60  # set as needed

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1