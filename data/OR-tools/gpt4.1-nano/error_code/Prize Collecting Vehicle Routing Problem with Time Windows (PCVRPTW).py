# Prize Collecting Vehicle Routing Problem with Time Windows (PCVRPTW)
# This code solves a vehicle routing problem where the goal is to maximize collected prizes while respecting time windows and maximum duration constraints.
# It uses Google OR-Tools' routing solver.

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, prizes: list, max_duration: int):
    # Create the data model containing all problem data
    data = {
        'time_matrix': time_matrix,  # Travel time between locations
        'time_windows': time_windows,  # Allowed time windows for each location
        'num_vehicles': num_vehicle,  # Number of vehicles
        'depot': depot,  # Starting location index
        'prizes': prizes,  # Prize values for each location
        'max_duration': max_duration  # Maximum allowed route duration
    }

    # Create the routing index manager to manage node indices and vehicle routes
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to return travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator for all vehicles to the travel time callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension for time to model time windows and travel times
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # Allow waiting time at nodes
        data['max_duration'],  # Maximum route duration
        False,  # Do not force start cumul to zero
        time  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for each location, including depot
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        # Set the time window for each node
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set time window constraints for each vehicle start node (depot)
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Set the depot's time window for each vehicle start
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )

    # Add disjunctions with prizes as penalties for skipping nodes (except depot)
    for node in range(len(data['time_matrix'])):
        if node != data['depot']:
            # Penalize skipping nodes with the prize value
            routing.AddDisjunction([manager.NodeToIndex(node)], data['prizes'][node])

    # Set the search parameters and the first solution strategy
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Initialize total prize collected
    total_prize = 0
    if solution:
        # If a solution is found, sum the prizes for visited nodes
        for node in range(len(data['time_matrix'])):
            index = manager.NodeToIndex(node)
            if routing.IsNodeVisited(solution, index):
                total_prize += data['prizes'][node]
    else:
        # If no solution is found, total prize remains zero
        total_prize = 0

    # Return the total prize collected in the solution
    return total_prize