# Prize Collecting Vehicle Routing Problem with Time Windows (PCVRPTW)
# This code models and solves a vehicle routing problem where the goal is to maximize collected prizes while respecting time windows and route duration constraints.
# It uses Google's OR-Tools routing library.

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, prizes: list, max_duration: int):
    # Create the data model containing all problem data
    data = {
        'distance_matrix': time_matrix,  # Matrix of travel times between locations
        'time_windows': time_windows,    # Allowed time windows for each location
        'num_vehicles': num_vehicle,     # Number of vehicles available
        'depot': depot,                   # Index of the depot location
        'prizes': prizes,                # Prize values for visiting each location
        'max_duration': max_duration,    # Maximum allowed route duration
    }

    # Create the routing index manager to handle node indices
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Instantiate the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Define a transit callback to return travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    # Register the transit callback with the routing model
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator for all vehicles to the travel time callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a time dimension to model travel times and waiting times
    time_dimension = routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        data['max_duration'],  # Maximum route duration
        False,  # Do not force start cumul to zero
        'Time')  # Dimension name

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        # Check if the time window is valid
        if time_window and len(time_window) == 2:
            start, end = time_window
            # Set the time window range if feasible
            if start >= 0 and end >= start:
                time_dimension.CumulVar(index).SetRange(start, end)

    # Add time window constraints for vehicle start nodes (depots)
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        depot_time_window = data['time_windows'][data['depot']]
        if depot_time_window and len(depot_time_window) == 2:
            start, end = depot_time_window
            if start >= 0 and end >= start:
                time_dimension.CumulVar(index).SetRange(start, end)

    # Add a duration dimension to track total route duration
    duration_dimension = routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack at nodes
        data['max_duration'],  # Maximum route duration
        True,  # Start cumul to zero
        'Duration')  # Dimension name

    # Set maximum route duration constraints for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        duration_dimension.CumulVar(routing.End(vehicle_id)).SetRange(0, data['max_duration'])

    # Register a callback to assign prizes to nodes
    def prize_callback(node_index):
        node = manager.IndexToNode(node_index)
        return data['prizes'][node]

    prize_callback_index = routing.RegisterUnaryTransitCallback(prize_callback)

    # Add disjunctions to allow skipping nodes with a penalty related to prizes
    for node in range(1, len(data['distance_matrix'])):
        routing.AddDisjunction([manager.NodeToIndex(node)], 1000000 - data['prizes'][node])

    # Set search parameters with a heuristic for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    assignment = routing.SolveWithParameters(search_parameters)

    # If a solution is found, calculate total prizes collected
    if assignment:
        total_prize = 0
        # Iterate over all nodes in the solution
        for node in range(routing.Size()):
            # Skip start and end nodes
            if routing.IsStart(node) or routing.IsEnd(node):
                continue
            # Check if the node is visited in the route
            if assignment.Value(routing.NextVar(node)) != node:
                node_index = manager.IndexToNode(node)
                # Accumulate prizes for visited nodes
                total_prize += data['prizes'][node_index]
        return total_prize  # Return total prize collected
    return -1  # Return -1 if no solution is found