# Capacitated Vehicle Routing Problem with Time Windows and Duration Limit (CVRPTWL)
# This code solves a vehicle routing problem considering capacity, time windows, and route duration constraints.
# It uses Google's OR-Tools constraint programming library.

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicle: int, depot: int, duration_limit: int):
    # Create the data model as a dictionary to hold all problem data
    data = {}
    data['time_matrix'] = time_matrix  # Matrix of travel times between locations
    data['time_windows'] = time_windows  # Allowed time windows for each location
    data['demands'] = demands  # Demand at each location
    data['vehicle_capacities'] = vehicle_capacities  # Capacity of each vehicle
    data['num_vehicles'] = num_vehicle  # Total number of vehicles
    data['depot'] = depot  # Index of the depot location

    # Create the routing index manager to handle node indices
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Transit callback for travel times between locations
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set arc cost evaluator for all vehicles to minimize total travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add capacity constraint to ensure vehicle loads do not exceed capacities
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)  # Convert index
        return data['demands'][from_node]  # Demand at the node

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)  # Register demand callback
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # Null capacity slack
        data['vehicle_capacities'],  # Vehicle capacities
        True,  # Start cumul to zero
        'Capacity')  # Name of the dimension

    # Add time dimension to handle time windows and travel times
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # Allow waiting time at locations
        30,  # Maximum time per vehicle (can be adjusted)
        False,  # Do not force start cumul to zero
        time)  # Dimension name
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location except depot
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue  # Skip depot
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])  # Set time window

    # Set time window constraints for depot start nodes for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Start node for vehicle
        # Set the start time window for the vehicle at the depot
        time_dimension.CumulVar(index).SetRange(data['time_windows'][data['depot']][0], data['time_windows'][data['depot']][1])

    # Add route duration dimension to limit total route duration
    routing.AddDimension(
        transit_callback_index,
        0,  # No slack
        duration_limit,  # Maximum route duration
        True,  # Start cumul to zero
        'Duration')  # Dimension name
    duration_dimension = routing.GetDimensionOrDie('Duration')  # Retrieve the duration dimension

    # Set route duration constraints for each vehicle to ensure total duration does not exceed limit
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Start node for vehicle
        duration_dimension.CumulVar(index).SetRange(0, duration_limit)  # Set duration range

    # Configure search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution
    search_parameters.time_limit.seconds = 30  # Time limit for the solver

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()  # Total travel time or cost
    else:
        return -1  # No solution found