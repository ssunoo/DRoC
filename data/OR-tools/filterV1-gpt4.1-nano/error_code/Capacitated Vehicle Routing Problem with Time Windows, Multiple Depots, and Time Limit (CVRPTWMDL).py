# Capacitated Vehicle Routing Problem with Time Windows, Multiple Depots, and Time Limit (CVRPTWMDL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicle: int, starts: list, ends: list, duration_limit: int):
    # Create the data model
    data = {
        'time_matrix': time_matrix,  # Matrix representing travel times between locations
        'time_windows': time_windows,  # Allowed time windows for each location
        'demands': demands,  # Demand at each location
        'vehicle_capacities': vehicle_capacities,  # Capacity of each vehicle
        'num_vehicles': num_vehicle,  # Total number of vehicles
        'starts': starts,  # Starting nodes for each vehicle
        'ends': ends,  # Ending nodes for each vehicle
        'depot_start': starts,  # Depot start nodes (same as starts)
        'depot_end': ends,  # Depot end nodes (same as ends)
        'duration_limit': duration_limit,  # Maximum duration per vehicle
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing variable to node index
        to_node = manager.IndexToNode(to_index)  # Convert from routing variable to node index
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set arc cost evaluator to the transit callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at locations
        data['duration_limit'],  # Maximum duration per vehicle
        False,  # Don't force start cumul to zero
        time  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Get the time dimension

    # Set time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node to index
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])  # Set time window

    # Set time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)  # Get start index for vehicle
        start_node = data['starts'][vehicle_id]  # Get start node
        start_time_window = data['time_windows'][start_node]  # Get start node's time window
        time_dimension.CumulVar(start_index).SetRange(start_time_window[0], start_time_window[1])  # Set start time window

    # Add demand dimension for capacity constraints
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)  # Convert from index to node
        return data['demands'][from_node]  # Return demand at node

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)  # Register demand callback
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,  # Demand callback index
        0,  # Null capacity slack
        data['vehicle_capacities'],  # Vehicle capacities
        True,  # Start cumul to zero
        'Capacity'  # Dimension name
    )

    # Set a time limit for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 60  # Increased time limit for better solutions
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)  # Solve with specified parameters

    # Return the objective value if solution exists
    if solution:
        return solution.ObjectiveValue()  # Return the objective value of the solution
    else:
        return -1  # Return -1 if no solution is found
