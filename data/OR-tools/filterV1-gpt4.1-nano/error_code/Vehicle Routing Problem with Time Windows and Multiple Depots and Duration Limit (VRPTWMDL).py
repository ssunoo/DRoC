# Vehicle Routing Problem with Time Windows and Multiple Depots and Duration Limit (VRPTWMDL)
# This code solves a complex vehicle routing problem with time windows, multiple depots, and route duration limits using Google OR-Tools.
# Import necessary modules from OR-Tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, starts: list, ends: list, num_vehicles: int, duration_limit: int):
    # Create the data model as a dictionary to hold problem data
    data = {}
    data['time_matrix'] = time_matrix  # Matrix representing travel times between locations
    data['time_windows'] = time_windows  # List of time windows for each location
    data['num_vehicles'] = num_vehicles  # Number of vehicles available
    data['starts'] = starts  # List of start depot indices for each vehicle
    data['ends'] = ends  # List of end depot indices for each vehicle

    # Create the routing index manager to handle node indices and vehicle routes
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create the routing model which will contain the routing problem
    routing = pywrapcp.RoutingModel(manager)

    # Define a callback function to get travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert to node index
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    # Register the transit callback with the routing model
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc to be the travel time, for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to model time with waiting allowed and route duration limit
    time = 'Time'  # Name of the dimension
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        duration_limit,  # Maximum route duration
        False,  # Don't force start cumul to zero
        time)  # Dimension name
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for start_idx in data['starts']:
        index = manager.NodeToIndex(start_idx)  # Convert start node to routing index
        # Set the start node's time window
        time_dimension.CumulVar(index).SetRange(data['time_windows'][start_idx][0], data['time_windows'][start_idx][1])

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use the PATH_CHEAPEST_ARC heuristic for initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Set a time limit for the search process
    search_parameters.time_limit.seconds = 30

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1