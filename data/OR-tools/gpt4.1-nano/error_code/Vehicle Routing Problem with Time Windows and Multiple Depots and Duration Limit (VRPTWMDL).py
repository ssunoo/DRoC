# Vehicle Routing Problem with Time Windows and Multiple Depots and Duration Limit (VRPTWMDL)
# This code solves a complex vehicle routing problem using Google OR-Tools.
# It considers multiple depots, time windows, and a maximum duration limit per vehicle.
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, starts: list, ends: list, num_vehicles: int, duration_limit: int):
    # Create the data model containing all problem data
    data = {
        'time_matrix': time_matrix,  # Travel time between locations
        'time_windows': time_windows,  # Allowed time windows for each location
        'starts': starts,  # Starting nodes for each vehicle
        'ends': ends,  # Ending nodes for each vehicle
        'num_vehicles': num_vehicles,  # Number of vehicles
        'depot': 0  # Assuming a single depot for start and end, modify if multiple depots
    }

    # Create the routing index manager to manage node indices for multiple vehicles
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the cost of each arc to be the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension to model to handle time windows and duration limits
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        duration_limit,  # Maximum total duration per vehicle
        False,  # Do not force start cumul to zero
        time  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node to index
        # Set the allowed time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)  # Get start index for vehicle
        start_node = manager.IndexToNode(start_index)  # Convert to node index
        # Set time window for the start node of each vehicle
        time_dimension.CumulVar(start_index).SetRange(
            data['time_windows'][start_node][0],  # Earliest start time
            data['time_windows'][start_node][1]   # Latest start time
        )

    # Minimize the total time at start and end nodes for all vehicles
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))  # Minimize start time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))  # Minimize end time

    # Set search parameters with a time limit for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution
    search_parameters.time_limit.seconds = 30  # Set a time limit for the search process

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value (total travel time) if a solution is found
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1  # Return -1 if no solution is found