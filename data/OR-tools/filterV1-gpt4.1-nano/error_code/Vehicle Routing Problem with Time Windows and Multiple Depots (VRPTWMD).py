# Vehicle Routing Problem with Time Windows and Multiple Depots (VRPTWMD)
# This code solves a VRPTWMD using Google OR-Tools.
# It considers multiple depots, time windows, and vehicle constraints.
# Import necessary modules from OR-Tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, starts: list, ends: list, num_vehicles: int):
    # Create the data model containing all problem data
    data = {
        'time_matrix': time_matrix,  # Travel time between locations
        'time_windows': time_windows,  # Allowed time windows for each location
        'starts': starts,  # Starting depots for each vehicle
        'ends': ends,  # Ending depots for each vehicle
        'num_vehicles': num_vehicles  # Total number of vehicles
    }

    # Create the routing index manager to manage node indices for vehicles
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to return travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]  # Return travel time

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the cost of each arc to be the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension for time to handle time windows and waiting times
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        30,  # Maximum time per vehicle (capacity)
        False,  # Don't force start cumul to zero
        time  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node to index
        # Set the time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle's start and end nodes
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)  # Start node index for vehicle
        end_index = routing.End(vehicle_id)  # End node index for vehicle
        start_node = data['starts'][vehicle_id]
        end_node = data['ends'][vehicle_id]
        # Set start node time window based on depot's time window
        time_dimension.CumulVar(start_index).SetRange(
            data['time_windows'][start_node][0], data['time_windows'][start_node][1]
        )
        # Set end node time window based on depot's time window
        time_dimension.CumulVar(end_index).SetRange(
            data['time_windows'][end_node][0], data['time_windows'][end_node][1]
        )

    # Minimize the arrival time at start and end nodes for each vehicle
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))  # Minimize start time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))  # Minimize end time

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy to find initial solution

    # Solve the VRPTWMD problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Check if a feasible solution was found
    if solution:
        # Return the objective value (total travel time or cost)
        return solution.ObjectiveValue()
    else:
        # Return -1 if no feasible solution exists
        return -1