# Vehicle Routing Problem with Time Windows and Multiple Depots and Duration Limit (VRPTWMDL)
# This code solves a complex vehicle routing problem with time windows, multiple depots, and duration constraints
# Import necessary modules from OR-Tools
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
        'depot': 0  # Assuming a single depot for simplicity, modify if multiple depots are specified
    }

    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert routing index to node index
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the cost of each arc to be the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint to ensure vehicles respect time windows and duration limits
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        duration_limit,  # Maximum duration per vehicle
        False,  # Don't force start cumul to zero
        'Time'  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie('Time')  # Retrieve the time dimension

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the time window for each node
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start and end nodes
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)  # Start node index for vehicle
        end_index = routing.End(vehicle_id)  # End node index for vehicle
        # Set start node time window based on data
        start_node_idx = data['starts'][vehicle_id]
        end_node_idx = data['ends'][vehicle_id]
        time_dimension.CumulVar(start_index).SetRange(
            data['time_windows'][start_node_idx][0],  # Start of start node time window
            data['time_windows'][start_node_idx][1]  # End of start node time window
        )
        # Set end node time window based on data
        time_dimension.CumulVar(end_index).SetRange(
            data['time_windows'][end_node_idx][0],  # Start of end node time window
            data['time_windows'][end_node_idx][1]  # End of end node time window
        )

    # Minimize the start and end times for each vehicle to optimize scheduling
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))  # Minimize start time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))  # Minimize end time

    # Set search parameters, including a time limit for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution
    search_parameters.time_limit.seconds = 30  # Set a time limit for the search process

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()  # Return total travel time or cost
    else:
        return -1  # Indicate no solution was found