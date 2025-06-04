# Vehicle Routing Problem with Duration Limit (VRPL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(time_matrix: list, num_vehicle: int, depot: int, duration_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Returns the travel time between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc (edge) for all vehicles using the transit callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Duration constraint to limit the total travel time per route
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack time between visits
        duration_limit,  # Maximum route duration
        True,  # Start cumul to zero (initial duration is zero)
        'Duration')  # Dimension name

    # Get the dimension object for 'Duration'
    duration_dimension = routing.GetDimensionOrDie('Duration')

    # Set the maximum route duration for each vehicle
    for vehicle_id in range(num_vehicle):
        # Set the maximum duration for the route of each vehicle
        duration_dimension.CumulVar(routing.End(vehicle_id)).SetRange(0, duration_limit)

    # Set the search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use the PATH_CHEAPEST_ARC heuristic for the initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the VRP with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Initialize total travel time
    total_time = 0
    if solution:
        # If a solution is found, compute the total travel time for all vehicles
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                previous_index = index
                # Move to the next node in the route
                index = solution.Value(routing.NextVar(index))
                # Add the arc cost (travel time) between nodes to total_time
                total_time += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
    else:
        # If no solution is found, set total_time to -1
        total_time = -1

    # Return the total travel time or -1 if no solution
    return total_time