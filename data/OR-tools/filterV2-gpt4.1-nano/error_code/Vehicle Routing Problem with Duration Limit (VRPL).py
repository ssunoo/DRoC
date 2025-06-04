# Vehicle Routing Problem with Duration Limit (VRPL)
# This code solves a VRP with a maximum duration constraint for each vehicle
# Import the OR-Tools routing library
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, num_vehicle: int, depot: int, duration_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Convert from routing variable index to node index in the time matrix
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return the travel time between the nodes
        return time_matrix[from_node][to_node]

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc to be the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to track the total travel duration for each vehicle
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack time
        duration_limit,  # Maximum travel duration per vehicle
        True,  # Set the start cumul to zero for all vehicles
        'Duration')  # Dimension name

    # Retrieve the dimension object to set additional constraints
    duration_dimension = routing.GetDimensionOrDie('Duration')

    # Set the maximum travel duration for each vehicle explicitly
    for vehicle_id in range(num_vehicle):
        # Set the maximum cumulative travel time at the end node of each vehicle
        duration_dimension.CumulVar(routing.End(vehicle_id)).SetMax(duration_limit)

    # Define the search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Set a time limit of 30 seconds for the solver
    search_parameters.time_limit.seconds = 30

    # Solve the VRP with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, return the total cost; otherwise, return -1
    if solution:
        # Get the total cost of the solution
        return routing.CostVar().Value()
    else:
        # No solution found within the time limit
        return -1