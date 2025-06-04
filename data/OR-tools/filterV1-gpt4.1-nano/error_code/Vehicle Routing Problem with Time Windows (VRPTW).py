# Vehicle Routing Problem with Time Windows (VRPTW)
# This code solves the VRPTW using Google OR-Tools
# Import necessary modules from OR-Tools
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Convert routing indices to node indices
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return travel time between nodes
        return time_matrix[from_node][to_node]

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc to be the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a service time callback (here assumed zero for all nodes)
    def service_time_callback(from_index):
        # Service time is zero for all nodes in this example
        return 0

    # Register the transit callback again (redundant, but kept as in original code)
    time_callback_index = routing.RegisterTransitCallback(time_callback)

    # Add a time dimension to model time constraints
    routing.AddDimension(
        time_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        1440,  # Maximum time per vehicle (e.g., 24 hours)
        False,  # Do not force start cumul to zero
        'Time')  # Dimension name

    # Retrieve the time dimension to set constraints
    time_dimension = routing.GetDimensionOrDie('Time')

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        # Convert node index to routing index
        index = manager.NodeToIndex(location_idx)
        # Set the time window for the node
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for the depot for each vehicle
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        # Set the depot's time window
        time_dimension.CumulVar(index).SetRange(time_windows[depot][0], time_windows[depot][1])

    # Set the search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use the PATH_CHEAPEST_ARC heuristic for the initial solution
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the VRPTW problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Check if a solution was found
    if solution:
        # Get the total cost (travel time) of the solution
        obj = routing.CostVar().Value()
    else:
        # No solution found
        obj = -1
    # Return the total travel time or -1 if no solution
    return obj