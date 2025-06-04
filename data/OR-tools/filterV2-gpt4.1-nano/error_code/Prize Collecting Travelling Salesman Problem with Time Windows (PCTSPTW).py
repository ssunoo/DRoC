# Prize Collecting Travelling Salesman Problem with Time Windows (PCTSPTW)
# This code solves a variant of the TSP where the goal is to maximize prize collection within time and duration constraints.
# Import necessary modules from OR-Tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, depot: int, prizes: list, max_duration: int):
    """
    Args:
         time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        prizes: the value of prize that a vehicle can collect at each node
        max_duration: maximum duration that a vehicle can travel
         depot: the index of the depot node

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1  # Initialize the objective value to -1 (indicating no solution found yet)
    # Create the routing index manager, which manages the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), 1, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert routing index to node index
        return time_matrix[from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set arc cost evaluator to the transit callback for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension to enforce time window constraints
    time = "Time"  # Name of the dimension
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack time allowed
        max_duration,  # Maximum travel time allowed
        False,  # Do not force start cumul to zero
        time  # Name of the dimension
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Set time window constraints for each location
    for location_idx, window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        time_dimension.CumulVar(index).SetRange(window[0], window[1])  # Set time window range

    # Add prize collection as a dimension to maximize total prize
    def prize_callback(from_index):
        node = manager.IndexToNode(from_index)  # Convert routing index to node index
        return prizes[node]  # Return prize value at the node

    prize_callback_index = routing.RegisterUnaryTransitCallback(prize_callback)  # Register prize callback

    # Add a dimension for prize collection to keep track of total prizes collected
    routing.AddDimension(
        prize_callback_index,  # Prize callback index
        0,  # No slack
        sum(prizes),  # Maximum total prize (sum of all prizes)
        True,  # Start cumul to zero
        "Prize"  # Name of the dimension
    )
    prize_dimension = routing.GetDimensionOrDie("Prize")  # Retrieve the prize dimension

    # Set the objective to maximize total prize collected
    # This is achieved by setting the solver to maximize the prize dimension value
    # No disjunction penalties are needed for prize collection

    # Add max duration constraint to limit total travel time
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack
        max_duration,  # Maximum total travel time
        True,  # Start cumul to zero
        "TravelTime"  # Name of the dimension
    )
    travel_time_dimension = routing.GetDimensionOrDie("TravelTime")  # Retrieve travel time dimension

    # Set start time window constraints for the depot
    for vehicle_id in range(routing.vehicles()):  # For each vehicle
        index = routing.Start(vehicle_id)  # Get start node index
        time_dimension.CumulVar(index).SetRange(time_windows[depot][0], time_windows[depot][1])  # Set start time window

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution
    search_parameters.time_limit.FromSeconds(30)  # Limit search time to 30 seconds

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Extract the objective value from the solution
    if solution:
        # The objective is the total prize collected, obtained from the prize dimension at the end node
        obj = solution.Value(prize_dimension.CumulVar(routing.End(0)))  # Get total prize collected
    return obj  # Return the objective value (total prize collected or -1 if no solution)