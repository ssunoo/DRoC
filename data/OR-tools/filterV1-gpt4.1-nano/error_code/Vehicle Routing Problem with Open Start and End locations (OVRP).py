# Vehicle Routing Problem with Open Start and End locations (OVRP)
# [('user', 'The solution failed the code execution test: Error: 'bool' object has no attribute 'SetGlobalSpanCostCoefficient'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmplgfbw316.py", line 90, in <module>\n    result = solve(distance_matrix, num_vehicle)\n  File "/tmp/tmplgfbw316.py", line 65, in solve\n    distance_dimension.SetGlobalSpanCostCoefficient(100)\nAttributeError: 'bool' object has no attribute 'SetGlobalSpanCostCoefficient'\n\n')] 
from ortools.constraint_solver import routing_enums_pb2  # Import routing enums for search strategies
from ortools.constraint_solver import pywrapcp  # Import the OR-Tools routing solver

def solve(distance_matrix: list, num_vehicle: int):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        num_vehicle: the number of the vehicle

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Validate that the distance matrix is square
    size = len(distance_matrix)
    for row in distance_matrix:
        if len(row) != size:
            raise ValueError("Distance matrix must be square")

    # Define start and end locations for each vehicle
    # For simplicity, assign all vehicles to start at node 0 and end at node 0
    starts = [0] * num_vehicle
    ends = [0] * num_vehicle

    # Validate start and end nodes to ensure they are within the valid node indices
    max_node_index = size - 1
    for node_list in [starts, ends]:
        for node in node_list:
            if node > max_node_index:
                raise ValueError(f"Node index {node} exceeds maximum index {max_node_index} in distance matrix")

    # Create the data model for the routing problem
    data = {
        'distance_matrix': distance_matrix,
        'num_vehicles': num_vehicle,
        'starts': starts,
        'ends': ends
    }

    # Create the routing index manager to handle node indices
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc to the distance between nodes
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension to constrain total travel distance
    distance_dimension = routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        20000,  # maximum travel distance per vehicle
        True,  # start cumul to zero
        'Distance'
    )

    # Set global span cost coefficient to minimize maximum route distance
    # This encourages the solver to minimize the longest route among all vehicles
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the total distance of the solution if found
    if solution:
        total_distance = solution.ObjectiveValue()
        return total_distance
    return -1