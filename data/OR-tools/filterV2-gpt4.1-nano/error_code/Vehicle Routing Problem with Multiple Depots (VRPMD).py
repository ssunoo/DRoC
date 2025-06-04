# Vehicle Routing Problem with Multiple Depots (VRPMD)
# [('user', 'The solution failed the code execution test: Error: \'RoutingModel\' object has no attribute \'IsSolutionOptimal\'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmprhboqe9r.py", line 70, in <module>\n    result = solve(distance_matrix, num_vehicle, starts, ends)\n  File "/tmp/tmprhboqe9r.py", line 54, in solve\n    if solution and routing.IsSolutionOptimal() and routing.IsSolutionFeasible():\nAttributeError: \'RoutingModel\' object has no attribute \'IsSolutionOptimal\'\n')]}

from ortools.constraint_solver import routing_enums_pb2  # Import enumeration for search strategies
from ortools.constraint_solver import pywrapcp  # Import the OR-Tools routing solver

def solve(distance_matrix: list, num_vehicle: int, starts: list, ends: list):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        num_vehicle: the number of the vehicle
        starts: the index of the starting depot for vehicles
        ends: the index of the ending depot for vehicles 

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, starts, ends)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    # Register the transit callback with the routing model
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc (edge) for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension to track total distance traveled by each vehicle
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        20000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    # Retrieve the distance dimension to set additional parameters
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    # Set a global span cost coefficient to minimize the maximum route distance
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic for the search
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Optional: set a time limit for the solver
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Initialize objective value
    obj = -1
    # Check if a solution exists and if it is both feasible and optimal
    # Note: 'IsSolutionOptimal' is not a valid method in OR-Tools RoutingModel
    # Instead, check the solution status directly
    if solution:
        # Retrieve the solution status
        status = routing.Status()
        # Check if the solution is feasible and optimal
        if status == pywrapcp.RoutingModel.OPTIMAL or status == pywrapcp.RoutingModel.FEASIBLE:
            obj = solution.ObjectiveValue()
    return obj