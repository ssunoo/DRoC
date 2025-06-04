# Vehicle Routing Problem with Duration Limit and Open Start and End locations (OVRPL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, num_vehicle: int, duration_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing variables
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, 0)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc (edge) in the route
    def time_callback(from_index, to_index):
        # Convert from routing variable Index to time matrix Node
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    # Register the transit callback to compute travel times between nodes
    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    # Set the cost evaluator for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Duration constraint to limit total route duration
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack time between visits
        duration_limit,  # Maximum route duration
        True,  # Set the cumulative variable to zero at the start
        'Duration')  # Dimension name
    # Retrieve the duration dimension to access its properties
    duration_dimension = routing.GetDimensionOrDie('Duration')

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use a heuristic to find initial solutions
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Set a time limit for the solver to find a solution
    search_parameters.time_limit.FromSeconds(30)  # 30 seconds time limit
    # Enable local search metaheuristics for improved solutions
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # Enable logging to monitor the search process
    search_parameters.log_search = True

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the total objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1