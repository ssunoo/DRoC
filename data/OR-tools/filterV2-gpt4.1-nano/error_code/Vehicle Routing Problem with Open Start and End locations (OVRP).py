# Vehicle Routing Problem with Open Start and End locations (OVRP)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, num_vehicle: int):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        num_vehicle: the number of the vehicle

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Data model
    data = {}
    data["distance_matrix"] = distance_matrix
    data["num_vehicles"] = num_vehicle

    # Generate start and end nodes for each vehicle
    # For simplicity, assume all vehicles start and end at node 0
    # You can customize this as needed
    data["starts"] = [0] * num_vehicle
    data["ends"] = [0] * num_vehicle

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data["distance_matrix"]), data["num_vehicles"], data["starts"], data["ends"])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension
    distance_dim_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        20000,  # maximum distance per vehicle
        True,  # start cumul to zero
        distance_dim_name)
    distance_dimension = routing.GetDimensionOrDie(distance_dim_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Increase the number of search attempts for better solutions
    search_parameters.time_limit.FromSeconds(30)
    search_parameters.solution_limit = 1000

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1