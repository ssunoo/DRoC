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
    # Create the data model
    data = {}
    data["distance_matrix"] = distance_matrix
    data["num_vehicles"] = num_vehicle
    # Define specific start and end locations for each vehicle
    # For open routes, start and end can be different or set to None
    # Here, we assume start and end locations are provided or default to 0
    # For example, set all starts to 0 and ends to 0 for simplicity
    data["starts"] = [0 for _ in range(num_vehicle)]
    data["ends"] = [0 for _ in range(num_vehicle)]

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
    distance_dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # maximum distance per vehicle, can be adjusted
        True,  # start cumul to zero
        distance_dimension_name)

    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)

    # Set global span cost coefficient to minimize the maximum route distance
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    obj = -1
    if solution:
        obj = solution.ObjectiveValue()
    return obj