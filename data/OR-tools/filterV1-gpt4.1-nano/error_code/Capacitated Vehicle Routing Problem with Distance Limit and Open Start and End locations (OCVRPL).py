# Capacitated Vehicle Routing Problem with Distance Limit and Open Start and End locations (OCVRPL)
# [('user', 'You solution returns nothing or 0. The program may be incomplete or your solution does not work for the task.')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacities: list, distance_limit: int):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        demands: the list of integer customer demands
        num_vehicle: the number of the vehicle
        vehicle_capacities: the capacity of each vehicle
        distance_limit: the travel distance of each route is upper bounded by the distance limit

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the data model
    data = {}
    data["distance_matrix"] = distance_matrix
    data["demands"] = demands
    data["num_vehicles"] = num_vehicle
    data["vehicle_capacities"] = vehicle_capacities
    # For open start/end, set start and end nodes for each vehicle
    # Here, assuming all vehicles start at node 0 and end at node 0, but can be customized
    data["starts"] = [0] * num_vehicle
    data["ends"] = [0] * num_vehicle

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, data["starts"], data["ends"])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for distances
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension with maximum travel distance constraint
    distance_dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        distance_limit,  # maximum travel distance per vehicle
        True,  # start cumul to zero
        distance_dimension_name)

    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)
    # Optional: set global span cost coefficient to minimize maximum route length
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add capacity constraint
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity")

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    # Increase the time limit to allow more extensive search
    search_parameters.time_limit.FromSeconds(10)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    obj = -1
    if solution:
        obj = solution.ObjectiveValue()
    return obj