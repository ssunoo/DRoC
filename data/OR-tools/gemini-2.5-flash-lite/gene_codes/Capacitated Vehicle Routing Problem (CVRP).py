# Capacitated Vehicle Routing Problem (CVRP)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacity: list, depot: int):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        demands: the list of integer customer demands
        num_vehicle: the number of the vehicle
        vehicle_capacities: the capacity of each vehicle
        depot: the index of the depot node

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(distance_matrix), num_vehicle, depot
    )

    # Create Routing Model.
    model = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = model.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    model.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = model.RegisterUnaryTransitCallback(demand_callback)
    model.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacity,  # vehicle capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(30) # Add a time limit for robustness

    # Solve the problem.
    solution = model.SolveWithParameters(search_parameters)

    # Get the objective value.
    if solution:
        obj = solution.ObjectiveValue()
    else:
        obj = -1 # Indicate no solution found

    return obj
