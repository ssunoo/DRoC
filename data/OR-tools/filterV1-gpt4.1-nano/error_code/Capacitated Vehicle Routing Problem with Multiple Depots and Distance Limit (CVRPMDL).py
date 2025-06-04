# Capacitated Vehicle Routing Problem with Multiple Depots and Distance Limit (CVRPMDL)
# [('user', "UnusedParameterError: Params {'depot'} are not used, you should ensure all the params are used in the function.")]
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacities: list, depot: int, starts: list, ends: list, distance_limit: int):
    # Create the routing index manager with a single depot if 'depot' is used, or with 'starts' and 'ends' if multiple depots are used
    # Assuming 'starts' and 'ends' are used for multiple depots, otherwise replace with 'depot'
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, starts, ends)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for distances
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Create and register a demand callback
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    # Add Capacity dimension
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Create and add Distance dimension
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        distance_limit,  # maximum distance per vehicle
        True,  # start cumul to zero
        distance_dimension_name)

    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)
    # Set global span cost coefficient to minimize the longest route
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if solution exists
    if solution:
        return routing.CostOfVehicle(0, solution)
    else:
        return -1