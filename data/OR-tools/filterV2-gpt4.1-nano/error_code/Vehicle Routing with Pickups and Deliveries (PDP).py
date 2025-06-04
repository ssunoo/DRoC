# Vehicle Routing with Pickups and Deliveries (PDP)
# [('user', 'The solution failed the code execution test: Error: solve() missing 2 required positional arguments: 'demands' and 'vehicle_capacities'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpnexh76uu.py", line 97, in <module>\n    result = solve(distance_matrix, pickups_deliveries, num_vehicle, depot)\nTypeError: solve() missing 2 required positional arguments: 'demands' and 'vehicle_capacities'\n')]
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int, demands: list, vehicle_capacities: list):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        pickups_deliveries: a list of pairs of pickup and delivery locations
        num_vehicle: the number of the vehicle
        depot: the index of the depot node
        demands: list of demands at each node
        vehicle_capacities: list of capacities for each vehicle

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback
    def distance_callback(from_index, to_index):
        # Returns the distance between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        100000,  # maximum distance per vehicle
        True,  # start cumul to zero
        distance_dimension_name)
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)

    # Add Capacity dimension
    capacity_dimension_name = 'Capacity'
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # vehicle maximum capacities
        True,  # start cumul to zero
        capacity_dimension_name)
    capacity_dimension = routing.GetDimensionOrDie(capacity_dimension_name)

    # Add Pickup and Delivery constraints
    for request in pickups_deliveries:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure pickup and delivery are on the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce pickup before delivery
        routing.solver().Add(distance_dimension.CumulVar(pickup_index) <= distance_dimension.CumulVar(delivery_index))

    # Setting first solution heuristic and search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Optional: set a time limit or other parameters to improve solution quality
    # search_parameters.time_limit.FromSeconds(30)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1