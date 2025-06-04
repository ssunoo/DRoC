# Vehicle Routing with Pickups and Deliveries and Distance Limit (PDPL)
# This code solves a vehicle routing problem with pickup and delivery constraints and a maximum distance limit per vehicle.
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int, distance_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing model indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)
    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Returns the distance between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc for all vehicles using the transit callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Pickup and Delivery constraints to ensure pickups and deliveries are paired and served by the same vehicle
    for pickup, delivery in pickups_deliveries:
        pickup_index = manager.NodeToIndex(pickup)
        delivery_index = manager.NodeToIndex(delivery)
        # Add pickup and delivery pair to the routing model
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Enforce that pickup and delivery are served by the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))

    # Create and add Distance dimension to track total distance traveled by each vehicle
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        distance_limit,  # maximum distance per vehicle
        True,  # start cumul to zero
        distance_dimension_name)
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)
    # Set global span cost coefficient to minimize the longest route distance
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add pickup-delivery precedence constraints after adding the distance dimension
    for pickup, delivery in pickups_deliveries:
        pickup_index = manager.NodeToIndex(pickup)
        delivery_index = manager.NodeToIndex(delivery)
        # Add the precedence constraint to ensure pickup occurs before delivery
        routing.solver().Add(distance_dimension.CumulVar(pickup_index) <= distance_dimension.CumulVar(delivery_index))

    # Set the first solution heuristic to PATH_CHEAPEST_ARC for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the routing problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the maximum route distance if a solution is found, otherwise return -1
    if solution:
        max_route_distance = distance_dimension.CumulVar(routing.End(0)).Max()
        return max_route_distance
    else:
        return -1