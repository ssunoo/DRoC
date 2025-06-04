# Vehicle Routing with Pickups and Deliveries (PDP)
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(distance_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define the cost of each arc (edge) for all vehicles using the transit callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to track the total distance traveled by each vehicle
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack (waiting time allowed at nodes)
        3000,  # maximum distance per vehicle, can be adjusted as needed
        True,  # start cumul to zero (initial distance is zero)
        distance_dimension_name)
    # Retrieve the distance dimension to set constraints or access cumulative variables
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)

    # Add Pickup and Delivery constraints to ensure proper pairing and sequencing
    for pickup, delivery in pickups_deliveries:
        # Convert pickup and delivery nodes to routing indices
        pickup_index = manager.NodeToIndex(pickup)
        delivery_index = manager.NodeToIndex(delivery)
        # Add pickup and delivery pair to the routing model
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Enforce that pickup and delivery are on the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce that pickup occurs before delivery in the route
        routing.solver().Add(distance_dimension.CumulVar(pickup_index) <= distance_dimension.CumulVar(delivery_index))

    # Set the search parameters for the solver, including the heuristic for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Check if a solution was found and return the total distance
    if solution:
        total_distance = solution.ObjectiveValue()
        return total_distance
    else:
        # Return -1 if no solution was found
        return -1