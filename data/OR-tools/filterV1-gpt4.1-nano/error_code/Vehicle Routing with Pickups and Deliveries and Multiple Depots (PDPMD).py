# Vehicle Routing with Pickups and Deliveries and Multiple Depots (PDPMD)
# This code solves a vehicle routing problem considering pickups, deliveries, and multiple depots using Google OR-Tools.
# The user comment indicates that the current solution may not be optimal and might not consider all constraints.
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, pickups_deliveries: list, num_vehicle: int, starts: list, ends: list):
    # Create the routing index manager to manage the conversion between node indices and routing variables
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, starts, ends)
    # Create the routing model which will contain the optimization problem
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Convert from routing variable index to actual node index in the distance matrix
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc (edge) in the routing problem to the distance callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to accumulate distance traveled, with a maximum limit
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack (extra waiting time)
        100000,  # maximum distance per vehicle, set to a large number to avoid constraints
        True,  # start cumul to zero at the beginning of each route
        distance_dimension_name)
    # Retrieve the distance dimension to set further constraints if needed
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)

    # Add Pickup and Delivery constraints to ensure proper pairing and order
    for pickup, delivery in pickups_deliveries:
        # Convert pickup and delivery nodes to routing indices
        pickup_index = manager.NodeToIndex(pickup)
        delivery_index = manager.NodeToIndex(delivery)
        # Add pickup and delivery pair to the routing model
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure that pickup and delivery are handled by the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce that pickup occurs before delivery in the route
        routing.AddPickupAndDelivery(pickup_index, delivery_index)

    # Set the search parameters for the solver, choosing a heuristic for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, return the total distance; otherwise, return -1 indicating failure
    if solution:
        total_distance = solution.ObjectiveValue()
        return total_distance
    else:
        return -1