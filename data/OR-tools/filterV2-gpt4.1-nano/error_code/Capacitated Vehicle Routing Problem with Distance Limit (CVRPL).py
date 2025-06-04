# Capacitated Vehicle Routing Problem with Distance Limit (CVRPL)
# This function solves the CVRPL using Google OR-Tools
# It considers vehicle capacities and a maximum distance limit per vehicle
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacities: list, depot: int, distance_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for distances
    def distance_callback(from_index, to_index):
        # Convert routing indices to node indices
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return the distance between the nodes
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the arc cost evaluator for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity dimension to account for demands and vehicle capacities
    def demand_callback(from_index):
        # Convert routing index to node index
        from_node = manager.IndexToNode(from_index)
        # Return demand at the node
        return demands[from_node]

    demand_callback_index = routing.RegisterTransitCallback(demand_callback)
    # Add capacity dimension with vehicle capacities
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Add Distance dimension to enforce maximum distance per vehicle
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        distance_limit,  # maximum distance per vehicle
        True,  # start cumul to zero
        distance_dimension_name)

    # Retrieve the distance dimension to set additional parameters
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)

    # Set global span cost coefficient to minimize the maximum route distance
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Set the search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC as the first solution heuristic
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the CVRPL problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, compute the maximum route distance among all vehicles
    if solution:
        max_route_distance = 0
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            route_distance = 0
            # Traverse the route for each vehicle
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                # Accumulate the distance for the route
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            # Track the maximum route distance
            max_route_distance = max(max_route_distance, route_distance)
        return max_route_distance
    else:
        # Return -1 if no solution exists
        return -1