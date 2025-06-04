# Capacitated Vehicle Routing Problem with Multiple Depots and Distance Limit (CVRPMDL)
# This code solves a variant of the Vehicle Routing Problem (VRP) with capacity and distance constraints, multiple depots, and distance limits.
# It uses Google's OR-Tools constraint programming library.

from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacities: list, depot: int, starts: list, ends: list, distance_limit: int):
    # Create the routing index manager, which manages the conversion between the problem's node indices and the solver's internal indices.
    # It takes the total number of nodes, number of vehicles, and lists of start and end nodes for each vehicle.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, starts, ends)

    # Create Routing Model, which is the main data structure for solving VRP.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for distances.
    # This callback returns the distance between two nodes.
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Add Distance dimension to the routing model.
    # This dimension keeps track of the total distance traveled by each vehicle.
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack, meaning no extra distance allowed beyond the actual route
        distance_limit,  # maximum travel distance per vehicle
        True,  # start cumul to zero, initial distance is zero at the start
        distance_dimension_name)

    # Retrieve the distance dimension to set additional parameters.
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)

    # Set global span cost coefficient to minimize the longest route among all vehicles.
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Create and register a demand callback for vehicle demands.
    # This callback returns the demand at a node.
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    # Add Capacity dimension to enforce vehicle capacity constraints.
    # It tracks the load of each vehicle along its route.
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # list of maximum capacities for each vehicle
        True,  # start cumul to zero
        'Capacity')

    # Set search parameters for the solver.
    # Using PATH_CHEAPEST_ARC as the first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        pywrapcp.RoutingModel.DefaultStrategy.PATH_CHEAPEST_ARC)

    # Solve the VRP with the specified parameters.
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, compute the maximum route distance among all vehicles.
    if solution:
        max_route_distance = 0
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            route_distance = 0
            # Traverse the route for each vehicle until the end node.
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                # Add the distance of the current arc to the route distance.
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            # Track the maximum route distance.
            max_route_distance = max(max_route_distance, route_distance)
        return max_route_distance
    else:
        # Return -1 if no solution is found.
        return -1