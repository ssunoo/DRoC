# Capacitated Vehicle Routing Problem with Distance Limit (CVRPL)
# This code solves a CVRPL using Google OR-Tools
# It considers vehicle capacities, demands, and a maximum distance limit per vehicle
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacities: list, depot: int, distance_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing model indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for distances between nodes
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)      # Convert from routing index to node index
        return distance_matrix[from_node][to_node]  # Return distance between nodes

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)  # Register callback

    # Set the arc cost evaluator for all vehicles to the distance callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension to track total distance per vehicle
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack (waiting time)
        distance_limit,  # Maximum distance per vehicle
        True,  # Start cumul to zero
        distance_dimension_name)  # Dimension name

    # Retrieve the distance dimension to set additional parameters
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)

    # Set a cost coefficient to minimize the maximum route distance (span)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Create and register a demand callback for vehicle demands at each node
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)  # Convert index
        return demands[from_node]  # Return demand at node

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)  # Register demand callback

    # Add Capacity dimension to ensure vehicle capacities are not exceeded
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,  # Demand callback index
        0,  # No slack
        vehicle_capacities,  # List of vehicle capacities
        True,  # Start cumul to zero
        'Capacity')  # Dimension name

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC heuristic for initial solution
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Use GUIDED_LOCAL_SEARCH for local search improvements
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # Set a time limit of 300 seconds (5 minutes) for the solver
    search_parameters.time_limit.FromSeconds(300)

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, compute the total distance traveled by all vehicles
    if solution:
        total_distance = 0
        for vehicle_id in range(num_vehicle):  # Iterate over each vehicle
            index = routing.Start(vehicle_id)  # Start index for vehicle
            route_distance = 0  # Initialize route distance
            # Traverse the route until the end node
            while not routing.IsEnd(index):
                previous_index = index  # Store current index
                index = solution.Value(routing.NextVar(index))  # Move to next node
                # Add distance for the arc between previous and current node
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            total_distance += route_distance  # Accumulate total distance
        return total_distance  # Return total distance of all routes
    else:
        return -1  # Return -1 if no solution is found
