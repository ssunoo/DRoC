# Capacitated Vehicle Routing Problem with Multiple Depots (CVRPMD)
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacities: list, starts: list, ends: list):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, starts, ends)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Returns the distance between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    # Register the distance callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc (edge) for all vehicles using the registered callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint to ensure vehicle loads do not exceed their capacities
    def demand_callback(from_index):
        # Returns the demand of the node
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    # Register demand callback
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    # Add capacity dimension with vehicle capacities
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic to guide the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Initialize total distance
    total_distance = 0
    if solution:
        # Loop through each vehicle to calculate total distance traveled
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                # Accumulate distance for each arc traveled by the vehicle
                total_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
    else:
        # If no solution is found, set total distance to None
        total_distance = None

    # Return the total distance traveled by all vehicles
    return total_distance