# Capacitated Vehicle Routing Problem (CVRP)
# This code solves the CVRP using Google OR-Tools
# It finds the optimal routes for a fleet of vehicles to deliver demands to various locations

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacity: list, depot: int):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Convert routing indices to node indices
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return the distance between the nodes
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc (edge) to the distance between nodes
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint to ensure vehicle capacities are not exceeded
    def demand_callback(from_index):
        # Convert routing index to node index
        from_node = manager.IndexToNode(from_index)
        # Return the demand at the node
        return demands[from_node]

    demand_callback_index = routing.RegisterTransitCallback(demand_callback)
    # Add capacity dimension with vehicle capacities
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,  # demand callback
        0,  # null capacity slack
        vehicle_capacity,  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')  # dimension name

    # Set the search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use the PATH_CHEAPEST_ARC heuristic for the initial solution
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the CVRP problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Check if a feasible solution was found
    if solution:
        # Return the total cost of the solution
        return routing.CostVar().Value(solution)
    else:
        # Return -1 if no feasible solution exists
        return -1