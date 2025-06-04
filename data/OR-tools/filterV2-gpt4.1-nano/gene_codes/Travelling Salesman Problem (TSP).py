# Travelling Salesman Problem (TSP)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, depot: int):
    # Create the data model
    data = {
        'distance_matrix': distance_matrix,
        'num_vehicles': 1,
        'depot': depot
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    # Register the callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc using the registered callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic to PATH_CHEAPEST_ARC for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the TSP problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value (total distance) if a solution is found
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1