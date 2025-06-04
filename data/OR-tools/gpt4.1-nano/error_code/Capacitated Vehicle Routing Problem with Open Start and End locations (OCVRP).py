# Capacitated Vehicle Routing Problem with Open Start and End locations (OCVRP)
# [('user', 'You solution returns nothing or 0. The program may be incomplete or your solution does not work for the task.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacity: list):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        demands: the list of integer customer demands
        num_vehicle: the number of the vehicle
        vehicle_capacities: the capacity of each vehicle

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the data model
    data = {}
    data["distance_matrix"] = distance_matrix
    data["demands"] = demands
    data["num_vehicles"] = num_vehicle
    data["vehicle_capacities"] = vehicle_capacity
    # For open VRP, set starts and ends to None or omit them
    # Here, we assume all vehicles start from node 0
    data["starts"] = [0] * data["num_vehicles"]
    data["ends"] = [0] * data["num_vehicles"]

    # Create the routing index manager with only number of nodes, number of vehicles, and depot index
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, 0)

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for distances
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of travel for each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add capacity constraint
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacity,
        True,  # start cumul to zero
        "Capacity"
    )

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Compute total distance (objective value)
    obj = 0
    if solution:
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                obj += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
    return obj