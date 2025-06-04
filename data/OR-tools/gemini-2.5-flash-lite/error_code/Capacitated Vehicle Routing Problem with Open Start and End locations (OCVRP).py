
# Capacitated Vehicle Routing Problem with Open Start and End locations (OCVRP)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    # Data model
    data = {}
    data['distance_matrix'] = distance_matrix
    data['demands'] = demands
    data['num_vehicles'] = num_vehicle
    data['vehicle_capacities'] = vehicle_capacity
    data['depot'] = 0  # Assuming node 0 is the depot

    # Create the routing index manager.
    # num_nodes is len(distance_matrix) because distance_matrix includes the depot.
    manager = pywrapcp.RoutingIndexManager(
        len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for distances.
    # For OCVRP, the cost of returning to the depot from any customer is 0.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # If the destination is the depot, and the origin is a customer,
        # the cost of this arc is 0 for open VRP.
        if to_node == data['depot'] and from_node != data['depot']:
            return 0
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle capacities
        True,  # start cumul to zero
        'Capacity')

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value.
    if solution:
        obj = solution.ObjectiveValue()

    return obj
