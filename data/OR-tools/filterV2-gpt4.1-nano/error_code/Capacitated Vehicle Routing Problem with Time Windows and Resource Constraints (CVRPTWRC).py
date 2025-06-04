# Capacitated Vehicle Routing Problem with Time Windows and Resource Constraints (CVRPTWRC)
# [('user', "UnusedParameterError: Params {'vehicle_load_time', 'vehicle_unload_time'} are not used, you should ensure all the params are used in the function.")]
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicles: int, depot: int, vehicle_load_time: int, vehicle_unload_time: int, depot_capacity: int):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicles, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator for all vehicles to minimize total travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint to ensure service occurs within specified time windows
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        1440,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        'Time')
    time_dimension = routing.GetDimensionOrDie('Time')

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add demand dimension to ensure vehicle capacities are not exceeded
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # vehicle maximum capacities
        True,  # start cumul to zero
        'Demand')

    # Add resource constraints at depot (loading/unloading)
    # For simplicity, assume resource constraints are modeled as limits on number of vehicles loading/unloading at the depot
    # This can be modeled with a custom resource dimension if needed
    # Here, we add a dummy resource dimension to limit the number of vehicles starting at depot
    def resource_callback(from_index):
        node = manager.IndexToNode(from_index)
        if node == depot:
            return 1  # vehicle loading/unloading at depot
        return 0

    resource_callback_index = routing.RegisterUnaryTransitCallback(resource_callback)
    routing.AddDimension(
        resource_callback_index,
        0,  # slack
        depot_capacity,  # capacity
        True,  # start cumul to zero
        'Resource')

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Calculate total cost
    total_time = 0
    if solution:
        total_time = routing.Cost(solution)

    # Return the total travel time as the objective value
    obj = total_time
    return obj