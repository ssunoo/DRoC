# Vehicle Routing Problem with Duration Limit (VRPL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(time_matrix: list, num_vehicle: int, depot: int, duration_limit: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        num_vehicle: the number of the vehicle
        depot: the index of the depot node
        duration_limit: the time duration of each route is upper bounded by the duration limit 

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager, which manages the indexing between the nodes and the solver
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model, which is the main object to define the VRP
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Returns the travel time between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc as the travel time, used for routing optimization
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension with duration limit to model the time constraints
    time_dimension = routing.AddDimension(
        transit_callback_index,
        0,  # allow waiting time
        duration_limit,  # maximum time per vehicle
        True,  # force start cumul to zero
        "Time")

    # Add constraints to ensure that the cumulative time at the start and end of each route does not exceed the duration limit
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        end_index = routing.End(vehicle_id)
        routing.solver().Add(
            routing.GetDimensionOrDie("Time").CumulVar(index) <= duration_limit)
        routing.solver().Add(
            routing.GetDimensionOrDie("Time").CumulVar(end_index) <= duration_limit)

    # Set search parameters for the solver, including time limit and strategies
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 30
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.log_search = True

    # Solve the VRP with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, compute the total travel time of all routes
    if solution:
        total_time = 0
        time_dimension_obj = routing.GetDimensionOrDie("Time")
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            end_index = routing.End(vehicle_id)
            # Sum the cumul variables at the end nodes for each route to get total time
            total_time += solution.Value(time_dimension_obj.CumulVar(end_index))
        return total_time
    else:
        # Return -1 if no solution is found
        return -1