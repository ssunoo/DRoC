# Vehicle Routing Problem with Service Time and Duration Limit (VRPSL)
# Import necessary modules from OR-Tools.
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    # Create the routing index manager, which manages the conversion between the node indices and the internal routing indices.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create the routing model which will contain the routing problem.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes.
    def time_callback(from_index, to_index):
        # Convert routing indices to node indices.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return the travel time between the nodes.
        return time_matrix[from_node][to_node]

    # Register the transit callback and get its index.
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator for all vehicles to the transit callback.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a time dimension to model the cumulative travel time.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack (waiting time allowed at nodes)
        duration_limit,  # maximum route duration for each vehicle
        True,  # force start cumul variables to be zero at the start of each route
        time)
    # Retrieve the time dimension for further constraints.
    time_dimension = routing.GetDimensionOrDie(time)

    # Add service time at each node as a fixed transit.
    for node in range(len(service_time)):
        index = manager.NodeToIndex(node)
        # Set the service time as a fixed cumulative transit at the node.
        time_dimension.CumulVar(index).SetValue(service_time[node])

    # Initialize start cumul variables for each vehicle to zero and add them to the finalizer.
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        # Set the start time for each vehicle to zero.
        time_dimension.CumulVar(index).SetValue(0)
        # Minimize the start time variable in the solution.
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(index))
        # Maximize the start time variable in the solution.
        routing.AddVariableMaximizedByFinalizer(time_dimension.CumulVar(index))

    # Optional: Remove or add other constraints as needed.
    # For example, to remove a 'Duration' dimension if it exists, or to add capacity constraints.

    # Set the search parameters, including the first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Optional: set a time limit for the search to prevent long runtimes.
    # For example, uncomment the following line to set a 30-second time limit.
    # search_parameters.time_limit.seconds = 30

    # Solve the routing problem with the specified parameters.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1