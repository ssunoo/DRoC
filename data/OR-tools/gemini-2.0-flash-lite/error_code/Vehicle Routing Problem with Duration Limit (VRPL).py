
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve():
    # Define the problem data.
    # time_matrix represents the travel time between each pair of locations.
    time_matrix = [
        [0, 10, 15, 20],
        [10, 0, 35, 25],
        [15, 35, 0, 30],
        [20, 25, 30, 0],
    ]
    # num_vehicle represents the number of vehicles available.
    num_vehicle = 1
    # depot represents the starting and ending location for all vehicles.
    depot = 0
    # duration_limit represents the maximum allowed travel time for each vehicle.
    duration_limit = 100

    # Create the routing index manager.
    # This manages the indices used in the routing model.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicle, depot
    )

    # Create the routing model.
    # This is the main object that solves the vehicle routing problem.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    # This callback calculates the travel time between two nodes.
    def time_callback(from_index, to_index):
        """Returns the travel time between two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    # This sets the cost (in this case, travel time) for each arc in the graph.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time constraint.
    # This adds a constraint to limit the total travel time for each vehicle.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,  # transit callback index
        3600,  # allow waiting time
        duration_limit,  # vehicle maximum travel time
        False,  # start cumul to zero
        time,  # dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)
    time_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    # This sets the strategy for finding the first solution.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy =
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem.
    # This solves the vehicle routing problem using the defined parameters.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    # This prints the solution to the console.
    if solution:
        obj = solution.ObjectiveValue()
        return obj
    else:
        return None
