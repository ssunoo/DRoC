
# Vehicle Routing Problem with Open Start and End locations (OVRP)
# [('user', 'The solution failed the code execution test: Error: name 'solve' is not defined\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpoqiigmhx.py", line 92, in <module>\n    result = solve(distance_matrix, num_vehicle)\nNameError: name 'solve' is not defined\n\n')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve_ovrp(distance_matrix, num_vehicle):
    num_locations = len(distance_matrix)

    # For an Open Vehicle Routing Problem (OVRP) where specific start/end
    # locations are not provided as input, we need to define them.
    # Following the pattern from the context for 'Open Start and End',
    # which uses explicit 'starts' and 'ends' arrays, we will assume:
    # - All vehicles start at node 0 (a common depot/starting point).
    # - For 'open end', vehicles do not need to return to a specific depot
    #   and the cost of the last leg is not counted. This is typically handled
    #   by setting the cost of the arc leading to the end node to zero.
    #   We will set all vehicles to end at node 0, and then explicitly set the
    #   cost of the arc leading to this end node to 0 for each vehicle.

    starts = [0] * num_vehicle  # All vehicles start at node 0
    ends = [0] * num_vehicle    # All vehicles end at node 0 (virtual depot for open end)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        num_locations, num_vehicle, starts, ends
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Set the cost of the arc leading to the end node to 0 for each vehicle
    # to model "Open End".
    for vehicle_id in range(num_vehicle):
        routing.SetEndArcCostForVehicle(vehicle_id, 0)

    # Add Distance dimension (optional, but good practice for VRPs).
    # The context example includes a distance dimension and sets a global span cost coefficient.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        1000000000,  # vehicle maximum travel distance (a sufficiently large constant)
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    # Set a cost coefficient for the global span of the distance dimension.
    # This penalizes routes with large total distances.
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        return solution.ObjectiveValue()
    else:
        # If no solution is found, return -1 as an indicator.
        return -1
