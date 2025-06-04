# Simple Vehicle Routing Problem (VRP)
# [('user', 'The solution failed the code execution test: Error: type object 'FirstSolutionStrategy' has no attribute 'PATH_CHEAPEST_ARC'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpgkr1513_.py", line 57, in <module>\n    result = solve(distance_matrix, num_vehicle, depot)\n  File "/tmp/tmpgkr1513_.py", line 27, in solve\n    pywrapcp.FirstSolutionStrategy.PATH_CHEAPEST_ARC)\nAttributeError: type object 'FirstSolutionStrategy' has no attribute 'PATH_CHEAPEST_ARC'\n')]}
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, num_vehicle: int, depot: int):
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        # Returns the distance between the two nodes.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    # (Optional: can be added if needed, but for simple VRP, setting arc cost suffices)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        pywrapcp.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Calculate total distance of the solution.
    total_distance = 0
    if solution:
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                total_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
    else:
        total_distance = -1

    return total_distance