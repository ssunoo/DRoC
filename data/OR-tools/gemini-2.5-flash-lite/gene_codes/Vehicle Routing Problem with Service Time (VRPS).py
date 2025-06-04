# Vehicle Routing Problem with Service Time (VRPS)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, num_vehicle: int, depot: int, service_time: list):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        num_vehicle: the number of the vehicle
        depot: the index of the depot node
        service_time: service time for each customer node

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1

    # Create the routing index manager.
    # The number of nodes is len(time_matrix).
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model.
    model = pywrapcp.RoutingModel(manager)

    # Create a transit callback.
    # This callback returns the total time to travel from 'from_node' to 'to_node',
    # including the service time at the 'from_node'.
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # The service time at 'from_node' is incurred before traveling to 'to_node'.
        return time_matrix[from_node][to_node] + service_time[from_node]

    transit_callback_index = model.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    # The objective is to minimize the total cost, which includes travel time and service time.
    model.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension to the model.
    # This dimension tracks the cumulative time for each vehicle.
    # A sufficiently large value for max_time_per_vehicle is used as no specific upper bound is given.
    # Slack is set to 0, meaning no waiting time is allowed at customer nodes.
    max_time_per_vehicle = 1000000 # A large enough value for vehicle maximum travel time
    time_dimension_name = 'Time'
    model.AddDimension(
        transit_callback_index,
        0,  # no slack (waiting time) at nodes
        max_time_per_vehicle,  # vehicle maximum travel time
        True,  # start cumul to zero for all vehicles at the depot
        time_dimension_name)
    time_dimension = model.GetDimensionOrDie(time_dimension_name)

    # Setting first solution heuristic and search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(30) # Set a time limit for the solver

    # Solve the problem.
    solution = model.SolveWithParameters(search_parameters)

    # Get the objective value from the solution.
    # The objective value represents the total travel time plus total service time
    # across all routes.
    if solution:
        obj = solution.ObjectiveValue()
    else:
        obj = -1 # Indicate that no solution was found

    return obj
