
# Capacitated Vehicle Routing Problem with Time Windows and Multiple Depots (CVRPTWMD)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicles: int, starts: list, ends: list):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        demands: the list of integer customer demands
        vehicle_capacities: the capacity of each vehicle
        num_vehicles: the number of the vehicle
        starts: the index of the starting depot for vehicles
        ends: the index of the ending depot for vehicles

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1

    # Create the routing index manager.
    # The number of nodes is the length of the time_matrix (assuming it's square).
    num_nodes = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(num_nodes, num_vehicles, starts, ends)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for time.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time = "Time"
    # A sufficiently large number for maximum time per vehicle and allowed waiting time (slack).
    # This ensures that the time dimension does not prematurely constrain the problem
    # beyond what the time windows themselves impose.
    big_number = 100000

    routing.AddDimension(
        transit_callback_index,
        big_number,  # allow waiting time (slack_max)
        big_number,  # maximum time per vehicle (capacity)
        False,  # Don't force start cumul to zero. (Time at start node can be non-zero)
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Instantiate route start and end times to produce feasible times.
    # This helps the solver find a solution by minimizing the time at start/end nodes.
    for vehicle_id in range(num_vehicles):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(vehicle_id)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(vehicle_id)))

    # Add Capacitated constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    capacity = "Capacity"
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack (no waiting for capacity, i.e., demand is picked up instantly)
        vehicle_capacities,  # vehicle capacities for each vehicle
        True,  # start cumul to zero (vehicle starts empty)
        capacity,
    )

    # Setting first solution heuristic and local search metaheuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    # Add a time limit for solving, useful for larger problems.
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        obj = solution.ObjectiveValue()
    return obj
