# Vehicle Routing Problem with Multiple Depots (VRPMD)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, num_vehicle: int, starts: list, ends: list):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        num_vehicle: the number of the vehicle
        starts: the index of the starting depot for vehicles
        ends: the index of the ending depot for vehicles 

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the data model
    data = {
        'distance_matrix': distance_matrix,
        'num_vehicles': num_vehicle,
        'starts': starts,
        'ends': ends
    }

    # Create the routing index manager
    # Manages the conversion between the problem's node indices and the solver's internal indices
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model
    # Represents the routing problem
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback
    # Defines how to compute the travel cost between nodes
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc
    # Assigns the callback to evaluate the cost of traveling between nodes for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint
    # Adds a dimension to track the total distance traveled
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        30000,  # vehicle maximum travel distance (increased from 20000)
        True,  # start cumul to zero
        distance_dimension_name)
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)
    # Set a global span cost coefficient to penalize longer routes
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic
    # Defines the strategy for initial solution search
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    # Finds a solution based on the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    # If a solution is found, return its objective value; otherwise, return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1