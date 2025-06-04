# Capacitated Vehicle Routing Problem with Open Start and End locations (OCVRP)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacities: list):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        demands: the list of integer customer demands
        num_vehicle: the number of the vehicle
        vehicle_capacities: the capacity of each vehicle

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the data model
    data = {}
    data["distance_matrix"] = distance_matrix  # Matrix of distances between locations
    data["demands"] = demands  # Customer demands
    data["num_vehicles"] = num_vehicle  # Number of vehicles
    data["vehicle_capacities"] = vehicle_capacities  # Capacities of each vehicle
    # Set start and end locations to 0 (depot) for all vehicles
    data["starts"] = [0] * num_vehicle  # All vehicles start at depot (index 0)
    data["ends"] = [0] * num_vehicle  # All vehicles end at depot (index 0)

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data["distance_matrix"]), data["num_vehicles"], data["starts"], data["ends"])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for distances
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing variable to node index
        to_node = manager.IndexToNode(to_index)  # Convert routing variable to node index
        return data["distance_matrix"][from_node][to_node]  # Return distance between nodes

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)  # Register callback

    # Set the cost of each arc to the distance callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension to measure total distance traveled
    distance_dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # no slack (waiting time)
        20000,  # maximum distance per vehicle
        True,  # start cumul to zero
        distance_dimension_name)  # Dimension name
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)  # Get the dimension
    distance_dimension.SetGlobalSpanCostCoefficient(100)  # Set cost coefficient for the total span

    # Add capacity constraint to ensure vehicle demands are within capacity
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing variable to node index
        return data["demands"][from_node]  # Return demand at node

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)  # Register demand callback
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,  # Demand callback index
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity")  # Dimension name

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution

    # Solve the problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()  # Objective value of the solution
    else:
        return -1  # No solution found