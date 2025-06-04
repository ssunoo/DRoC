# Capacitated Vehicle Routing Problem with Distance Limit and Open Start and End locations (OCVRPL)
# [('user', 'The solution failed the code execution test: Error: \'bool\' object has no attribute \'SetGlobalSpanCostCoefficient\'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmp3ivmiu1c.py", line 80, in <module>\n    result = solve(distance_matrix, demands, num_vehicle, vehicle_capacities, distance_limit)\n  File "/tmp/tmp3ivmiu1c.py", line 40, in solve\n    distance_dimension.SetGlobalSpanCostCoefficient(100)\nAttributeError: \'bool\' object has no attribute \'SetGlobalSpanCostCoefficient\'
\n')}

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacities: list, distance_limit: int):
    # Create the data model
    data = {
        'distance_matrix': distance_matrix,
        'demands': demands,
        'num_vehicles': num_vehicle,
        'vehicle_capacities': vehicle_capacities,
        'starts': list(range(num_vehicle)),  # Assuming open start locations, can be customized
        'ends': list(range(num_vehicle)),    # Assuming open end locations, can be customized
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Register transit callback for distance
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension with distance limit
    distance_dimension = routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        distance_limit,  # maximum travel distance per vehicle
        True,  # start cumul to zero
        'Distance')
    # Check if distance_dimension is valid before calling method
    if distance_dimension:
        distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add Capacity constraint
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return solution.ObjectiveValue()
    return -1
