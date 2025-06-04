
# Vehicle Routing with Pickups and Deliveries (PDP)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)
# Create the routing index manager.
#   - len(distance_matrix): Number of nodes in the problem.
#   - num_vehicle: Number of vehicles available.
#   - depot: Index of the depot node.
routing = pywrapcp.RoutingModel(manager)
# Create the routing model.

def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Convert from routing model internal indices to distance matrix indices.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return distance_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
# Define a callback function that returns the distance between two nodes.
# Register the transit callback.

# Define cost of each arc.
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
# Set the cost of each arc using the transit callback.

# Add Distance Dimension
dimension_name = "Distance"
# A sufficiently large number for the maximum travel distance for a single vehicle.
# This should be large enough not to constrain the problem unless there's an explicit
# maximum distance per vehicle.
# Using a large constant instead of complex calculation that might lead to
# an excessively large domain for cumulative variables, causing performance issues.
max_route_distance = 999999999999

routing.AddDimension(
    transit_callback_index,
    0,  # null slack (no waiting time for distance)
    max_route_distance,  # vehicle maximum travel distance
    True,  # start cumul to zero
    dimension_name,
)
# Add a dimension to track the distance traveled by each vehicle.
#   - transit_callback_index: The index of the transit callback.
#   - 0: The null slack, which is the amount of time a vehicle can wait at a node.
#   - max_route_distance: The maximum distance a vehicle can travel.
#   - True: Whether the cumulative variable starts at zero.
#   - dimension_name: The name of the dimension.
distance_dimension = routing.GetDimensionOrDie(dimension_name)

# Add pickup and delivery requests.
for request in pickups_deliveries:
    pickup_index = manager.NodeToIndex(request[0])
    delivery_index = manager.NodeToIndex(request[1])
    routing.AddPickupAndDelivery(pickup_index, delivery_index)
    # Ensure pickup and delivery are handled by the same vehicle.
    routing.solver().Add(
        routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
    )
    # Ensure pickup occurs before delivery.
    routing.solver().Add(
        distance_dimension.CumulVar(pickup_index) <= distance_dimension.CumulVar(delivery_index)
    )
# Iterate through the pickup and delivery requests.
#   - request[0]: The index of the pickup node.
#   - request[1]: The index of the delivery node.
#   - routing.AddPickupAndDelivery(): Adds a pickup and delivery constraint.
#   - routing.solver().Add(): Adds a constraint to the solver.

# Set search parameters.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)
# Set the first solution strategy to PATH_CHEAPEST_ARC.
search_parameters.local_search_metaheuristic = (
    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
)
# Set the local search metaheuristic to GUIDED_LOCAL_SEARCH.
search_parameters.time_limit.FromSeconds(30) # Set a reasonable time limit
# Set a time limit for the search.

# Solve the problem.
solution = routing.SolveWithParameters(search_parameters)
# Solve the problem using the search parameters.

obj = -1
if solution:
    obj = solution.ObjectiveValue()
# Get the objective value of the solution.
return obj
# Return the objective value.
