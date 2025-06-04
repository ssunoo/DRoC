
# Capacitated Vehicle Routing Problem with Time Windows and Duration Limit (CVRPTWL)
# [('user', 'The solution failed the code execution test: Error: Wrong number or type of arguments for overloaded function \'new_RoutingIndexManager\'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpsrros5br.py", line 112, in <module>\n    result = solve(time_matrix, time_windows, demands, vehicle_capacities, num_vehicle, depot, duration_limit)\n  File "/tmp/tmpsrros5br.py", line 7, in solve\n    manager = pywrapcp.RoutingIndexManager(\n  File "/disks/local/r.2t_6/r13922049/miniconda3/envs/AI/lib/python3.10/site-packages/ortools/constraint_solver/pywrapcp.py", line 4749, in __init__\n    _pywrapcp.RoutingIndexManager_swiginit(self, _pywrapcp.new_RoutingIndexManager(*args))\nTypeError: Wrong number or type of arguments for overloaded function \'new_RoutingIndexManager\'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\n\n')]
import sys
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix, num_vehicle, depot, demands, vehicle_capacities, time_windows, duration_limit):
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicle, depot
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for time (cost and time dimension).
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Add Time Windows and Duration Limit constraint.
    time_dimension_name = "Time"
    routing.AddDimension(
        transit_callback_index,
        sys.maxsize,  # allow waiting time (slack_max) - set to a very large number
        duration_limit,  # maximum time per vehicle (capacity_max of the dimension)
        False,  # Don't force start cumul to zero.
        time_dimension_name,
    )
    time_dimension = routing.GetDimensionOrDie(time_dimension_name)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot).
    # This ensures the depot's time window is applied to the start of each vehicle's route.
    depot_idx = depot
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            time_windows[depot_idx][0], time_windows[depot_idx][1]
        )

    # Instantiate route start and end times to produce feasible times.
    # This is crucial for the solver to find a solution that respects time windows.
    for i in range(num_vehicle):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i))
        )
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i))
        )

    # Setting first solution heuristic and search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Add a time limit to prevent indefinite running.
    search_parameters.time_limit.FromSeconds(60) # Set a reasonable time limit, e.g., 60 seconds.
    # Add a local search metaheuristic for better solution quality and faster convergence.
    search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    obj = -1
    if solution:
        obj = solution.ObjectiveValue()
    return obj
