# Vehicle Routing Problem with Service Time (VRPS)
# [('user', 'The solution failed the code execution test: Error: Wrong number or type of arguments for overloaded function \'Solver_FixedDurationIntervalVar\'.\n  Possible C/C++ prototypes are:\n    operations_research::Solver::MakeFixedDurationIntervalVar(int64_t,int64_t,int64_t,bool,std::string const &)\n    operations_research::Solver::MakeFixedDurationIntervalVar(operations_research::IntVar *,int64_t,std::string const &)\n    operations_research::Solver::MakeFixedDurationIntervalVar(operations_research::IntVar *,int64_t,operations_research::IntVar *,std::string const &)\n\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpq4b2um0h.py", line 89, in <module>\n    result = solve(time_matrix, num_vehicle, depot, service_time)\n  File "/tmp/tmpq4b2um0h.py", line 59, in solve\n    service_interval = routing.solver().FixedDurationIntervalVar(\n  File "/disks/local/r.2t_6/r13922049/miniconda3/envs/AI/lib/python3.10/site-packages/ortools/constraint_solver/pywrapcp.py", line 1298, in FixedDurationIntervalVar\n    return _pywrapcp.Solver_FixedDurationIntervalVar(self, *args)\nTypeError: Wrong number or type of arguments for overloaded function \'Solver_FixedDurationIntervalVar\'.\n  Possible C/C++ prototypes are:\n    operations_research::Solver::MakeFixedDurationIntervalVar(int64_t,int64_t,int64_t,bool,std::string const &)\n    operations_research::Solver::MakeFixedDurationIntervalVar(operations_research::IntVar *,int64_t,std::string const &)\n    operations_research::Solver::MakeFixedDurationIntervalVar(operations_research::IntVar *,int64_t,operations_research::IntVar *,std::string const &)\n\n\n')]}

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
    # Create the data model
    data = {
        'distance_matrix': time_matrix,
        'num_vehicles': num_vehicle,
        'depot': depot,
        'service_time': service_time
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        10,  # slack to allow waiting
        10000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Set start time for each vehicle
    for v in range(data['num_vehicles']):
        index = routing.Start(v)
        time_dimension.CumulVar(index).SetValue(0)

    # Add Service Time constraints
    for node in range(len(data['service_time'])):
        if node != data['depot']:
            index = manager.NodeToIndex(node)
            # Create an interval variable for the service time
            # Corrected the function call to match the expected arguments
            service_interval = routing.solver().MakeFixedDurationIntervalVar(
                index, data['service_time'][node], f'Service_{node}')
            # Link service interval with the node visit
            routing.solver().Add(routing.solver().IntervalVarCovers(service_interval, index))

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    obj = -1
    if solution:
        obj = solution.ObjectiveValue()
    return obj