classdef GuSTOSolution
    properties
    traj
	status
	accepted
    success
	convergence
    converged
	iterations
    problem
    end
    methods
        function obj = GuSTOSolution(Problem,model,method,initTime)
          obj.traj=InitialTrajectory(Problem,model,method,initTime);
          obj.status=[];
          obj.accepted=[true];
          obj.convergence=[0.];
          obj.success=false;
          obj.converged=false;
          obj.iterations=0;
          obj.problem=Problem;
        end
    end
end

