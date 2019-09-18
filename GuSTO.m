%% Guaranteed Sequential Trajectory Optimization (GuSTO)
% ECE 602 Winter 2019 Project
% Nurken Tuktibayev (20764609)
clear all
clc
N=41;
init=[0;0;0];
goal=[3;3;3];
maxIterations=50;
initTimeGuess=5;
initTrajMethod=0;
model=DubinsCar();
world=GenerateObstacles(init,goal,5,0.5);
save('TheWorld.mat','world');
%load('TheWorld.mat','world'); 
Problem=GuSTOProblem(init,goal,N,world);
Solution=GuSTOSolution(Problem,model,initTrajMethod,initTimeGuess);
delta0=10000;
delta=delta0;
omega0=1;
omegaM=1.0e10;
omega=omega0;
rho0=0.4;
rho1=0.9;
rhok=0;
betaS=2;
betaF=0.5;
gammaF=5;
epsilon=1.0e-6;
convergence_threshold=0.1;
[model.f, model.A, model.B]=InitializeModelParams(model,Problem, Solution.traj);
Problem.obstacleTolerance=delta*0.8+model.clearance;
ind=0;
figure()
while ind<maxIterations
        iterations_left=maxIterations-ind
		[model.f model.A]=UpdateModelParams(model,Problem, Solution.traj);
        cvx_begin
        variables Tff X(model.xDim,Problem.N)
        variable U(model.uDim,Problem.N-1)
        minimize (ObjectiveFunction(X,U,Solution.traj,Problem,model,omega,delta))
        subject to
             for i=1:model.xDim
                 X(i,1)-Problem.init(i)==0;%state init constraints
                 X(i,Problem.N)-Problem.goal(i)==0;%state goal constraints
             end
             for k=1:N-1
                DynamicConstraints(model,Problem,X,U,Tff,Solution.traj,k,0)==0
                U(1,k)-model.uMax(1)<=0 %control max bound constraints
                model.uMin(1)-U(1,k)<=0 %control min bound constraints
             end
             Tff>=0.1
		cvx_end
        Solution.status=cvx_status;
		if strcmp(cvx_status,'Infeasible') || strcmp(cvx_status,'Failed')
            warning("GuSTO SCP iteration failed to find an optimal solution");
            break
        end
        newTraj.X=X;
        newTraj.U=U;
        newTraj.Tff=Tff;
        newTraj.dt=Tff/(size(X,2)-1);
        convergence=ConvergenceMetric(Problem,newTraj,Solution.traj);
        maxv=-Inf;
        for k=1:Problem.N
            maxv=max(norm(newTraj.X(:,k)-Solution.traj.X(:,k))^2,maxv);
        end
        if maxv<=delta
            rhok=TrustRegionRatio(Problem,model,newTraj,Solution.traj);
			if rhok>rho1
                Solution.accepted=false;
                delta=betaF*delta;
            else 
				Solution.accepted=true;
                if rhok<rho0 
                   delta=min(betaS*delta,delta0);
                end
 				if ConvexIneqSatisfied(Problem,model,newTraj,Solution.traj,epsilon)==false
                    omega=gammaF*omega;
                else
                    omega=omega0;
                end
            end
        else
 			Solution.accepted=false;
 			omega=gammaF*omega;
        end
 		if Solution.accepted
			Solution.traj=newTraj;
            plot(Solution.traj.X(1,:),Solution.traj.X(2,:),'--g','HandleVisibility','off')
        hold on    
        else
            plot(newTraj.X(1,:),newTraj.X(2,:),'--r','HandleVisibility','off')
        end
        Problem.obstacleTolerance=delta/8+model.clearance;
		ind=ind+1;
        Solution.iterations=ind;
		if omega>omegaM
			warning("Omega_max exceeded")
			break
		end
		if Solution.accepted==false
           continue
        end
		if convergence<=convergence_threshold
            if ConvexIneqSatisfied(Problem,model,newTraj,Solution.traj,epsilon)
                Solution.success=true;
                if Solution.success %algorithm converged
                   disp('GuSTO converged')
                   break 
                end
            end
		end
    end	
Output(Solution,model,init,goal,world)