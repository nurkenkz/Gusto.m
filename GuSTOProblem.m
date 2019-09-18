classdef GuSTOProblem
    properties
    init
    goal
	N 				
	dh
    obstacles
    obstacleTolerance
    end
    methods
        function obj = GuSTOProblem(init,goal,N,world)
            obj.init=init;
            obj.goal=goal;
            obj.N=N;
            obj.dh=1/(N-1);
            obj.obstacles=world;
        end
        function o=InitialTrajectory(Problem,model,method,initTime)
             %creates initial trajectory base in method:
             %(method=1 straightline, method=0 middle point
             init=Problem.init;
             goal=Problem.goal;
             xDim=model.xDim;
             uDim=model.uDim;
             N=Problem.N;
             X=[];
             if method==1
               X=zeros(xDim,N);
               for j=1:xDim
                  X(j,:)=linspace(init(j,:), goal(j,:), N);
               end
               U=zeros(uDim,N);
             else
               X=repmat(0.5*(init+goal),1,N);
               U=zeros(uDim,N);
             end
             o.X=X;
             o.U=U;
             o.Tff=initTime;
             o.dt=initTime/(N-1);
        end
        function o=ConvergenceMetric(Problem,traj,trajPrev)
             maxNk=-Inf;
             maxDk=-Inf;
             for k=1:Problem.N
               maxNk=max(norm(traj.X(:,k)-trajPrev.X(:,k)),maxNk);
               maxDk=max(norm(traj.X(:,k)),maxDk);
               o= maxNk/maxDk;%normalized maximum relative error
             end
        end
        function o=TrustRegionRatio(Problem,model,traj,trajPrev) %(5)
             X=traj.X;
             U=traj.U;
             Xp=trajPrev.X;
             Up=trajPrev.U;
             N=Problem.N;
             fp=model.f;
             Ap=model.A;
             Bp=model.B;
             Nk=0;
             Dk=0;
             for k=1:N-1
                linearized = fp(:,k)+Ap(:,:,k)*(X(:,k)-Xp(:,k))+Bp*(U(:,k)-Up(:,k));
                Nk=Nk+norm(fDynamic(model,X(:,k),U(:,k))-linearized);
                Dk=Dk+norm(linearized);
             end  
             for k=1:N
                  r=traj.X(1:2,k);%new trajectory point
                  rp=trajPrev.X(1:2,k);%current trajectory point
                  for i=1:size(Problem.obstacles,2) %from Eq 12a-12c in [16]
                     obsC=Problem.obstacles(1:2,i);%obstraction center
                     obsR=Problem.obstacles(3,i);%obstraction radius
                     p_chaser=rp;%trajectory collision point
                     p_ko=obsC+obsR*(p_chaser-obsC)/norm(p_chaser-obsC);%obstraction collision point 
                     di=norm(p_chaser-obsC)-obsR;%signed distance between trajectory and obstacle
                     if di>0
                        dhat=(p_chaser-p_ko)./norm(p_chaser-p_ko);
                     else
                        dhat=(p_ko-p_chaser)./norm(p_ko-p_chaser);
                     end
                     distLinearized=model.clearance-(di+dhat'*(r-rp));
                     distTrue=model.clearance-(norm(r-obsC)-obsR);
                     Nk=Nk+abs(distTrue-distLinearized);
                     Dk=Dk+abs(distLinearized);
                  end
             end
             if Dk~=0  
                 o=Nk/Dk;
             else 
                 o=0;
             end
        end
        function o=ConvexIneqSatisfied(Problem,model,traj,trajPrev,epsilon)
            o=true;%satisfaction of convex state inequalities
            for k=1:Problem.N
                for i=1:model.xDim
                   if (traj.X(i,k)-model.xMax(i))>epsilon || (model.xMin(i)-traj.X(i,k))>epsilon
                       o=false;
                       break;
                   end
                end
            end
            for k=1:Problem.N
               for i=1:size(Problem.obstacles,2)
                  if max(ObstacleAvoidanceLinearized(model,Problem,traj.X,traj.U,trajPrev,k,i),0)>epsilon
                      o=false;
                      break;
                  end
               end
            end
        end
    end
end

