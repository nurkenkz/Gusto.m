function o=ObjectiveFunction(X,U,trajPrev,Problem,model,omega,delta)
	Jcp=0; %convex penalty cost
    Jst=0; %state trust cost
    Jnp=0; %nonconvex penalty convexified cost
	for k=1:Problem.N 
        for i=1:model.xDim
        Jcp=Jcp+omega*max(X(i,k)-model.xMax(i),0.);
        Jcp=Jcp+omega*max(model.xMin(i)-X(i,k),0.);
        end
    end
    for k=1:Problem.N
		Jst=Jst+max(square_pos(norm(X(:,k)-trajPrev.X(:,k)))-delta,0.);
    end
    for k=1:Problem.N
        for i=1:size(Problem.obstacles,2)
              Jnp=Jnp+100*omega*max(ObstacleAvoidanceLinearized(model,Problem,X,U,trajPrev,k,i),0);
        end
    end  
    o=CostTrue(model,U,trajPrev)+Jcp+Jst+Jnp;
end

