classdef DubinsCar
    properties
    xDim %x(1):=x,x(2):=y,x(3):=theta
	uDim %turning rate control
	v  %:=speed
	k  %:=curvature parameter
	xMin  
	xMax  
	uMin
	uMax
	f
	A
	B
    clearance
    end
    methods
        function obj=DubinsCar()
            obj.xDim=3;
        	obj.uDim=1;
        	obj.v=2.;
        	obj.k=1;
        	obj.xMax=[100.; 100.; 2*pi];
        	obj.xMin=-obj.xMax;
        	obj.uMax=10.;
        	obj.uMin=-obj.uMax;
            obj.f=[];
        	obj.A=[];
        	obj.B=[];
            obj.clearance=0.01;
        end
        function o=fDynamic(model,x,u)
            xDim=model.xDim;
            f=zeros(xDim,1);
            f(1)=model.v*cos(x(3));
            f(2)=model.v*sin(x(3));
        	f(3)=model.k*u(1);
            o=f;
        end
        function o=ADynamic(model,x)
            xDim=model.xDim;
            A=zeros(xDim, xDim);
            A(1,3)=-model.v*sin(x(3));
            A(2,3)=model.v*cos(x(3));
            o=A;
        end
        function o=BDynamic(model,x)
            o=[0;0;model.k];
        end
        function o=CostTrue(obj,U,trajPrev)
            o=trajPrev.dt*pow_pos(norm(U(1,:)),2);
        end
        function [o1,o2,o3]=InitializeModelParams(model,Problem,trajPrev)
        	N=Problem.N;
            xDim=model.xDim;
            uDim=model.uDim;
            Xp=trajPrev.X;
            Up=trajPrev.U;
            model.f=[];
            model.A=[];
            model.B=BDynamic(model,Xp(:,1));
        	for k=1:N-1
                model.f=[model.f fDynamic(model,Xp(:,k),Up(:,k))];
            	model.A(:,:,k)=ADynamic(model,Xp(:,k));
            end
            o1=model.f;
            o2=model.A;
            o3=model.B;
        end
        function [o1,o2]=UpdateModelParams(model,Problem,trajPrev)
            N=Problem.N;
            X=trajPrev.X;
            U=trajPrev.U;
            f=model.f;
            A=model.A;
            for k = 1:N-1
            	f(1,k)=model.v*cos(X(3,k));
                f(2,k)=model.v*sin(X(3,k));
                f(3,k)=model.k*U(1,k);
            	A(1,3,k)=-model.v*sin(X(3,k));
                A(2,3,k)=model.v*cos(X(3,k));
            end
            o1=f;
            o2=A;
        end
        function o=DynamicConstraints(model,Problem,X,U,Tff,trajPrev, k, i)
           Xp=trajPrev.X;
           Up=trajPrev.U;
           Tfp=trajPrev.Tff;
           dh=Problem.dh;
           o=Tff*model.f(:,k)+Tfp*(model.A(:,:,k)*(X(:,k)-Xp(:,k))+ model.B*(U(:,k)-Up(:,k)))-(X(:,k+1)-X(:,k))/dh;
        end
        function o=ObstacleAvoidanceLinearized(model,Problem,X,U,trajPrev,k,i)
           %from Eq 12a-12c in [16]
           dhat=0;
           dhatp=[0.01;0.01];
           rp=trajPrev.X(1:2,k);%previous trajectory point
           r=X(1:2,k);%current trajectory point
           obsC=Problem.obstacles(1:2,i);%obstraction center
           obsR=Problem.obstacles(3,i);%obstraction radius
           p_chaser=rp;%trajectory collision point
           if norm(p_chaser-obsC)>0
              p_ko=obsC+obsR*(p_chaser-obsC)/norm(p_chaser-obsC);%obstraction collision point 
              di=norm(p_chaser-obsC)-obsR;%signed distance between trajectory and obstacle
              if di<Problem.obstacleTolerance
                if di>0.01
                   dhat=(p_chaser-p_ko)./norm(p_chaser-p_ko);
                      elseif di<-0.01
                         dhat=(p_ko-p_chaser)./norm(p_ko-p_chaser);
                else
                    dhat=dhatp;%to avoid NaN value in objective functions
                end
                o=model.clearance-(di+dhat'*(r-rp));
             else
                o=0;
             end
           else
             o=model.clearance+obsR;
           end
        end
    end  
end

