function o = GenerateObstacles(init,goal,oN,maxR)
%This function creates obstacles between init and goal points:
%circles with random center and radius
i=1;
failure_iter=0;
max_failure_iter=100;
aworld=double.empty(3,0);
while i<oN+1
    world(1,i)=randi([min(init(1,1),goal(1,1))*10,max(init(1,1),goal(1,1))*10])/10; %obstacle x
    world(2,i)=randi([min(init(2,1),goal(2,1))*10,max(init(2,1),goal(2,1))*10])/10; %obstacle y
    world(3,i)=randi([round(maxR*5) round(maxR*10)])/10; %obstacle radius
    clear=true;
    %checking if there is an overlap between new obstacle and old obstacle,
    %init or goal max_iter times
    if norm(world(1:2,i)-init(1:2))>2*world(3,i) && norm(world(1:2,i)-goal(1:2))>2*world(3,i)
        for j=1:i-1
            if norm(world(1:2,i)-world(1:2,j))<1.1*(world(3,i)+world(3,j))
                clear=false;
                failure_iter=failure_iter+1;
                break
            end
        end
    else
        clear=false;
        failure_iter=failure_iter+1;
    end
    if clear
        aworld(1,i)=world(1,i);
        aworld(2,i)=world(2,i);
        aworld(3,i)=world(3,i);
        i=i+1; 
    elseif failure_iter>max_failure_iter
        warning(['Area is too confined, can not fit all obstacles. Obstacle number changed to ' num2str(i-1)])
        break;
    end
end
o=aworld;
end
