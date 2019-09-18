function Output(Solution,model,init,goal,world)
%visualizes init and goal points, arrows showing theta direction,
%obstacles, final accepted trajectory and number of iterations in the title
text(init(1),init(2),'init','HorizontalAlignment','center');
hold on
text(goal(1),goal(2),'goal','VerticalAlignment','top');
q=quiver(init(1),init(2),model.v*cos(init(3)),model.v*sin(init(3)),'HandleVisibility','off');
q.AutoScaleFactor=0.3;
q.MaxHeadSize=1;
p=quiver(goal(1),goal(2),model.v*cos(goal(3)),model.v*sin(goal(3)),'HandleVisibility','off');
p.AutoScaleFactor=0.3;
p.MaxHeadSize=1;
xlabel("x")
ylabel("y")
axis("equal")
for i=1:size(world,2)
    viscircles([world(1,i) world(2,i)],world(3,i),'Color','r');
end
if strcmp(Solution.status,'Solved') || strcmp(Solution.status,'Inaccurate/Solved')
    plot(Solution.traj.X(1,:),Solution.traj.X(2,:),'b','LineWidth',2 )
end
legend('Optimal Path')
title(['GuSTO for Dubins Car (' num2str(Solution.iterations) ' iterations)'])
hold off
end

