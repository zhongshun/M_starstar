% clear all; close all; clc;
% format long;
% 
% 
% Initial_Robot = [3;3];
% Initial_Target = [0;0];
% Obstacle_Set = [2 2 2 2 2;1 2 3 4 5];

function [Robot_next,Target_next] = Minimax_seach_BF(Initial_Robot,Initial_Target,Obstacle_Set,Lookahead)

Vis = digraph([1],[]);
Vis.Nodes.Robot_x= Initial_Robot(1);
Vis.Nodes.Robot_y= Initial_Robot(2);
Vis.Nodes.Target_x=Initial_Target(1);
Vis.Nodes.Target_y=Initial_Target(2);
Vis.Nodes.Generation = 1;

T = Lookahead;
New_Initial = 1;
New_End = 1;
Count = 1;
for i = 2:2*T+1
    Initial_node = New_Initial;
    End_node = New_End;
    if mod(i,2) == 0
        for j = Initial_node:End_node
            if j == Initial_node
                New_Initial = Count+1;
            end
            
            %x forward
            if ~isInObstacle([Vis.Nodes.Robot_x(j)+1, Vis.Nodes.Robot_y(j)], Obstacle_Set)
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j)+1;
                Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
                Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            %x backward
            if ~isInObstacle([Vis.Nodes.Robot_x(j)-1, Vis.Nodes.Robot_y(j)] ,Obstacle_Set)
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j)-1;
                Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
                Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            %y forward
            if ~isInObstacle( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)+1] ,Obstacle_Set)
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
                Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j)+1;
                Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            %y backward
            if ~isInObstacle( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)-1] , Obstacle_Set)
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
                Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j)-1;
                Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
%             %stay here
%             if isInObstacle( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)] ,Obstacle_Set)
%                 Vis=addedge(Vis,j,Count+1);
%                 Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
%                 Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
%                 Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
%                 Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
%                 Vis.Nodes.Decision_Value(Count+1) = 0;
%                 Vis.Nodes.Decision_Node(Count+1) = 0;
%                 
%                 Vis.Nodes.Generation(Count+1) = i;
%                 Count = Count+1;
%             end
            
            if j == End_node
                New_End = Count;
            end
            
        end
    else
        for j = Initial_node:End_node
            if j == Initial_node
                New_Initial = Count+1;
            end
            
            %x forward
            Vis=addedge(Vis,j,Count+1);
            Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
            Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
            Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j)+0.7;
            Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
            
            Vis.Nodes.Generation(Count+1) = i;         
            Count = Count+1;
            
            %x backward
            Vis=addedge(Vis,j,Count+1);
            Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
            Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
            Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j)-0.7;
            Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
            
            Vis.Nodes.Generation(Count+1) = i;
            Count = Count+1;
            
            %y forward
            Vis=addedge(Vis,j,Count+1);
            Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
            Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
            Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
            Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j)+0.7;
            
            Vis.Nodes.Generation(Count+1) = i;
            Count = Count+1;
            
            %y backward
            Vis=addedge(Vis,j,Count+1);
            Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
            Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
            Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
            Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j)-0.7;
            
            Vis.Nodes.Generation(Count+1) = i;
            
            Count = Count+1;
            
            
            if j == End_node
                New_End = Count;
            end
            
        end
    end
    
    
end


for i = 2*T+1 :-1:1
    list =  find(Vis.Nodes.Generation == i);
    if i == 2*T+1
        for j=1:nnz(list)
            Vis.Nodes.Decision_Value(list(j)) =  distance([Vis.Nodes.Robot_x(list(j)) Vis.Nodes.Robot_y(list(j))],[Vis.Nodes.Target_x(list(j)) Vis.Nodes.Target_y(list(j))]);
            Vis.Nodes.Decision_Node(list(j)) = list(j);
        end
    elseif mod(i,2)
        for j = 1:nnz(list)
            Children_node = successors(Vis,list(j));
            Vis.Nodes.Decision_Value(list(j)) = min(Vis.Nodes.Decision_Value(Children_node));
            Best_value = Vis.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(Vis.Nodes.Generation == i+1 & Vis.Nodes.Decision_Value == Best_value)));
            Vis.Nodes.Decision_Node(list(j)) = Best_node(1);
        end
    else
        for j = 1:nnz(list)
            Children_node = successors(Vis,list(j));
            Vis.Nodes.Decision_Value(list(j)) = max(Vis.Nodes.Decision_Value(Children_node));
            Best_value = Vis.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(Vis.Nodes.Generation == i+1 & Vis.Nodes.Decision_Value == Best_value)));
            Vis.Nodes.Decision_Node(list(j)) = Best_node(1);
        end
    end
    
end
% 
% 
%find the optimal path


Node_path = 1;
Best_node = 1;
for i = 2:2*T+1
    Best_node = Vis.Nodes.Decision_Node(Best_node);
    Node_path = [Node_path Best_node];
end


for k =1:2*T+1
    if mod(k,2)
        Robot_path_x((k+1)/2) = Vis.Nodes.Robot_x(Node_path(k));
        Robot_path_y((k+1)/2) = Vis.Nodes.Robot_y(Node_path(k));
    else
        Target_path_x(k/2) = Vis.Nodes.Target_x(Node_path(k));
        Target_path_y(k/2) = Vis.Nodes.Target_y(Node_path(k));
    end
end

Robot_next = [Vis.Nodes.Robot_x(Node_path(2)); Vis.Nodes.Robot_y(Node_path(2))];
Target_next = [Vis.Nodes.Target_x(Node_path(3)); Vis.Nodes.Target_y(Node_path(3))];


end
