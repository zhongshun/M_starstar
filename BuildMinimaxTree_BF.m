% clear all; close all; clc;
% format long;
% 
% 
% Initial_Agent = [3;3];
% Initial_Opponent = [0;0];
% Obstacle_Set = [2 2 2 2 2;1 2 3 4 5];

function Vis = BuildMinimaxTree_BF(Initial_Agent,Initial_Opponent,Teammate,environment,Lookahead)

epsilon = 0.01;
snap_distance = 0.05;

Vis = digraph([1],[]);
Vis.Nodes.Agent_x= Initial_Agent(1);
Vis.Nodes.Agent_y= Initial_Agent(2);
Vis.Nodes.Opponent_x=Initial_Opponent(1);
Vis.Nodes.Opponent_y=Initial_Opponent(2);
Vis.Nodes.Generation = 1;

Vis.Nodes.Visited_Time = 1;
Vis.Nodes.Detection_time = 0;

V{1} = visibility_polygon( [Initial_Agent(1) Initial_Agent(2)] , environment , epsilon, snap_distance);
Vis.Nodes.Agent_Region{1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50);

W{1} = visibility_polygon( [Initial_Opponent(1) Initial_Opponent(2)] , environment , epsilon , snap_distance );
if in_environment( [Initial_Agent(1) Initial_Agent(2)] , W , epsilon )
    Vis.Nodes.Agent_Detection_time = 1;
else
    Vis.Nodes.Agent_Detection_time = 0;
end

if in_environment( [Teammate(1) Teammate(2)] , W , epsilon )
    Vis.Nodes.Teammate_Detection_time = 1;
else
    Vis.Nodes.Teammate_Detection_time = 0;
end


T = Lookahead;
New_Initial = 1;
New_End = 1;
Count = 1;

environment_min_x = min(environment{1}(:,1));
environment_max_x = max(environment{1}(:,1));
environment_min_y = min(environment{1}(:,2));
environment_max_y = max(environment{1}(:,2));

for i = 2:2*T+1
    Initial_node = New_Initial;
    End_node = New_End;
    if mod(i,2) == 0
        for j = Initial_node:End_node
            if j == Initial_node
                New_Initial = Count+1;
            end
            
            %x forward
            if in_environment( [Vis.Nodes.Agent_x(j)+1, Vis.Nodes.Agent_y(j)] , environment , 0.01 ) &&...
                   in_environment( [Vis.Nodes.Agent_x(j)+1/2, Vis.Nodes.Agent_y(j)] , environment , 0.01 ) 
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j)+1;
                Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j);
                Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j);
                Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j);
                Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j);
                Vis.Nodes.Parent(Count+1) = j;
                
                V{1} = visibility_polygon( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , environment , epsilon , snap_distance );
                Vis.Nodes.Agent_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Agent_Region{j};

                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            %x backward
            if in_environment( [Vis.Nodes.Agent_x(j)-1, Vis.Nodes.Agent_y(j)] , environment , 0.01 )&&...
                   in_environment( [Vis.Nodes.Agent_x(j)-1/2, Vis.Nodes.Agent_y(j)] , environment , 0.01 ) 
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j)-1;
                Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j);
                Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j);
                Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j);
                Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j);
                Vis.Nodes.Parent(Count+1) = j;
                
                V{1} = visibility_polygon( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , environment , epsilon , snap_distance );
                Vis.Nodes.Agent_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Agent_Region{j};
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            %y forward
            if in_environment( [Vis.Nodes.Agent_x(j), Vis.Nodes.Agent_y(j)+1] , environment , 0.01 )&&...
                   in_environment( [Vis.Nodes.Agent_x(j), Vis.Nodes.Agent_y(j)+1/2] , environment , 0.01 ) 
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j);
                Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j)+1;
                Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j);
                Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j);
                Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j);
                Vis.Nodes.Parent(Count+1) = j;
                
                V{1} = visibility_polygon( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , environment , epsilon , snap_distance );
                Vis.Nodes.Agent_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Agent_Region{j};
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            %y backward
            if in_environment( [Vis.Nodes.Agent_x(j), Vis.Nodes.Agent_y(j)-1] , environment , 0.01 )&&...
                   in_environment( [Vis.Nodes.Agent_x(j)+1/2, Vis.Nodes.Agent_y(j)-1/2] , environment , 0.01 ) 
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j);
                Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j)-1;
                Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j);
                Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j);
                Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j);
                Vis.Nodes.Parent(Count+1) = j;
                
                V{1} = visibility_polygon( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , environment , epsilon , snap_distance );
                Vis.Nodes.Agent_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Agent_Region{j};
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            
            
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
            if in_environment( [Vis.Nodes.Opponent_x(j)+1, Vis.Nodes.Opponent_y(j)] , environment , 0.01 ) &&...
                   in_environment( [Vis.Nodes.Opponent_x(j)+1/2, Vis.Nodes.Opponent_y(j)] , environment , 0.01 ) 
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j);
                Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j);
                Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j)+1;
                Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j);
                Vis.Nodes.Agent_Region{Count+1} = Vis.Nodes.Agent_Region{j};
                Vis.Nodes.Parent(Count+1) = j;
                
                W{1} = visibility_polygon( [Vis.Nodes.Opponent_x(Count+1) Vis.Nodes.Opponent_y(Count+1)] , environment , epsilon , snap_distance );
                if in_environment( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , W , epsilon )
                    Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j) + 1;
                else
                    Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                end

                if in_environment( [Teammate(1) Teammate(2)] , W , epsilon )
                    Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j) + 1;
                else
                    Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j);
                end
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            %x backward
            if in_environment( [Vis.Nodes.Opponent_x(j)-1, Vis.Nodes.Opponent_y(j)] , environment , 0.01 ) &&...
                   in_environment( [Vis.Nodes.Opponent_x(j)-1/2, Vis.Nodes.Opponent_y(j)] , environment , 0.01 ) 
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j);
                Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j);
                Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j)-1;
                Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j);
                Vis.Nodes.Agent_Region{Count+1} = Vis.Nodes.Agent_Region{j};
                Vis.Nodes.Parent(Count+1) = j;
                
                
                W{1} = visibility_polygon( [Vis.Nodes.Opponent_x(Count+1) Vis.Nodes.Opponent_y(Count+1)] , environment , epsilon , snap_distance );
                if in_environment( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , W , epsilon )
                    Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j) + 1;
                else
                    Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                end

                if in_environment( [Teammate(1) Teammate(2)] , W , epsilon )
                    Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j) + 1;
                else
                    Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j);
                end
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            %y forward
            if in_environment( [Vis.Nodes.Opponent_x(j), Vis.Nodes.Opponent_y(j)+1] , environment , 0.01 ) &&...
                   in_environment( [Vis.Nodes.Opponent_x(j), Vis.Nodes.Opponent_y(j)+1/2] , environment , 0.01 ) 
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j);
                Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j);
                Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j);
                Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j)+1;
                Vis.Nodes.Agent_Region{Count+1} = Vis.Nodes.Agent_Region{j};
                Vis.Nodes.Parent(Count+1) = j;
                
                W{1} = visibility_polygon( [Vis.Nodes.Opponent_x(Count+1) Vis.Nodes.Opponent_y(Count+1)] , environment , epsilon , snap_distance );
                if in_environment( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , W , epsilon )
                    Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j) + 1;
                else
                    Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                end

                if in_environment( [Teammate(1) Teammate(2)] , W , epsilon )
                    Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j) + 1;
                else
                    Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j);
                end
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            %y backward
            if in_environment( [Vis.Nodes.Opponent_x(j), Vis.Nodes.Opponent_y(j)-1] , environment , 0.01 ) &&...
                   in_environment( [Vis.Nodes.Opponent_x(j), Vis.Nodes.Opponent_y(j)-1/2] , environment , 0.01 ) 
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j);
                Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j);
                Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j);
                Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j)-1;
                Vis.Nodes.Agent_Region{Count+1} = Vis.Nodes.Agent_Region{j};
                Vis.Nodes.Parent(Count+1) = j;

                
                W{1} = visibility_polygon( [Vis.Nodes.Opponent_x(Count+1) Vis.Nodes.Opponent_y(Count+1)] , environment , epsilon , snap_distance );
                if in_environment( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , W , epsilon )
                    Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j) + 1;
                else
                    Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                end

                if in_environment( [Teammate(1) Teammate(2)] , W , epsilon )
                    Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j) + 1;
                else
                    Vis.Nodes.Teammate_Detection_time(Count+1) = Vis.Nodes.Teammate_Detection_time(j);
                end
                
                Vis.Nodes.Generation(Count+1) = i;
                Count = Count+1;
            end
            
            
            if j == End_node
                New_End = Count;
            end
            
        end
    end
    
    
end


% for i = 2*T+1 :-1:1
%     list =  find(Vis.Nodes.Generation == i);
%     if i == 2*T+1
%         for j=1:nnz(list)
%             Vis.Nodes.Decision_Value(list(j)) =  distance([Vis.Nodes.Agent_x(list(j)) Vis.Nodes.Agent_y(list(j))],[Vis.Nodes.Opponent_x(list(j)) Vis.Nodes.Opponent_y(list(j))]);
%             Vis.Nodes.Decision_Node(list(j)) = list(j);
%         end
%     elseif mod(i,2)
%         for j = 1:nnz(list)
%             Children_node = successors(Vis,list(j));
%             Vis.Nodes.Decision_Value(list(j)) = min(Vis.Nodes.Decision_Value(Children_node));
%             Best_value = Vis.Nodes.Decision_Value(list(j));
%             Best_node = intersect(Children_node,(find(Vis.Nodes.Generation == i+1 & Vis.Nodes.Decision_Value == Best_value)));
%             Vis.Nodes.Decision_Node(list(j)) = Best_node(1);
%         end
%     else
%         for j = 1:nnz(list)
%             Children_node = successors(Vis,list(j));
%             Vis.Nodes.Decision_Value(list(j)) = max(Vis.Nodes.Decision_Value(Children_node));
%             Best_value = Vis.Nodes.Decision_Value(list(j));
%             Best_node = intersect(Children_node,(find(Vis.Nodes.Generation == i+1 & Vis.Nodes.Decision_Value == Best_value)));
%             Vis.Nodes.Decision_Node(list(j)) = Best_node(1);
%         end
%     end
%     
% end
% % 
% % 
% %find the optimal path
% 
% 
% Node_path = 1;
% Best_node = 1;
% for i = 2:2*T+1
%     Best_node = Vis.Nodes.Decision_Node(Best_node);
%     Node_path = [Node_path Best_node];
% end
% 
% 
% for k =1:2*T+1
%     if mod(k,2)
%         Agent_path_x((k+1)/2) = Vis.Nodes.Agent_x(Node_path(k));
%         Agent_path_y((k+1)/2) = Vis.Nodes.Agent_y(Node_path(k));
%     else
%         Opponent_path_x(k/2) = Vis.Nodes.Opponent_x(Node_path(k));
%         Opponent_path_y(k/2) = Vis.Nodes.Opponent_y(Node_path(k));
%     end
% end
% 
% Agent_next = [Vis.Nodes.Agent_x(Node_path(2)); Vis.Nodes.Agent_y(Node_path(2))];
% Opponent_next = [Vis.Nodes.Opponent_x(Node_path(3)); Vis.Nodes.Opponent_y(Node_path(3))];


end