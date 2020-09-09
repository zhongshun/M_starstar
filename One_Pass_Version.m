clear all; close all; clc;
%Robustness constant
epsilon = 0.000000001;


%Snap distance (distance within which an observer location will be snapped to the
%boundary before the visibility polygon is computed)
snap_distance = 0.05;


%Read environment geometry from file
environment = read_vertices_from_file('./Environments/M_starstar6.environment');

Initial_Agent = [2;7];
Initial_Opponent = [3;8];
Teammate = [7;7];
%The frequency that the teammate appear
Teammate_appear_mod = 2;

Negtive_Reward = 1;
Negtive_Teammate = 5;

Lookahead = 3;  % was 8
T = Lookahead;


%The frquence
Pr = 0.5;
%E_them = bwarea(FirstPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* FirstPass.Nodes.Agent_Detection_time(list(j));
%E_smart with pr
%E_smart = bwarea(SecondPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* SecondPass.Nodes.Agent_Detection_time(list(j)) -Pr* Negtive_Teammate*(SecondPass.Nodes.Teammate_Detection_time_E_smart(list(j)) >= 1);
%E_smaet with mod
%E_smaet = bwarea(ThirdPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* ThirdPass.Nodes.Agent_Detection_time(list(j)) - Negtive_Teammate*(ThirdPass.Nodes.Detection_time_E_smart(list(j)) >= 1);
%E_us = bwarea(ThirdPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* ThirdPass.Nodes.Agent_Detection_time(list(j)) - Negtive_Teammate*(ThirdPass.Nodes.Teammate_Detection_time(list(j)) >= 1);


%% Build the tree
Tree = BuildMinimaxTree_BF(Initial_Agent,Initial_Opponent,Teammate,environment,Teammate_appear_mod,Lookahead);


%% One Pass
One_Pass = Tree;
for i = 2*T+1 :-1:1
    list =  find(One_Pass.Nodes.Generation == i);
    if i == 2*T+1
        for j=1:nnz(list)
             %   Compute E_them
            One_Pass.Nodes.Decision_Value_E_them(list(j)) =  bwarea(One_Pass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* One_Pass.Nodes.Agent_Detection_time(list(j));
            %   Compute E_smart
            One_Pass.Nodes.Decision_Value_E_smart(list(j)) =  bwarea(One_Pass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* One_Pass.Nodes.Agent_Detection_time(list(j)) - Pr*Negtive_Teammate*(One_Pass.Nodes.Teammate_Detection_time(list(j)) >= 1);
            %   Compute E_us
            One_Pass.Nodes.Decision_Value_E_us(list(j)) =  bwarea(One_Pass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* One_Pass.Nodes.Agent_Detection_time(list(j)) - Negtive_Teammate*(One_Pass.Nodes.Teammate_Detection_time(list(j)) >= 1);

            One_Pass.Nodes.Decision_Node(list(j)) = list(j);
        end
    elseif ~mod(i,2) %MIN Level
        for j = 1:nnz(list)
            Children_node = successors(One_Pass,list(j));
            
            One_Pass.Nodes.Decision_Value_E_them(list(j)) = min(One_Pass.Nodes.Decision_Value_E_them(Children_node));
            Best_value_E_them = One_Pass.Nodes.Decision_Value_E_them(list(j));
            Best_node_E_them = intersect(Children_node,(find(One_Pass.Nodes.Generation == i+1 & One_Pass.Nodes.Decision_Value_E_them == Best_value_E_them))); 
            
            One_Pass.Nodes.Decision_Value_E_smart(list(j)) = min(One_Pass.Nodes.Decision_Value_E_smart(Children_node));
            Best_value_E_smart = One_Pass.Nodes.Decision_Value_E_them(list(j));
            Best_node_E_smart = intersect(Children_node,(find(One_Pass.Nodes.Generation == i+1 & One_Pass.Nodes.Decision_Value_E_them == Best_value_E_smart))); 
            
            
            if One_Pass.Nodes.WiseUp(list(j)) == 0
                One_Pass.Nodes.Decision_Node(list(j)) = Best_node_E_them(1);
                Best_one = 1;
                ParentOfB = One_Pass.Nodes.Parent(list(j));
                for B = 1:nnz(Best_node_E_them)
                    if distance([One_Pass.Nodes.Agent_x(ParentOfB),One_Pass.Nodes.Agent_y(ParentOfB)],[One_Pass.Nodes.Opponent_x(Best_node_E_them(B)),One_Pass.Nodes.Opponent_y(Best_node_E_them(B))])...
                            < distance([One_Pass.Nodes.Agent_x(ParentOfB),One_Pass.Nodes.Agent_y(ParentOfB)],[One_Pass.Nodes.Opponent_x(Best_node_E_them(Best_one)),One_Pass.Nodes.Opponent_y(Best_node_E_them(Best_one))])
                        Best_one = B;
                    end
                end
                One_Pass.Nodes.Decision_Node(list(j)) = Best_node_E_them(Best_one);
                
            else
                One_Pass.Nodes.Decision_Node(list(j)) = Best_node_E_smart(1);
                Best_one = 1;
                ParentOfB = One_Pass.Nodes.Parent(list(j));
                for B = 1:nnz(Best_node_E_smart)
                    if distance([One_Pass.Nodes.Agent_x(ParentOfB),One_Pass.Nodes.Agent_y(ParentOfB)],[One_Pass.Nodes.Opponent_x(Best_node_E_smart(B)),One_Pass.Nodes.Opponent_y(Best_node_E_smart(B))])...
                            < distance([One_Pass.Nodes.Agent_x(ParentOfB),One_Pass.Nodes.Agent_y(ParentOfB)],[One_Pass.Nodes.Opponent_x(Best_node_E_smart(Best_one)),One_Pass.Nodes.Opponent_y(Best_node_E_smart(Best_one))])
                        Best_one = B;
                    end
                end
                One_Pass.Nodes.Decision_Node(list(j)) = Best_node_E_smart(Best_one);
            end
            
             One_Pass.Nodes.Decision_Value_E_us(list(j)) = One_Pass.Nodes.Decision_Value_E_us( One_Pass.Nodes.Decision_Node(list(j)) );
        end
    else
        for j = 1:nnz(list)
            Children_node = successors(One_Pass,list(j));
            
            One_Pass.Nodes.Decision_Value_E_them(list(j)) = max(One_Pass.Nodes.Decision_Value_E_them(Children_node));
            One_Pass.Nodes.Decision_Value_E_smart(list(j)) = max(One_Pass.Nodes.Decision_Value_E_smart(Children_node));
            One_Pass.Nodes.Decision_Value_E_us(list(j)) = max(One_Pass.Nodes.Decision_Value_E_us(Children_node));
            
            
            Best_value = One_Pass.Nodes.Decision_Value_E_us(list(j));
            Best_node = intersect(Children_node,(find(One_Pass.Nodes.Generation == i+1 & One_Pass.Nodes.Decision_Value_E_us == Best_value)));
            One_Pass.Nodes.Decision_Node(list(j)) = Best_node(1);
            Best_one = 1;
            for B = 1:nnz(Best_node)
                if distance([One_Pass.Nodes.Agent_x(Best_node(B)),One_Pass.Nodes.Agent_y(Best_node(B))],[One_Pass.Nodes.Opponent_x(Best_node(B)),One_Pass.Nodes.Opponent_y(Best_node(B))])...
                        > distance([One_Pass.Nodes.Agent_x(Best_node(Best_one)),One_Pass.Nodes.Agent_y(Best_node(Best_one))],[One_Pass.Nodes.Opponent_x(Best_node(Best_one)),One_Pass.Nodes.Opponent_y(Best_node(Best_one))])
                    Best_one = B;
                end
            end
            One_Pass.Nodes.Decision_Node(list(j)) = Best_node(Best_one);
        end
    end
    
end



% %find the optimal path
One_Pass_Node_path = 1;
One_Pass_Best_node = 1;
for i = 2:2*T+1
    One_Pass_Best_node = One_Pass.Nodes.Decision_Node(One_Pass_Best_node);
    One_Pass_Node_path = [One_Pass_Node_path One_Pass_Best_node];
end

% %find the optimal path

for k =1:2*T+1
    if mod(k,2)
        Agent_path_x((k+1)/2) = One_Pass.Nodes.Agent_x(One_Pass_Node_path(k));
        Agent_path_y((k+1)/2) = One_Pass.Nodes.Agent_y(One_Pass_Node_path(k));
    else
        Opponent_path_x(k/2) = One_Pass.Nodes.Opponent_x(One_Pass_Node_path(k));
        Opponent_path_y(k/2) = One_Pass.Nodes.Opponent_y(One_Pass_Node_path(k));
    end
end



% Agent_next = [Vis.Nodes.Agent_x(Node_path(2)); Vis.Nodes.Agent_y(Node_path(2))];
% Opponent_next = [Vis.Nodes.Opponent_x(Node_path(3)); Vis.Nodes.Opponent_y(Node_path(3))];