clear all; close all; clc;
%Robustness constant
epsilon = 0.000000001;


%Snap distance (distance within which an observer location will be snapped to the
%boundary before the visibility polygon is computed)
snap_distance = 0.05;


%Read environment geometry from file
environment = read_vertices_from_file('./Environments/M_starstar3.environment');

Initial_Agent = [1;5];
Initial_Opponent = [1;7];
Teammate = [10;7];
%The frequency that the teammate appear
Teammate_appear_mod = 6;

Negtive_Reward = 1;
Negtive_Teammate = 5;

Lookahead = 2;  % was 8
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

%% First Pass
FirstPass = Tree;
for i = 2*T+1 :-1:1
    list =  find(FirstPass.Nodes.Generation == i);
    if i == 2*T+1
        for j=1:nnz(list)
%             FirstPass.Nodes.Decision_Value(list(j)) =  distance([FirstPass.Nodes.Agent_x(list(j)) FirstPass.Nodes.Agent_y(list(j))],[FirstPass.Nodes.Opponent_x(list(j)) FirstPass.Nodes.Opponent_y(list(j))]);
            FirstPass.Nodes.Decision_Value(list(j)) =  bwarea(FirstPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* FirstPass.Nodes.Agent_Detection_time(list(j));
            FirstPass.Nodes.Decision_Node(list(j)) = list(j);
        end
    elseif ~mod(i,2)
        for j = 1:nnz(list)
            Children_node = successors(FirstPass,list(j));
            FirstPass.Nodes.Decision_Value(list(j)) = min(FirstPass.Nodes.Decision_Value(Children_node));
            Best_value = FirstPass.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(FirstPass.Nodes.Generation == i+1 & FirstPass.Nodes.Decision_Value == Best_value)));            
            FirstPass.Nodes.Decision_Node(list(j)) = Best_node(1);
            Best_one = 1;
            ParentOfB = SecondPass.Nodes.Parent(list(j));
            for B = 1:nnz(Best_node)
                if distance([FirstPass.Nodes.Agent_x(ParentOfB),FirstPass.Nodes.Agent_y(ParentOfB)],[FirstPass.Nodes.Opponent_x(Best_node(B)),FirstPass.Nodes.Opponent_y(Best_node(B))])...
                        < distance([FirstPass.Nodes.Agent_x(ParentOfB),FirstPass.Nodes.Agent_y(ParentOfB)],[FirstPass.Nodes.Opponent_x(Best_node(Best_one)),FirstPass.Nodes.Opponent_y(Best_node(Best_one))])
                    Best_one = B;
                end
            end
            FirstPass.Nodes.Decision_Node(list(j)) = Best_node(Best_one);
        end
    else
        for j = 1:nnz(list)
            Children_node = successors(FirstPass,list(j));
            FirstPass.Nodes.Decision_Value(list(j)) = max(FirstPass.Nodes.Decision_Value(Children_node));
            Best_value = FirstPass.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(FirstPass.Nodes.Generation == i+1 & FirstPass.Nodes.Decision_Value == Best_value)));
            FirstPass.Nodes.Decision_Node(list(j)) = Best_node(1);
            Best_one = 1;
            for B = 1:nnz(Best_node)
                if distance([FirstPass.Nodes.Agent_x(Best_node(B)),FirstPass.Nodes.Agent_y(Best_node(B))],[FirstPass.Nodes.Opponent_x(Best_node(B)),FirstPass.Nodes.Opponent_y(Best_node(B))])...
                        > distance([FirstPass.Nodes.Agent_x(Best_node(Best_one)),FirstPass.Nodes.Agent_y(Best_node(Best_one))],[FirstPass.Nodes.Opponent_x(Best_node(Best_one)),FirstPass.Nodes.Opponent_y(Best_node(Best_one))])
                    Best_one = B;
                end
            end
            FirstPass.Nodes.Decision_Node(list(j)) = Best_node(Best_one);
        end
    end
    
end

%% Second Pass
SecondPass = Tree;
for i = 2*T+1 :-1:1
    list =  find(SecondPass.Nodes.Generation == i);
    if i == 2*T+1
        for j=1:nnz(list)
%             SecondPass.Nodes.Decision_Value(list(j)) =  distance([SecondPass.Nodes.Agent_x(list(j)) SecondPass.Nodes.Agent_y(list(j))],[SecondPass.Nodes.Opponent_x(list(j)) SecondPass.Nodes.Opponent_y(list(j))]);
            SecondPass.Nodes.Decision_Value(list(j)) =  bwarea(SecondPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* SecondPass.Nodes.Agent_Detection_time(list(j)) -Pr* Negtive_Teammate*(SecondPass.Nodes.Teammate_Detection_time(list(j)) >= 1);
            SecondPass.Nodes.Decision_Node(list(j)) = list(j);
        end
    elseif ~mod(i,2)
        for j = 1:nnz(list)
            Children_node = successors(SecondPass,list(j));
            SecondPass.Nodes.Decision_Value(list(j)) = min(SecondPass.Nodes.Decision_Value(Children_node));
            Best_value = SecondPass.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(SecondPass.Nodes.Generation == i+1 & SecondPass.Nodes.Decision_Value == Best_value)));            
            SecondPass.Nodes.Decision_Node(list(j)) = Best_node(1);
            Best_one = 1;
            ParentOfB = SecondPass.Nodes.Parent(list(j));
            for B = 1:nnz(Best_node)
                if distance([SecondPass.Nodes.Agent_x(ParentOfB),SecondPass.Nodes.Agent_y(ParentOfB)],[SecondPass.Nodes.Opponent_x(Best_node(B)),SecondPass.Nodes.Opponent_y(Best_node(B))])...
                        < distance([SecondPass.Nodes.Agent_x(ParentOfB),SecondPass.Nodes.Agent_y(ParentOfB)],[SecondPass.Nodes.Opponent_x(Best_node(Best_one)),SecondPass.Nodes.Opponent_y(Best_node(Best_one))])
                    Best_one = B;
                end
            end
            SecondPass.Nodes.Decision_Node(list(j)) = Best_node(Best_one);
        end
    else
        for j = 1:nnz(list)
            Children_node = successors(SecondPass,list(j));
            SecondPass.Nodes.Decision_Value(list(j)) = max(SecondPass.Nodes.Decision_Value(Children_node));
            Best_value = SecondPass.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(SecondPass.Nodes.Generation == i+1 & SecondPass.Nodes.Decision_Value == Best_value)));
            SecondPass.Nodes.Decision_Node(list(j)) = Best_node(1);
            Best_one = 1;          
            for B = 1:nnz(Best_node)
                if distance([SecondPass.Nodes.Agent_x(Best_node(B)),SecondPass.Nodes.Agent_y(Best_node(B))],[SecondPass.Nodes.Opponent_x(Best_node(B)),SecondPass.Nodes.Opponent_y(Best_node(B))])...
                        > distance([SecondPass.Nodes.Agent_x(Best_node(Best_one)),SecondPass.Nodes.Agent_y(Best_node(Best_one))],[SecondPass.Nodes.Opponent_x(Best_node(Best_one)),SecondPass.Nodes.Opponent_y(Best_node(Best_one))])
                    Best_one = B;
                end
            end
            SecondPass.Nodes.Decision_Node(list(j)) = Best_node(Best_one);
        end
    end
    
end

%% Third Pass
ThirdPass = Tree;
for i = 2*T+1 :-1:1
    list =  find(ThirdPass.Nodes.Generation == i);
    if i == 2*T+1
        for j=1:nnz(list)
%             ThirdPass.Nodes.Decision_Value(list(j)) =  distance([ThirdPass.Nodes.Agent_x(list(j)) ThirdPass.Nodes.Agent_y(list(j))],[ThirdPass.Nodes.Opponent_x(list(j)) ThirdPass.Nodes.Opponent_y(list(j))]);
            ThirdPass.Nodes.Decision_Value(list(j)) =  bwarea(ThirdPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* ThirdPass.Nodes.Agent_Detection_time(list(j)) - Negtive_Teammate*(ThirdPass.Nodes.Teammate_Detection_time(list(j)) >= 1);
            ThirdPass.Nodes.Decision_Node(list(j)) = list(j);
        end
    elseif ~mod(i,2)
        for j = 1:nnz(list)
            if ThirdPass.Nodes.Wiseup(list(j)) == 0
                ThirdPass.Nodes.Decision_Node(list(j)) = FirstPass.Nodes.Decision_Node(list(j));
                ThirdPass.Nodes.Decision_Value(list(j)) = ThirdPass.Nodes.Decision_Value(ThirdPass.Nodes.Decision_Node(list(j)));
            else
                ThirdPass.Nodes.Decision_Node(list(j)) = SecondPass.Nodes.Decision_Node(list(j));
                ThirdPass.Nodes.Decision_Value(list(j)) = ThirdPass.Nodes.Decision_Value(ThirdPass.Nodes.Decision_Node(list(j)));
            end
        end
    else
        for j = 1:nnz(list)
            Children_node = successors(ThirdPass,list(j));
            ThirdPass.Nodes.Decision_Value(list(j)) = max(ThirdPass.Nodes.Decision_Value(Children_node));
            Best_value = ThirdPass.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(ThirdPass.Nodes.Generation == i+1 & ThirdPass.Nodes.Decision_Value == Best_value)));
            ThirdPass.Nodes.Decision_Node(list(j)) = Best_node(1);
            Best_one = 1;
            for B = 1:nnz(Best_node)
                if distance([ThirdPass.Nodes.Agent_x(Best_node(B)),ThirdPass.Nodes.Agent_y(Best_node(B))],[ThirdPass.Nodes.Opponent_x(Best_node(B)),ThirdPass.Nodes.Opponent_y(Best_node(B))])...
                        > distance([ThirdPass.Nodes.Agent_x(Best_node(Best_one)),ThirdPass.Nodes.Agent_y(Best_node(Best_one))],[ThirdPass.Nodes.Opponent_x(Best_node(Best_one)),ThirdPass.Nodes.Opponent_y(Best_node(Best_one))])
                    Best_one = B;
                end
            end
            ThirdPass.Nodes.Decision_Node(list(j)) = Best_node(Best_one);
        end
    end 
end

% %find the optimal path
FirstPass_Node_path = 1;
FirstPass_Best_node = 1;
for i = 2:2*T+1
    FirstPass_Best_node = FirstPass.Nodes.Decision_Node(FirstPass_Best_node);
    FirstPass_Node_path = [FirstPass_Node_path FirstPass_Best_node];
end

% %find the optimal path
ThirdPass_Node_path = 1;
ThirdPass_Best_node = 1;
for i = 2:2*T+1
    ThirdPass_Best_node = ThirdPass.Nodes.Decision_Node(ThirdPass_Best_node);
    ThirdPass_Node_path = [ThirdPass_Node_path ThirdPass_Best_node];
end

for k =1:2*T+1
    if mod(k,2)
        Agent_path_x((k+1)/2) = ThirdPass.Nodes.Agent_x(ThirdPass_Node_path(k));
        Agent_path_y((k+1)/2) = ThirdPass.Nodes.Agent_y(ThirdPass_Node_path(k));
    else
        Opponent_path_x(k/2) = ThirdPass.Nodes.Opponent_x(ThirdPass_Node_path(k));
        Opponent_path_y(k/2) = ThirdPass.Nodes.Opponent_y(ThirdPass_Node_path(k));
    end
end



% Agent_next = [Vis.Nodes.Agent_x(Node_path(2)); Vis.Nodes.Agent_y(Node_path(2))];
% Opponent_next = [Vis.Nodes.Opponent_x(Node_path(3)); Vis.Nodes.Opponent_y(Node_path(3))];