clear all; close all; clc;
%Robustness constant
epsilon = 0.000000001;


%Snap distance (distance within which an observer location will be snapped to the
%boundary before the visibility polygon is computed)
snap_distance = 0.05;


%Read environment geometry from file
environment = read_vertices_from_file('./Environments/Mstar_ED_edition.environment');

Initial_Agent = [5;7];
Initial_Opponent = [5;10];
Teammate = [10;7];
Negtive_Reward = 1;
Negtive_Teammate = 50;

Lookahead = 5;
T = Lookahead;

%% Build the tree
Tree = BuildMinimaxTree_BF(Initial_Agent,Initial_Opponent,Teammate,environment,Lookahead);

%% First Pass
M_star_FirstPass = Tree;
for i = 2*T+1 :-1:1
    list =  find(M_star_FirstPass.Nodes.Generation == i);
    if i == 2*T+1
        for j=1:nnz(list)
            %             M_star_FirstPass.Nodes.Decision_Value(list(j)) =  distance([M_star_FirstPass.Nodes.Agent_x(list(j)) M_star_FirstPass.Nodes.Agent_y(list(j))],[M_star_FirstPass.Nodes.Opponent_x(list(j)) M_star_FirstPass.Nodes.Opponent_y(list(j))]);
            M_star_FirstPass.Nodes.Decision_Value(list(j)) =  bwarea(M_star_FirstPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* M_star_FirstPass.Nodes.Agent_Detection_time(list(j));
            M_star_FirstPass.Nodes.Decision_Node(list(j)) = list(j);
        end
    elseif ~mod(i,2)
        for j = 1:nnz(list)
            Children_node = successors(M_star_FirstPass,list(j));
            M_star_FirstPass.Nodes.Decision_Value(list(j)) = min(M_star_FirstPass.Nodes.Decision_Value(Children_node));
            Best_value = M_star_FirstPass.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(M_star_FirstPass.Nodes.Generation == i+1 & M_star_FirstPass.Nodes.Decision_Value == Best_value)));
            M_star_FirstPass.Nodes.Decision_Node(list(j)) = Best_node(1);
            Best_one = 1;
            for B = 1:nnz(Best_node)
                if distance([M_star_FirstPass.Nodes.Agent_x(Best_node(B)),M_star_FirstPass.Nodes.Agent_y(Best_node(B))],[M_star_FirstPass.Nodes.Opponent_x(Best_node(B)),M_star_FirstPass.Nodes.Opponent_y(Best_node(B))])...
                        < distance([M_star_FirstPass.Nodes.Agent_x(Best_node(Best_one)),M_star_FirstPass.Nodes.Agent_y(Best_node(Best_one))],[M_star_FirstPass.Nodes.Opponent_x(Best_node(Best_one)),M_star_FirstPass.Nodes.Opponent_y(Best_node(Best_one))])
                    Best_one = B;
                end
            end
            M_star_FirstPass.Nodes.Decision_Node(list(j)) = Best_node(Best_one);
        end
    else
        for j = 1:nnz(list)
            Children_node = successors(M_star_FirstPass,list(j));
            M_star_FirstPass.Nodes.Decision_Value(list(j)) = max(M_star_FirstPass.Nodes.Decision_Value(Children_node));
            Best_value = M_star_FirstPass.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(M_star_FirstPass.Nodes.Generation == i+1 & M_star_FirstPass.Nodes.Decision_Value == Best_value)));
            M_star_FirstPass.Nodes.Decision_Node(list(j)) = Best_node(1);
            Best_one = 1;
            for B = 1:nnz(Best_node)
                if distance([M_star_FirstPass.Nodes.Agent_x(Best_node(B)),M_star_FirstPass.Nodes.Agent_y(Best_node(B))],[M_star_FirstPass.Nodes.Opponent_x(Best_node(B)),M_star_FirstPass.Nodes.Opponent_y(Best_node(B))])...
                        > distance([M_star_FirstPass.Nodes.Agent_x(Best_node(Best_one)),M_star_FirstPass.Nodes.Agent_y(Best_node(Best_one))],[M_star_FirstPass.Nodes.Opponent_x(Best_node(Best_one)),M_star_FirstPass.Nodes.Opponent_y(Best_node(Best_one))])
                    Best_one = B;
                end
            end
            M_star_FirstPass.Nodes.Decision_Node(list(j)) = Best_node(Best_one);
        end
    end
    
end

%% Second Pass
M_star_SecondPass = Tree;
for i = 2*T+1 :-1:1
    list =  find(M_star_SecondPass.Nodes.Generation == i);
    if i == 2*T+1
        for j=1:nnz(list)
            %             M_star_SecondPass.Nodes.Decision_Value(list(j)) =  distance([M_star_SecondPass.Nodes.Agent_x(list(j)) M_star_SecondPass.Nodes.Agent_y(list(j))],[M_star_SecondPass.Nodes.Opponent_x(list(j)) M_star_SecondPass.Nodes.Opponent_y(list(j))]);
            M_star_SecondPass.Nodes.Decision_Value(list(j)) =  bwarea(M_star_SecondPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* M_star_SecondPass.Nodes.Agent_Detection_time(list(j)) - Negtive_Teammate*(M_star_SecondPass.Nodes.Teammate_Detection_time(list(j)) >= 1);
            M_star_SecondPass.Nodes.Decision_Node(list(j)) = list(j);
        end
    elseif ~mod(i,2)
        for j = 1:nnz(list)
            M_star_SecondPass.Nodes.Decision_Node(list(j)) = M_star_FirstPass.Nodes.Decision_Node(list(j));
            M_star_SecondPass.Nodes.Decision_Value(list(j)) = M_star_SecondPass.Nodes.Decision_Value(M_star_SecondPass.Nodes.Decision_Node(list(j)));
        end
    else
        for j = 1:nnz(list)
            Children_node = successors(M_star_SecondPass,list(j));
            M_star_SecondPass.Nodes.Decision_Value(list(j)) = max(M_star_SecondPass.Nodes.Decision_Value(Children_node));
            Best_value = M_star_SecondPass.Nodes.Decision_Value(list(j));
            Best_node = intersect(Children_node,(find(M_star_SecondPass.Nodes.Generation == i+1 & M_star_SecondPass.Nodes.Decision_Value == Best_value)));
            M_star_SecondPass.Nodes.Decision_Node(list(j)) = Best_node(1);
            Best_one = 1;
            for B = 1:nnz(Best_node)
                if distance([M_star_SecondPass.Nodes.Agent_x(Best_node(B)),M_star_SecondPass.Nodes.Agent_y(Best_node(B))],[M_star_SecondPass.Nodes.Opponent_x(Best_node(B)),M_star_SecondPass.Nodes.Opponent_y(Best_node(B))])...
                        > distance([M_star_SecondPass.Nodes.Agent_x(Best_node(Best_one)),M_star_SecondPass.Nodes.Agent_y(Best_node(Best_one))],[M_star_SecondPass.Nodes.Opponent_x(Best_node(Best_one)),M_star_SecondPass.Nodes.Opponent_y(Best_node(Best_one))])
                    Best_one = B;
                end
            end
            M_star_SecondPass.Nodes.Decision_Node(list(j)) = Best_node(Best_one);
        end
    end
    
end


% %find the optimal path
M_star_FirstPass_Node_path = 1;
M_star_FirstPass_Best_node = 1;
for i = 2:2*T+1
    M_star_FirstPass_Best_node = M_star_FirstPass.Nodes.Decision_Node(M_star_FirstPass_Best_node);
    M_star_FirstPass_Node_path = [M_star_FirstPass_Node_path M_star_FirstPass_Best_node];
end

% %find the optimal path
M_star_SecondPass_Node_path = 1;
M_star_SecondPass_Best_node = 1;
for i = 2:2*T+1
    M_star_SecondPass_Best_node = M_star_SecondPass.Nodes.Decision_Node(M_star_SecondPass_Best_node);
    M_star_SecondPass_Node_path = [M_star_SecondPass_Node_path M_star_SecondPass_Best_node];
end

for k =1:2*T+1
    if mod(k,2)
        Agent_path_x((k+1)/2) = M_star_SecondPass.Nodes.Agent_x(M_star_SecondPass_Node_path(k));
        Agent_path_y((k+1)/2) = M_star_SecondPass.Nodes.Agent_y(M_star_SecondPass_Node_path(k));
    else
        Opponent_path_x(k/2) = M_star_SecondPass.Nodes.Opponent_x(M_star_SecondPass_Node_path(k));
        Opponent_path_y(k/2) = M_star_SecondPass.Nodes.Opponent_y(M_star_SecondPass_Node_path(k));
    end
end



% Agent_next = [Vis.Nodes.Agent_x(Node_path(2)); Vis.Nodes.Agent_y(Node_path(2))];
% Opponent_next = [Vis.Nodes.Opponent_x(Node_path(3)); Vis.Nodes.Opponent_y(Node_path(3))];