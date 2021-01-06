function [Initial_Agent,Initial_Opponent,Initial_Agent_Region, WiseUp] = RunMstar(Tree,T,Negtive_Reward,Negtive_Teammate)

E_them = @them_eval_fn;
E_us = @true_eval_fn;
E_smart = @true_eval_fn;

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
Initial_Agent = [Agent_path_x(2);Agent_path_y(2)];
Initial_Opponent = [Opponent_path_x(2);Opponent_path_y(2)];
Initial_Agent_Region = ThirdPass.Nodes.Agent_Region{ThirdPass_Node_path(3)};
WiseUp = ThirdPass.Nodes.WiseUp(ThirdPass_Node_path(3));