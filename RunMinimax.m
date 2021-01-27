function [Initial_Agent,Initial_Opponent,Initial_Agent_Region,Assets_Collected] = RunMinimax(One_Pass,T,Asset_Position,Negtive_Reward,...
                                                            Negtive_Asset,Number_of_Function,Function_index_size,Visibility_Data,Region,Asset_Visibility_Data,step)
epsilon = step;

for i = 2*T+1 :-1:1
    list =  find(One_Pass.Nodes.Generation == i);
    if i == 2*T+1
        for j=1:nnz(list)
            %   List all the reward value based on the detection of one
            %   of the assests or not
            E_them =  One_Pass.Nodes.Current_Step_reward(list(j));
            %             One_Pass.Nodes.E_them{list(j)} = E_them;
            E_them_temp = E_them;
            
            Detection_Asset_Collect = One_Pass.Nodes.Detection_Asset_Collect{list(j)}; %indicator to label which asset is collected along the path to this node

            for N = Function_index_size:-1:1
                %                     if bitand(Function_M, bitset(0,length(Asset_Position) - N + 1))
                %                     if   mod(bitshift(Function_M,-(length(Asset_Position) - N + 1)),2)
                E_them = E_them - (Detection_Asset_Collect(N)>0) * (Negtive_Asset + 0.01*(100-Detection_Asset_Collect(N)));
            end
            One_Pass_Nodes_E_them = E_them;
            
            
            
            One_Pass.Nodes.E_them{list(j)} = One_Pass_Nodes_E_them;
            One_Pass.Nodes.E_us(list(j)) = E_them;
            One_Pass.Nodes.Decision_Node(list(j)) = list(j);
            
        end
    elseif ~mod(i,2) %MIN Level
        for j = 1:nnz(list)
            %             Children_node = successors(One_Pass,list(j));
            
            if list(j) == 2
                a = 1;
            end
            Children_node = One_Pass.Nodes.Successors{list(j)};
            

              
            
            Best_value = One_Pass.Nodes.E_them{Children_node(1)};
            
            for k = 1:nnz(Children_node)
                if Best_value > One_Pass.Nodes.E_them{Children_node(k)}
                    Best_value = One_Pass.Nodes.E_them{Children_node(k)};
                end
            end
            
            %Find the minimal value based on the wise up state of the
            %opponent
            Best_nodes = [];
            P = list(j);
            for k = 1:nnz(Children_node)
                if One_Pass.Nodes.E_them{Children_node(k)} == Best_value
                    Best_nodes(length(Best_nodes) + 1) = Children_node(k);
                end
            end
            
            
            Best_node = Best_nodes(1);
            % get the minimal reward for current step
            if nnz(Best_nodes) > 1
                Best_one = 1;
                for B = 1:nnz(Best_nodes)
                    
                    if One_Pass.Nodes.Opponent{Best_nodes(B)}(1) == One_Pass.Nodes.Agent{P}(1) &&...
                            One_Pass.Nodes.Opponent{Best_nodes(B)}(1) == One_Pass.Nodes.Agent{P}(2)
                        Best_one = B;
                        break
                    elseif One_Pass.Nodes.Current_Step_reward_with_assest(Best_nodes(B)) < One_Pass.Nodes.Current_Step_reward_with_assest(Best_nodes(Best_one))
                        Best_one = B;
                    end
                end
                Best_node = Best_nodes(Best_one);
            else
                Best_node = Best_nodes;
            end
            
            
            %            if nnz(Best_nodes) > 1
            %                Best_one = 1;
            %                for B = 1:nnz(Best_nodes)
            %                    if distance([One_Pass.Nodes.Agent_x(Best_nodes(B)),One_Pass.Nodes.Agent_y(Best_nodes(B))],[One_Pass.Nodes.Opponent_x(Best_nodes(B)),One_Pass.Nodes.Opponent_y(Best_nodes(B))])...
            %                            < distance([One_Pass.Nodes.Agent_x(Best_nodes(Best_one)),One_Pass.Nodes.Agent_y(Best_nodes(Best_one))],[One_Pass.Nodes.Opponent_x(Best_nodes(Best_one)),One_Pass.Nodes.Opponent_y(Best_nodes(Best_one))])
            %                        Best_one = B;
            %                    end
            %                    Best_node = Best_nodes(Best_one);
            %                end
            %            else
            %                Best_node = Best_nodes;
            %            end
            
            One_Pass.Nodes.Decision_Value(list(j)) = One_Pass.Nodes.E_them{Best_node};
            One_Pass.Nodes.Decision_Node(list(j)) = Best_node;
            One_Pass.Nodes.E_us(list(j)) = One_Pass.Nodes.E_us(Best_node);
            One_Pass.Nodes.E_them(list(j)) = One_Pass.Nodes.E_them(Best_node);
        end
    else %MAX Level
        for j = 1:nnz(list)
            %             Children_node = successors(One_Pass,list(j));
            Children_node = One_Pass.Nodes.Successors{list(j)};
            
            
            Best_value = One_Pass.Nodes.E_us(Children_node(1));
            
            for k = 1:nnz(Children_node)
                if Best_value < One_Pass.Nodes.E_us(Children_node(k))
                    Best_value = One_Pass.Nodes.E_us(Children_node(k));
                end
            end
            
            %Find the maximal value based on the wise up state of the
            %opponent
            Best_nodes = [];
            for k = 1:nnz(Children_node)
                if One_Pass.Nodes.E_us(Children_node(k)) == Best_value
                    Best_nodes(nnz(Best_nodes) + 1) = Children_node(k);
                end
            end
            Best_node = Best_nodes(1);
            
            if nnz(Best_nodes) > 1
                for k = 1:nnz(Best_nodes)
                    if One_Pass.Nodes.Agent{i}(1) == One_Pass.Nodes.Agent{Children_node(k)}(1) &&...
                            One_Pass.Nodes.Agent{i}(2) == One_Pass.Nodes.Agent{Children_node(k)}(2)
                        Best_node = Best_nodes(k);
                        break
                    else
                        Best_one = 1;
                        for B = 1:nnz(Best_nodes)
                            if One_Pass.Nodes.Current_Step_reward_with_assest(Best_nodes(B)) > One_Pass.Nodes.Current_Step_reward_with_assest(Best_nodes(Best_one))
                                Best_one = B;
                                %                            elseif One_Pass.Nodes.Current_Step_reward(Best_nodes(B)) == One_Pass.Nodes.Current_Step_reward(Best_nodes(Best_one))
                                %                                if norm(One_Pass.Nodes.Agent{Best_nodes(B)}-One_Pass.Nodes.Agent{1})...
                                %                                        > norm(One_Pass.Nodes.Agent{Best_nodes(Best_one)}-One_Pass.Nodes.Opponent{1)
                                %                                    Best_one = B;
                                %                                end
                            end
                            Best_node = Best_nodes(Best_one);
                        end
                    end
                end
            else
                Best_node = Best_nodes;
            end
            
            One_Pass.Nodes.Decision_Node(list(j)) = Best_node;
            One_Pass.Nodes.E_us(list(j)) = One_Pass.Nodes.E_us(Best_node);
            One_Pass.Nodes.Decision_Value(list(j)) = One_Pass.Nodes.E_them{Best_node};
            
            %Update E_them
            %            E_them = One_Pass.Nodes.E_them{Children_node(1)};
            %            for M = 1:Number_of_Function
            %                 for k = 1:nnz(Children_node)
            %                     One_Pass_Nodes_E_them_temp = One_Pass.Nodes.E_them{Children_node(k)};
            %                     E_them(M) = max(E_them(M),One_Pass_Nodes_E_them_temp(M));
            %                 end
            %            end
            
            E_them = One_Pass.Nodes.E_them{Children_node(1)};
            
            for k = 1:nnz(Children_node)
                One_Pass_Nodes_E_them_temp = One_Pass.Nodes.E_them{Children_node(k)};
                E_them = max(E_them,One_Pass_Nodes_E_them_temp);
            end
            
            One_Pass.Nodes.E_them{list(j)} = E_them;
            
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

for k =1:2:nnz(One_Pass_Node_path)
    Agent_path_x((k+1)/2) = One_Pass.Nodes.Agent{One_Pass_Node_path(k)}(1);
    Agent_path_y((k+1)/2) = One_Pass.Nodes.Agent{One_Pass_Node_path(k)}(2);
    
    Opponent_path_x((k+1)/2) = One_Pass.Nodes.Opponent{One_Pass_Node_path(k)}(1);
    Opponent_path_y((k+1)/2) = One_Pass.Nodes.Opponent{One_Pass_Node_path(k)}(2);
end

Initial_Agent = [Agent_path_x(1);Agent_path_y(1)];
for k = 2:2:length(One_Pass_Node_path)
    if Initial_Agent(1) ~= One_Pass.Nodes.Agent{One_Pass_Node_path(k)}(1) || Initial_Agent(2) ~= One_Pass.Nodes.Agent{One_Pass_Node_path(k)}(2)
        Initial_Agent =  One_Pass.Nodes.Agent{One_Pass_Node_path(k)};
        break;
    end
end
Initial_Agent_Region = One_Pass.Nodes.Agent_Region{One_Pass_Node_path(k)};


Initial_Opponent = [Opponent_path_x(1);Opponent_path_y(1)];
for k = 3:2:length(One_Pass_Node_path)
    if Initial_Opponent(1) ~= One_Pass.Nodes.Opponent{One_Pass_Node_path(k)}(1)  || One_Pass.Nodes.Opponent{One_Pass_Node_path(k)}(2)
        Initial_Opponent =  One_Pass.Nodes.Opponent{One_Pass_Node_path(k)};
        break;
    end
end
% Initial_chose = k;
% for K = k:2:length(One_Pass_Node_path)
%     if norm(One_Pass.Nodes.Opponent{One_Pass_Node_path(1)}-One_Pass.Nodes.Opponent{One_Pass_Node_path(K)}) == 1 &&      
%         Initial_Opponent =  One_Pass.Nodes.Opponent{One_Pass_Node_path(K)};
%     end
% end

Assets_Collected = One_Pass.Nodes.Detection_Asset_Collect{One_Pass_Node_path(k)};

end