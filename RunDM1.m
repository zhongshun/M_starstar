function [Initial_Agent,Initial_Opponent,Initial_Agent_Region,Assets_Collected] = RunDM1(One_Pass,T,Asset_Position,Negtive_Reward,Negtive_Asset,Number_of_Function,Function_index_size,Visibility_Data,Region)


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
            for Function_M = 0:Number_of_Function-1
                E_them = E_them_temp;
                %                 Function_M = dec2bin(M,Function_index_size);
                %                 Detection_Asset_Collect = One_Pass.Nodes.Detection_Asset_Collect{list(j)};
                Index = Function_M;
                for N = Function_index_size:-1:1
%                     if bitand(Function_M, bitset(0,length(Asset_Position) - N + 1))
%                     if   mod(bitshift(Function_M,-(length(Asset_Position) - N + 1)),2)
                    if  mod(Index,2)
                        E_them = E_them - Detection_Asset_Collect(N) * Negtive_Asset;
                    end
                    Index = floor(Index/2);
                end
                One_Pass_Nodes_E_them(1,Function_M+1) = E_them;
            end
            
            
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
            
            %Find which function we need to use based on the wise up state
            %of the opponent
            Decision_Index_E_them = 0;
            M = length(One_Pass.Nodes.Detection_Asset_WiseUp_Index{list(j)});
            for CheckBit = 1:M
                if One_Pass.Nodes.Detection_Asset_WiseUp_Index{list(j)}(CheckBit) == 1
                    Decision_Index_E_them = bitset(Decision_Index_E_them,M - CheckBit + 1);
                end
            end
            %             Decision_Index_E_them = bin2dec(num2str(One_Pass.Nodes.Detection_Asset_WiseUp_Index{list(j)}'))+1;
            Decision_Index_E_them = Decision_Index_E_them + 1;
            
            
            
            Best_value = One_Pass.Nodes.E_them{Children_node(1)}(Decision_Index_E_them);
            
            for k = 1:nnz(Children_node)
                if Best_value > One_Pass.Nodes.E_them{Children_node(k)}(Decision_Index_E_them)
                    Best_value = One_Pass.Nodes.E_them{Children_node(k)}(Decision_Index_E_them);
                end
            end
            
            %Find the minimal value based on the wise up state of the
            %opponent
            Best_nodes = [];
            P = list(j);
            for k = 1:nnz(Children_node)
                if One_Pass.Nodes.E_them{Children_node(k)}(Decision_Index_E_them) == Best_value
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
            
            One_Pass.Nodes.Decision_Value(list(j)) = One_Pass.Nodes.E_them{Best_node}(Decision_Index_E_them);
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
            One_Pass.Nodes.Decision_Value(list(j)) = One_Pass.Nodes.E_them{Best_node}(Decision_Index_E_them);
            
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
                for M = 1:Number_of_Function
                    E_them(M) = max(E_them(M),One_Pass_Nodes_E_them_temp(M));
                end
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