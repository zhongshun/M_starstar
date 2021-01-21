function [Initial_Agent,Initial_Opponent,Initial_Agent_Region] = RunDM1(Tree,T,Negtive_Reward,Negtive_Asset)

One_Pass = Tree;
Number_of_Asset = size(Asset_Position,1);
Number_of_Function = 0;
for i = 0:Number_of_Asset
    Number_of_Function = Number_of_Function + nchoosek(Number_of_Asset,i);
end
Function_index = dec2bin(Number_of_Function-1);
Function_index_size = size(Function_index,2);

for i = 2*T+1 :-1:1
    list =  find(One_Pass.Nodes.Generation == i);
    if i == 2*T+1
        for j=1:nnz(list)
            %   List all the reward value based on the detection of one
            %   of the assests or not
            E_them = [bwarea(One_Pass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* One_Pass.Nodes.Agent_Detection_time(list(j))];
            One_Pass.Nodes.E_them{list(j)} = E_them;
            
            for M = 0:Number_of_Function-1
                E_them = [bwarea(One_Pass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* One_Pass.Nodes.Agent_Detection_time(list(j))];
                Function_M = dec2bin(M,Function_index_size);
                for N = Function_index_size:-1:1
                    if Function_M(N) == '1' 
                        E_them = E_them - str2num(One_Pass.Nodes.Detection_Asset_Collect{list(j)}(N)) * Negtive_Asset;
                    end      
                end
                One_Pass.Nodes.E_them{list(j)}(1,M+1) = E_them;
            end
            
            One_Pass.Nodes.E_us(list(j)) = E_them; 
            One_Pass.Nodes.Decision_Node(list(j)) = list(j);
            
        end
    elseif ~mod(i,2) %MIN Level
        for j = 1:nnz(list)
            Children_node = successors(One_Pass,list(j));
            %Find which function we need to use based on the wise up state
            %of the opponent

            Decision_Index_E_them = bin2dec(One_Pass.Nodes.Detection_Asset_WiseUp_Index{list(j)}')+1;     
            Best_value = One_Pass.Nodes.E_them{Children_node(1)}(Decision_Index_E_them);
            
            for k = 1:nnz(Children_node)
                if Best_value > One_Pass.Nodes.E_them{Children_node(k)}(Decision_Index_E_them)
                    Best_value = One_Pass.Nodes.E_them{Children_node(k)}(Decision_Index_E_them);
                end
            end

           %Find the minimal value based on the wise up state of the
           %opponent 
           Best_nodes = [];
           for k = 1:nnz(Children_node)           
                if One_Pass.Nodes.E_them{Children_node(k)}(Decision_Index_E_them) == Best_value
                    Best_nodes(nnz(Best_nodes) + 1) = Children_node(k);
                end             
           end
           Best_node = Best_nodes(1);
           if nnz(Best_nodes) > 1
               Best_one = 1;
               for B = 1:nnz(Best_nodes)
                   if distance([One_Pass.Nodes.Agent_x(Best_nodes(B)),One_Pass.Nodes.Agent_y(Best_nodes(B))],[One_Pass.Nodes.Opponent_x(Best_nodes(B)),One_Pass.Nodes.Opponent_y(Best_nodes(B))])...
                           < distance([One_Pass.Nodes.Agent_x(Best_nodes(Best_one)),One_Pass.Nodes.Agent_y(Best_nodes(Best_one))],[One_Pass.Nodes.Opponent_x(Best_nodes(Best_one)),One_Pass.Nodes.Opponent_y(Best_nodes(Best_one))])
                       Best_one = B;
                   end
                   Best_node = Best_nodes(Best_one);
               end
           else
               Best_node = Best_nodes;
           end

           One_Pass.Nodes.Decision_Value(list(j)) = One_Pass.Nodes.E_them{Best_node}(Decision_Index_E_them);
           One_Pass.Nodes.Decision_Node(list(j)) = Best_node;
           One_Pass.Nodes.E_us(list(j)) = One_Pass.Nodes.E_us(Best_node);
           One_Pass.Nodes.E_them(list(j)) = One_Pass.Nodes.E_them(Best_node);
        end
    else %MAX Level
        for j = 1:nnz(list)
            Children_node = successors(One_Pass,list(j));

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
                   if One_Pass.Nodes.Agent_x(i) == One_Pass.Nodes.Agent_x(Children_node(k)) &&...
                           One_Pass.Nodes.Agent_y(i) == One_Pass.Nodes.Agent_y(Children_node(k))
                       Best_node = Best_nodes(k);
                       break
                   else
                       Best_node = Best_nodes(randi([1,nnz(Best_nodes)]));
                   end
               end
           else
               Best_node = Best_nodes;
           end
                 
           One_Pass.Nodes.Decision_Node(list(j)) = Best_node;
           One_Pass.Nodes.E_us(list(j)) = One_Pass.Nodes.E_us(Best_node);
           One_Pass.Nodes.Decision_Value(list(j)) = One_Pass.Nodes.E_them{Best_node}(Decision_Index_E_them);
           
           %Update E_them
           E_them = One_Pass.Nodes.E_them{Children_node(1)};
           for M = 1:Number_of_Function
                for k = 1:nnz(Children_node) 
                    E_them(M) = max(E_them(M),One_Pass.Nodes.E_them{Children_node(1)}(M));
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
    Agent_path_x((k+1)/2) = One_Pass.Nodes.Agent_x(One_Pass_Node_path(k));
    Agent_path_y((k+1)/2) = One_Pass.Nodes.Agent_y(One_Pass_Node_path(k));
    
    Opponent_path_x((k+1)/2) = One_Pass.Nodes.Opponent_x(One_Pass_Node_path(k));
    Opponent_path_y((k+1)/2) = One_Pass.Nodes.Opponent_y(One_Pass_Node_path(k));
end


Initial_Agent = [Agent_path_x(2);Agent_path_y(2)];
Initial_Opponent = [Opponent_path_x(2);Opponent_path_y(2)];
Initial_Agent_Region = ThirdPass.Nodes.Agent_Region{ThirdPass_Node_path(3)};

end