% clear all; close all; clc;
% format long;
% 
% 
% Initial_Agent = [3;3];
% Initial_Opponent = [0;0];
% Obstacle_Set = [2 2 2 2 2;1 2 3 4 5];

function Vis = BuildMinimaxTree_BF2(Initial_Agent,Initial_Opponent,Asset,environment,Lookahead)

Number_of_Asset = size(Asset,1);
Number_of_Function = 0;
for i = 0:Number_of_Asset
    Number_of_Function = Number_of_Function + nchoosek(Number_of_Asset,i);
end

Function_index = dec2bin(Number_of_Function-1);

epsilon = 0.01;
snap_distance = 0.05;

Vis = digraph([1],[]);
Vis.Nodes.Agent_x= Initial_Agent(1);
Vis.Nodes.Agent_y= Initial_Agent(2);
Vis.Nodes.Opponent_x=Initial_Opponent(1);
Vis.Nodes.Opponent_y=Initial_Opponent(2);
Vis.Nodes.Generation = 1;

Vis.Nodes.Visited_Time = 1;
Vis.Nodes.Detection_Asset_WiseUp_Index{1} = num2str(zeros(Number_of_Asset,1));
Vis.Nodes.Detection_Asset_Collect{1} = num2str(zeros(Number_of_Asset,1));

Vis.Nodes.WiseUp = 0;

V{1} = visibility_polygon( [Initial_Agent(1) Initial_Agent(2)] , environment , epsilon, snap_distance);
Vis.Nodes.Agent_Region{1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50);

W{1} = visibility_polygon( [Initial_Opponent(1) Initial_Opponent(2)] , environment , epsilon , snap_distance );
if in_environment( [Initial_Agent(1) Initial_Agent(2)] , W , epsilon )
    Vis.Nodes.Agent_Detection_time = 1;
else
    Vis.Nodes.Agent_Detection_time = 0;
end

for N = 1:Number_of_Asset
    if in_environment( [Asset(N,1) Asset(N,2)] , W , epsilon )
        Vis.Nodes.Detection_Asset_WiseUp_Index{1}(N) = '1';
    end
end


T = Lookahead;
New_Initial = 1;
New_End = 1;
Count = 1;


environment_min_x = min(environment{1}(:,1));
environment_max_x = max(environment{1}(:,1));
environment_min_y = min(environment{1}(:,2));
environment_max_y = max(environment{1}(:,2));

Action_Space = [1 0;0 1;-1 0; 0 -1];

for i = 2:2*T+1
    Current_step = ceil(i/2);
    Initial_node = New_Initial;
    End_node = New_End;

        
    if mod(i,2) == 0
        for j = Initial_node:End_node
            if j == Initial_node
                New_Initial = Count+1;
            end
            if Count == 2322
                a = 1
            end
            
            for actions = 1:size(Action_Space,1)
                
                if in_environment( [Vis.Nodes.Agent_x(j)+Action_Space(actions,1), Vis.Nodes.Agent_y(j)+Action_Space(actions,2)] , environment , 0.01 ) &&...
                        in_environment( [Vis.Nodes.Agent_x(j)+Action_Space(actions,1)*1/2, Vis.Nodes.Agent_y(j)+Action_Space(actions,2)*1/2] , environment , 0.01 )
                    Vis=addedge(Vis,j,Count+1);
                    Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j)+Action_Space(actions,1);
                    Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j)+Action_Space(actions,2);
                    Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j);
                    Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j);
                    Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                    Vis.Nodes.WiseUp(Count+1) = Vis.Nodes.WiseUp(j);
                    Vis.Nodes.Detection_Asset_WiseUp_Index(Count+1) = Vis.Nodes.Detection_Asset_WiseUp_Index(j);
                    %                     Vis.Nodes.Asset_Detection_time_E_smart(Count+1) = Vis.Nodes.Asset_Detection_time_E_smart(j);
                    Vis.Nodes.Detection_Asset_Collect{Count+1} = Vis.Nodes.Detection_Asset_Collect{j};
                    Vis.Nodes.Parent(Count+1) = j;
                    
                    V{1} = visibility_polygon( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , environment , epsilon , snap_distance );
                    Vis.Nodes.Agent_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Agent_Region{j};
                    
                    Vis.Nodes.Generation(Count+1) = i;
                    Count = Count+1;
                end
                
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
            
            for actions = 1:size(Action_Space,1)
                if in_environment( [Vis.Nodes.Opponent_x(j)+Action_Space(actions,1), Vis.Nodes.Opponent_y(j)+Action_Space(actions,2)] , environment , 0.01 ) &&...
                        in_environment( [Vis.Nodes.Opponent_x(j)+0.5*Action_Space(actions,1), Vis.Nodes.Opponent_y(j)+0.5*Action_Space(actions,2)] , environment , 0.01 )
                    Vis=addedge(Vis,j,Count+1);
                    Vis.Nodes.Agent_x(Count+1) = Vis.Nodes.Agent_x(j);
                    Vis.Nodes.Agent_y(Count+1) = Vis.Nodes.Agent_y(j);
                    Vis.Nodes.Opponent_x(Count+1) = Vis.Nodes.Opponent_x(j)+Action_Space(actions,1);
                    Vis.Nodes.Opponent_y(Count+1) = Vis.Nodes.Opponent_y(j)+Action_Space(actions,2);
                    Vis.Nodes.Agent_Region{Count+1} = Vis.Nodes.Agent_Region{j};
                    Vis.Nodes.Parent(Count+1) = j;
                    
                    W{1} = visibility_polygon( [Vis.Nodes.Opponent_x(Count+1) Vis.Nodes.Opponent_y(Count+1)] , environment , epsilon , snap_distance );
                    if in_environment( [Vis.Nodes.Agent_x(Count+1) Vis.Nodes.Agent_y(Count+1)] , W , epsilon )
                        Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j) + 1;
                    else
                        Vis.Nodes.Agent_Detection_time(Count+1) = Vis.Nodes.Agent_Detection_time(j);
                    end
                    
                    Vis.Nodes.Detection_Asset_WiseUp_Index(Count+1) = Vis.Nodes.Detection_Asset_WiseUp_Index(j);
                    for N = 1:Number_of_Asset
                        if in_environment( [Asset(N,1) Asset(N,2)] , W , epsilon )
                            Vis.Nodes.Detection_Asset_WiseUp_Index{Count+1}(N) = '1';
                        end
                    end
                    %Check if one assest was being collected
                    Vis.Nodes.Detection_Asset_Collect{Count+1} = Vis.Nodes.Detection_Asset_Collect{j};
                    for N = 1:Number_of_Asset
                        if  Asset(N,1) == Vis.Nodes.Opponent_x(Count+1) &&  Asset(N,2) == Vis.Nodes.Opponent_y(Count+1)
                            Vis.Nodes.Detection_Asset_Collect{Count+1}(N) = '1';
                        end
                    end
                    
                    
                    Vis.Nodes.Generation(Count+1) = i;
                    Count = Count+1;
                end
            end
            
            
            
            if j == End_node
                New_End = Count;
            end
            
        end
    end
    
    
end



end