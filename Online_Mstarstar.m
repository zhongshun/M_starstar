clear all; close all; clc;
%Robustness constant
epsilon = 0.000000001;


%Snap distance (distance within which an observer location will be snapped to the
%boundary before the visibility polygon is computed)
snap_distance = 0.05;

ENV_SIZE = 50;  % will be ENV_SIZE x ENV_SIZE grid

%Read environment geometry from file
environment = read_vertices_from_file('./Environments/M_starstar6.environment');
environment_min_x = min(environment{1}(:,1));
environment_max_x = max(environment{1}(:,1));
environment_min_y = min(environment{1}(:,2));
environment_max_y = max(environment{1}(:,2));
X_MIN = environment_min_x-0.1*(environment_max_x-environment_min_x);
X_MAX = environment_max_x+0.1*(environment_max_x-environment_min_x);
Y_MIN = environment_min_y-0.1*(environment_max_y-environment_min_y);
Y_MAX = environment_max_y+0.1*(environment_max_y-environment_min_y);

while 1
    Initial_Agent = [randi([round(X_MIN),round(X_MAX)]);randi([round(Y_MIN),round(Y_MAX)])];
    Initial_Opponent = [randi([round(X_MIN),round(X_MAX)]);randi([round(Y_MIN),round(Y_MAX)])];
    Teammate = [randi([round(X_MIN),round(X_MAX)]);randi([round(Y_MIN),round(Y_MAX)])];   
    if in_environment( Initial_Agent , environment , epsilon ) && in_environment( Initial_Agent , environment , epsilon )  && in_environment( Initial_Agent , environment , epsilon ) 
        break
    end
    
end

%The frequency that the teammate appear
Record_path_Agent = Initial_Agent;
Record_path_Opponent = Initial_Opponent;

Teammate_appear_mod = 3;
Teammate_appear_mod_E_smart = 3;

Negtive_Reward = 3;
Negtive_Teammate = 30;
WiseUp = 0;

Lookahead = 8;
T = Lookahead;

T_execution = 10;       % how many time steps to execute the online planner

V{1} = visibility_polygon( [Initial_Agent(1) Initial_Agent(2)] , environment , epsilon, snap_distance);
Initial_Agent_Region = poly2mask(V{1}(:,1),V{1}(:,2),ENV_SIZE, ENV_SIZE);
%The frquence
Pr = 1;
%E_them = bwarea(FirstPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* FirstPass.Nodes.Agent_Detection_time(list(j));
%E_smart with pr
%E_smart = bwarea(ThirdPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* ThirdPass.Nodes.Agent_Detection_time(list(j)) -Pr* Negtive_Teammate*(ThirdPass.Nodes.Teammate_Detection_time_E_smart(list(j)) >= 1);
%E_smaet with mod
%E_smaet = bwarea(ThirdPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* ThirdPass.Nodes.Agent_Detection_time(list(j)) - Negtive_Teammate*(ThirdPass.Nodes.Detection_time_E_smart(list(j)) >= 1);
%E_us = bwarea(ThirdPass.Nodes.Agent_Region{list(j)}) - Negtive_Reward* ThirdPass.Nodes.Agent_Detection_time(list(j)) - Negtive_Teammate*(ThirdPass.Nodes.Teammate_Detection_time(list(j)) >= 1);
for step = 1:T_execution
    
    %% Build the tree
    Tree = BuildMinimaxTree_Online(Initial_Agent,Initial_Opponent,step,Initial_Agent_Region,Teammate,environment,Teammate_appear_mod,Teammate_appear_mod_E_smart,WiseUp,Lookahead,ENV_SIZE);
    [Initial_Agent, Initial_Opponent, Initial_Agent_Region,WiseUp] = RunThreePasses(Tree,T,Negtive_Reward,Negtive_Teammate,Pr);
    
    Record_path_Agent(:,step + 1) = Initial_Agent
    Record_path_Opponent(:,step + 1) = Initial_Opponent
    
end

%%
save('Online.mat')

%%
Plot_Path