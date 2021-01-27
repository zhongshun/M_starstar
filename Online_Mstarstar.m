clear all; close all; clc;
%Robustness constant
epsilon = 0.000000001;

% load('Save_Visibility_Data\M_starstar12.mat')

%Snap distance (distance within which an observer location will be snapped to the
% %boundary before the visibility polygon is computed)
% snap_distance = 0.05;
% 
%  ENV_SIZE1 = 50;  % will be ENV_SIZE x ENV_SIZE grid
%  ENV_SIZE2 = 25; 
% 
% %Read environment geometry from file
environment = read_vertices_from_file('./Environments/M_starstar12.environment');

Initial_Agent = [12;5];
Initial_Opponent = [12;4];

Asset = [4 7; 16 8;17 8;16 6; 17 6];
Number_of_Asset = size(Asset,1);


%The frequency that the teammate appear
Record_path_Agent = Initial_Agent;
Record_path_Opponent = Initial_Opponent;

% Teammate_appear_mod = 3;
% Teammate_appear_mod_E_smart = 3;
Detection_Asset_Collect = zeros(Number_of_Asset,1);

Negtive_Reward = 1;
Negtive_Asset = 100;
% WiseUp = 0;

      % how many time steps to execute the online planner

Creat_Environment_Visbility_Data
load('Save_Visibility_Data\M_starstar12.mat')

Lookahead = 4;
T = Lookahead;

T_execution = 20; 

V{1} = visibility_polygon( [Initial_Agent(1) Initial_Agent(2)] , environment , epsilon, snap_distance);
Initial_Agent_Region = poly2mask(V{1}(:,1),V{1}(:,2),ENV_SIZE1, ENV_SIZE2);

Number_of_Asset = size(Asset,1);
Number_of_Function = 0;
for i = 0:Number_of_Asset
    Number_of_Function = Number_of_Function + nchoosek(Number_of_Asset,i);
end
Function_index = dec2bin(Number_of_Function-1);
Function_index_size = size(Function_index,2);


for step = 1:T_execution
    
    %% Build the tree
    Tree = BuildMinimaxTree_BF2(Initial_Agent,Initial_Opponent,Initial_Agent_Region,Asset,...
            Detection_Asset_Collect,environment,Lookahead,Negtive_Reward,Negtive_Asset,Visibility_Data,Region,Asset_Visibility_Data,Visibility_in_environment,step);
    %% Run the DM1 One Pass to back propagate the reward values
    %Change RunDM1 to RunLeafLookAhed or RunMinimax_multi_assets to run
    %other algorithms
    [Initial_Agent,Initial_Opponent,Initial_Agent_Region,Assets_Collected] = ...
        RunDM1(Tree,T,Asset,Negtive_Reward,Negtive_Asset,Number_of_Function,Function_index_size,Visibility_Data,Region,Asset_Visibility_Data,step);
    %% Record the action for next step, also record the assets collected realdy
    Record_path_Agent(:,step + 1) = Initial_Agent;
    Record_path_Opponent(:,step + 1) = Initial_Opponent;
    Detection_Asset_Collect = Assets_Collected;
end

%%
save('Online.mat')

%%
Plot_Path_DM1