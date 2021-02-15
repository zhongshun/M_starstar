clear all; 
clc;
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

% Initial_Agent = [12;9];
% Initial_Opponent = [12;8];

Initial_Agent = [17;7];
Initial_Opponent = [17;6];

Asset = [4 7; 16 10;17 10;16 4; 17 4];
% Asset = [15 14; 17 12; 15 4; 7 6];
Number_of_Asset = size(Asset,1);


%The frequency that the teammate appear
Record_path_Agent = Initial_Agent;
Record_path_Opponent = Initial_Opponent;

% Teammate_appear_mod = 3;
% Teammate_appear_mod_E_smart = 3;
Detection_Asset_Collect = zeros(Number_of_Asset,1);

Negtive_Reward = 1;
Negtive_Asset = 30;
% WiseUp = 0;

      % how many time steps to execute the online planner
Resolution = 10;

Creat_Environment_Visbility_Data
% load('Save_Visibility_Data\M_starstar12.mat')

Lookahead = 4;
T = Lookahead;

T_execution = 10; 
Discount_factor =0.85;


V{1} = visibility_polygon( [Initial_Agent(1) Initial_Agent(2)] , environment , epsilon, snap_distance);
Initial_Agent_Region = poly2mask(Resolution*V{1}(:,1),Resolution*V{1}(:,2),Resolution*ENV_SIZE1, Resolution*ENV_SIZE2);
% Initial_Agent_Region = poly2mask(V{1}(:,1),V{1}(:,2),11, 24);
Number_of_Asset = size(Asset,1);
Number_of_Function = 0;
for i = 0:Number_of_Asset
    Number_of_Function = Number_of_Function + nchoosek(Number_of_Asset,i);
end
Function_index = dec2bin(Number_of_Function-1);
Function_index_size = size(Function_index,2);
save('Save_Visibility_Data\Initial.mat')

for step = 1:T_execution
    
    %% Build the tree
% %   
%     if T_execution - step + 1  <= Lookahead
%         Lookahead = T_execution - step + 1;
%         T = Lookahead;
%     end
%     

    Tree_Agent = BuildMinimaxTree_BF2(Initial_Agent,Initial_Opponent,Initial_Agent_Region,Asset,...
            Detection_Asset_Collect,environment,Lookahead,Negtive_Reward,Negtive_Asset,Visibility_Data,Region,Asset_Visibility_Data,Visibility_in_environment,step,Resolution,Discount_factor);
    %% Run the DM1 One Pass to back propagate the reward values
    %Change RunDM1 to RunLeafLookAhed or RunMinimax_multi_assets to run
    %other algorithms
    
    [Initial_Agent_update,Initial_Opponent1,Initial_Agent_Region_update,Assets_Collected_agent] = ...
        RunDM1(Tree_Agent,T,Asset,Negtive_Reward,Negtive_Asset,Number_of_Function,Function_index_size,Visibility_Data,Region,Asset_Visibility_Data,step,Discount_factor);
    clear Tree_Agent;
%     
    %% Build the tree for the opponent 
    Tree_Opponent = BuildMinimaxTree_BF(Initial_Agent,Initial_Opponent,Initial_Agent_Region,Asset,...
            Detection_Asset_Collect,environment,Lookahead,Negtive_Reward,Negtive_Asset,Visibility_Data,Region,Asset_Visibility_Data,Visibility_in_environment,step,Resolution,Discount_factor);
    
    %% Run the Minimax
    [Initial_Agent1,Initial_Opponent_update,Initial_Agent_Region_opponent,Assets_Collected] = ...
        RunMinimax(Tree_Opponent,T,Asset,Negtive_Reward,Negtive_Asset,Number_of_Function,Function_index_size,Visibility_Data,Region,Asset_Visibility_Data,step,Discount_factor);
    clear Tree_Opponent;

    %% Record the action for next step, also record the assets collected realdy
    Record_path_Agent(:,step + 1) = Initial_Agent_update;
    Record_path_Opponent(:,step + 1) = Initial_Opponent_update;
    Initial_Agent = Initial_Agent_update;
    Initial_Opponent = Initial_Opponent_update;
    Initial_Agent_Region = Initial_Agent_Region_update;
    Detection_Asset_Collect = Assets_Collected;

end

%%
save('Online.mat')

%%
Plot_Path_DM1