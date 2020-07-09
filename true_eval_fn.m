function val = true_eval_fn(posreward,agent_detected,Teammate_Detection_time_E_smart,Negtive_Reward,Negtive_Teammate,Pr)

val = posreward - Negtive_Reward*agent_detected - Negtive_Teammate*Teammate_Detection_time_E_smart;