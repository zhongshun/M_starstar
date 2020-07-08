function val = them_eval_fn(posreward,agent_detected,teammate_detected,Negtive_Reward,Negtive_Teammate,Pr)

val = posreward - Negtive_Reward*agent_detected;