# Pose2DWithSpeed.msg

float64 x # [m]
float64 y # [m]
float64 theta # [rad]
float64 speed # [m/s]
float64 lookahead_distance # [m]
float64 angle_p_gain # 経路の各点で変数
float64 angle_i_gain # 経路の各点で変数
float64 angle_threshold # 経路の先頭に格納された定数
float64 path_p_gain
float64 path_i_gain