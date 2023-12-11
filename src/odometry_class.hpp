#include <iostream>
#include <cmath>

struct pose2D {
    // ロボットの姿勢
    double x;        // ロボットのx座標[m]
    double y;        // ロボットのy座標[m]
    double theta;    // ロボットの向き[rad]
};

struct deltaPose2D {
    // ロボットの微小変位
    double x;      // ロボットのx方向微小変位
    double y;      // ロボットのy方向微小変位
    double theta;  // ロボットの角度微小変位
};

class Odometry {
private:
    const double radius;             // 車輪半径[m]
    const double length;             // タイヤの配置半径[m]
    pose2D pose;                    // 姿勢
    double tire_angles[3] = {0.0};        // エンコーダの角変位

public:
    // コンストラクタ
    Odometry(double _radius, double _len, double _x0, double _y0, double _theta0)
            : radius(_radius), length(_len), pose({_x0, _y0, _theta0}) {}

    // メンバ関数
    deltaPose2D calc_vel(double _tire_angles[3]);        // 微小変位を計算する関数
    void calc_pose(deltaPose2D delta_pose);   // 位置の積分
    void update_angles(double _tire_angles[3]);         // カウントの更新
    void update_pose(double _tire_angles[3]);           // 位置の更新
    void set_pose(pose2D pose0);             // ロボット姿勢のセッター
    pose2D get_pose();
};