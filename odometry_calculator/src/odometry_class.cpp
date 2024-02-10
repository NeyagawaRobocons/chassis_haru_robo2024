#include <iostream>
#include <cmath>
#include "odometry_class.hpp"

using namespace std;

deltaPose2D Odometry::calc_vel(double _tire_angles[3]) {
    // 速度の計算
    double delta_angle[3];
    for (int i=0; i < 3; i++) {
        // カウントの差分から計測輪の微小変位を計算
        delta_angle[i] = _tire_angles[i] - tire_angles[i];
    }
    deltaPose2D delta_pose;
    // 正三角形の配置の場合
    // double delta_x = radius * 2.0 / 3.0 * (-delta_angle[0] + delta_angle[1] / 2.0 + delta_angle[2] / 2.0);
    // double delta_y = radius * 2.0 / 3.0 * (-delta_angle[1] + delta_angle[2]) * 0.866025403784;
    // 四角形の配置の場合
    double delta_x = - radius * delta_angle[1];
    double delta_y = radius * (delta_angle[0] - delta_angle[2]) / 2.0;
    // ロボットの向きに合わせて速度ベクトルを回転
    delta_pose.x = delta_x * cos(pose.theta) - delta_y * sin(pose.theta);
    delta_pose.y = delta_x * sin(pose.theta) + delta_y * cos(pose.theta);
    delta_pose.theta = radius * (delta_angle[0] + delta_angle[1] + delta_angle[2]) / 3.0 / length;
    update_angles(_tire_angles); // 前回カウントの更新
    return delta_pose;
}

void Odometry::calc_pose(deltaPose2D delta_pose) {
    // 姿勢の積分
    pose.x += delta_pose.x;
    pose.y += delta_pose.y;
    pose.theta += delta_pose.theta;
    // pose.theta = fmod(pose.theta + M_PI, 2 * M_PI) - M_PI;
}

void Odometry::update_angles(double _tire_angles[3]) {
    // 前回のタイヤ角変位を更新
    for (int i=0; i<3; i++) {
        tire_angles[i] = _tire_angles[i];
    }
}

void Odometry::update_pose(double _tire_angles[3]) {
    // 位置の更新
    calc_pose(calc_vel(_tire_angles));
}

void Odometry::set_pose(pose2D pose0) {
    // 姿勢をセットする
    pose.x = pose0.x;
    pose.y = pose0.y;
    pose.theta = pose0.theta;
}

pose2D Odometry::get_pose() {
    return pose;
}