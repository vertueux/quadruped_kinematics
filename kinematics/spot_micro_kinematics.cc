// Copyright (c) Virtuous. Licensed under the MIT license.
// See LICENSE.md in the project root for license information.

#include "kinematics/spot_micro_kinematics.h"
#include <math.h>

namespace sms {

template<typename> struct is_std_vector :  false_type {};
template<typename T, typename A> struct is_std_vector<vector<T, A>> : true_type {};
template <typename T>
enable_if_t<is_std_vector<decay_t<T>>::value, T>
operator+(T&& vec1, T&& vec2) {
	for (size_t i = 0; i < vec2.size(); ++i) {
    for (size_t j = 0; j < vec2[i].size(); j++) {
      vec1[i][j] += vec2[i][j];
    }
  }
	return vec1;
}

vector<vector<vector<vector<double>>>> SpotMicroKinematics::body_kinematics(double omega, double phi, double psi, double xm, double ym, double zm) {
  vector<vector<double>> rx = {{1, 0, 0, 0}, {0, cos(omega), -sin(omega), 0}, {0, sin(omega), cos(omega), 0}, {0, 0, 0, 1}};
  vector<vector<double>> ry = {{cos(phi), 0, sin(phi), 0}, {0, 1, 0, 0}, {-sin(phi), 0, cos(phi), 0}, {0, 0, 0, 1}};
  vector<vector<double>> rz = {{cos(psi), -sin(psi), 0, 0}, {sin(psi), cos(psi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  vector<vector<double>> _rxyz = Matrix::dot_product(ry, rz);
  vector<vector<double>> rxyz = Matrix::dot_product(rx, _rxyz);
  
  vector<vector<double>> t{{0, 0, 0, xm}, {0, 0, 0, ym}, {0, 0, 0, zm}, {0, 0, 0, 0}};
  vector<vector<double>> tm = t + rxyz;

  double s_hp = sin(3.141592653589793 / 2);
  double c_hp = cos(3.141592653589793 / 2);
  double _L = this->L, _W = this->W;

  vector<vector<double>> _tm1{{c_hp, 0, s_hp, _L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, _W / 2}, {0, 0, 0, 1}};
  vector<vector<double>> _tm2{{c_hp, 0, s_hp, _L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, -_W / 2}, {0, 0, 0, 1}};
  vector<vector<double>> _tm3{{c_hp, 0, s_hp, -_L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, _W / 2}, {0, 0, 0, 1}};
  vector<vector<double>> _tm4{{c_hp, 0, s_hp, -_L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, -_W / 2}, {0, 0, 0, 1}};

  vector<vector<double>> new_tm1_rslt = Matrix::dot_product(tm, _tm1);
  vector<vector<double>> new_tm2_rslt = Matrix::dot_product(tm, _tm2);
  vector<vector<double>> new_tm3_rslt = Matrix::dot_product(tm, _tm3);
  vector<vector<double>> new_tm4_rslt = Matrix::dot_product(tm, _tm4);

  vector<vector<vector<vector<double>>>> final_result = {{new_tm1_rslt}, {new_tm2_rslt}, {new_tm3_rslt}, {new_tm4_rslt}};
  
  _tm1.clear();
  _tm2.clear();
  _tm3.clear();
  _tm4.clear();

  return final_result;
}

vector<double> SpotMicroKinematics::leg_kinematics(vector<double> point) {
  double x = point[0], y = point[1], z = point[2];

  double f = sqrt(pow(x, 2) + pow(y, 2) - pow(l1, 2));
  double g = f - l2;
  double h = sqrt(pow(g, 2) + pow(z, 2));

  double theta1 = atan2(y, x) - atan2(f, -l1);

  double d = (pow(h, 2) - pow(l3, 2) - pow(l4, 2)) / (2 * l3 * l4);

  double theta3 = acos(d);
  double theta2 = atan2(z, g) - atan2(l4 * sin(theta3), l3 + l4 * cos(theta3));

  return vector<double> {theta1, theta2, theta3};
}

vector<double> SpotMicroKinematics::calculate_leg_points(double angles) {
  double _l1 = l1, _l2 = l2, _l3 = l3, _l4 = l4; 
  double theta1, theta2, theta3;
  (theta1, theta2, theta3) = angles;

  double theta23 = theta2 + theta3;

  vector<double> t0 = {0, 0, 0, 1};

  vector<double> _t0 = {-_l1 * cos(theta1), _l1 * sin(theta1), 0, 0};
  vector<double> t1 = t0 + _t0;
  _t0.clear();

  vector<double> _t1 = {-_l2 * sin(theta1), -_l2 * cos(theta1), 0, 0};
  vector<double> t2 = t1 + _t1;
  _t1.clear();
  
  vector<double> _t2 = {-_l3 * sin(theta1) * cos(theta2), -_l3 * cos(theta1) * cos(theta2), _l3 * sin(theta2), 0};
  vector<double> t3 = t2 + _t2;
  _t2.clear();

  vector<double> _t3 = {-_l4 * sin(theta1) * cos(theta23), -_l4 * cos(theta1) * cos(theta23), _l4 * sin(theta23), 0};
  vector<double> t4 = t3 + _t3;
  _t3.clear();

  vector<double> final_result;
  for (int i = 0; i < t0.size(); ++i)
    final_result.push_back(t0.at(i));

  for (int i = 0; i < t1.size(); ++i) 
    final_result.push_back(t1.at(i));

  for (int i = 0; i < t2.size(); ++i) 
    final_result.push_back(t2.at(i));

  for (int i = 0; i < t3.size(); ++i) 
    final_result.push_back(t3.at(i));
  
  for (int i = 0; i < t4.size(); ++i) 
    final_result.push_back(t4.at(i));

  return final_result;
}

vector<vector<vector<vector<double>>>> SpotMicroKinematics::calculate_kinematics(vector<double> lp, double angles, double center) {
  double omega = angles, phi = angles, psi = angles;
  double xm = center, ym = center, zm = center;

  vector<vector<vector<vector<double>>>> t_lf = body_kinematics(omega, phi, psi, xm, ym, zm);
  vector<vector<vector<vector<double>>>> t_rf = body_kinematics(omega, phi, psi, xm, ym, zm);
  vector<vector<vector<vector<double>>>> t_lb = body_kinematics(omega, phi, psi, xm, ym, zm);
  vector<vector<vector<vector<double>>>> t_rb = body_kinematics(omega, phi, psi, xm, ym, zm);

  vector<vector<double>> ix = {{-1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

}


}

