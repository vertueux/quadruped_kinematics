#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <iostream>
using namespace std;

double l1 = 50, l2 = 20, l3 = 120, l4 = 155;

template<typename> struct is_std_vector :  false_type {};
template<typename T, typename A> struct is_std_vector<vector<T, A>> : true_type {};
template <typename T>
enable_if_t<is_std_vector<decay_t<T>>::value, T>
operator+(T&& vec1, T&& vec2) {
	for (size_t i = 0; i < vec2.size(); ++i) {
        vec1[i] += vec2[i];
    }
	return vec1;
}

vector<double> calculate_leg_points(vector<double> angles) {
  double _l1 = l1, _l2 = l2, _l3 = l3, _l4 = l4; 
  double theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];
  double theta23 = theta2 + theta3;
  vector<double> final_result;

  vector<double> t0 = {0, 0, 0, 1};

  for (int i = 0; i < t0.size(); i++)
    final_result.push_back(t0.at(i));

  vector<double> _t0 = {-_l1 * cos(theta1), _l1 * sin(theta1), 0, 0};
  vector<double> t1 = t0 + _t0;
  _t0.clear();

  for (int j = 0; j < t1.size(); j++) 
    final_result.push_back(t1.at(j));

  vector<double> _t1 = {-_l2 * sin(theta1), -_l2 * cos(theta1), 0, 0};
  vector<double> t2 = t1 + _t1;
  _t1.clear();

  for (int h = 0; h < t2.size(); h++) 
    final_result.push_back(t2.at(h));
  
  vector<double> _t2 = {-_l3 * sin(theta1) * cos(theta2), -_l3 * cos(theta1) * cos(theta2), _l3 * sin(theta2), 0};
  vector<double> t3 = t2 + _t2;
  _t2.clear();

  for (int k = 0; k < t3.size(); k++) 
    final_result.push_back(t3.at(k));

  vector<double> _t3 = {-_l4 * sin(theta1) * cos(theta23), -_l4 * cos(theta1) * cos(theta23), _l4 * sin(theta23), 0};
  vector<double> t4 = t3 + _t3;
  _t3.clear();
  
  for (int l = 0; l < t4.size(); l++) 
    final_result.push_back(t4.at(l));

  return final_result;
}

int main() {
  vector<double> a = {90, 45, 30};
  vector<double> b = calculate_leg_points(a);

  for(size_t i = 0; i < b.size(); i++) { 
    cout << b[i] << endl;
  }
}