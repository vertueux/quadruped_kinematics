#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <iostream>
using namespace std;

double l1 = 50, l2 = 20, l3 = 120, l4 = 155, L = 140, W = 75;

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

vector<vector<double>> dot_product(const vector<vector<double>>& m1, const vector<vector<double>>& m2) {
  if(!(m1.size() == m2.size()))
    throw std::invalid_argument("Dot product not compatible.");

  vector<vector<double>> m = {{0, 0, 0, 0,}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
  double result = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      for (int h = 0; h < 4; h++) {
        result += m1[i][h] * m2[h][j];
      }
      m[i][j] = result;
      result = 0;
    }
  }
  return m;
}

vector<vector<vector<vector<double>>>> body_kinematics(double omega, double phi, double psi, double xm, double ym, double zm) {
  vector<vector<double>> rx = {{1, 0, 0, 0}, {0, cos(omega), -sin(omega), 0}, {0, sin(omega), cos(omega), 0}, {0, 0, 0, 1}};
  vector<vector<double>> ry = {{cos(phi), 0, sin(phi), 0}, {0, 1, 0, 0}, {-sin(phi), 0, cos(phi), 0}, {0, 0, 0, 1}};
  vector<vector<double>> rz = {{cos(psi), -sin(psi), 0, 0}, {sin(psi), cos(psi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  vector<vector<double>> _rxyz = dot_product(ry, rz);
  vector<vector<double>> rxyz = dot_product(rx, _rxyz);
  
  vector<vector<double>> t{{0, 0, 0, xm}, {0, 0, 0, ym}, {0, 0, 0, zm}, {0, 0, 0, 0}};
  vector<vector<double>> tm = t + rxyz;

  double s_hp = sin(3.141592653589793 / 2);
  double c_hp = cos(3.141592653589793 / 2);
  double _L = L, _W = W;

  vector<vector<double>> _tm1{{c_hp, 0, s_hp, _L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, _W / 2}, {0, 0, 0, 1}};
  vector<vector<double>> _tm2{{c_hp, 0, s_hp, _L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, -_W / 2}, {0, 0, 0, 1}};
  vector<vector<double>> _tm3{{c_hp, 0, s_hp, -_L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, _W / 2}, {0, 0, 0, 1}};
  vector<vector<double>> _tm4{{c_hp, 0, s_hp, -_L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, -_W / 2}, {0, 0, 0, 1}};

  vector<vector<double>> new_tm1_rslt = dot_product(tm, _tm1);
  vector<vector<double>> new_tm2_rslt = dot_product(tm, _tm2);
  vector<vector<double>> new_tm3_rslt = dot_product(tm, _tm3);
  vector<vector<double>> new_tm4_rslt = dot_product(tm, _tm4);

  vector<vector<vector<vector<double>>>> final_result = {{new_tm1_rslt}, {new_tm2_rslt}, {new_tm3_rslt}, {new_tm4_rslt}};
  
  _tm1.clear();
  _tm2.clear();
  _tm3.clear();
  _tm4.clear();

  return final_result;
}

int main() {
  vector<vector<vector<vector<double>>>> a = body_kinematics(40.5, 40.5, 40.5, 40.5, 40.5, 40.5);

  for(std::size_t i = 0; i < a.size(); i++) {
    for(std::size_t j = 0; j < a[0].size(); j++) {
      for (std::size_t h = 0; h < a[0][0].size(); h++) {
        for (std::size_t k = 0; k < a[0][0][0].size(); k++) {
          std::cout << std::setw(8) << a[i][j][h][k] << " ";
        }
        std::cout << "\n";
      }
      std::cout << "\n";
    }
    std::cout << "\n";
  }

}