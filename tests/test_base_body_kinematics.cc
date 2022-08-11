#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <iostream>
using namespace std;

double omega = 40.5; 
double phi = 40.5;
double psi = 40.5; 
double xm = 40.5; 
double ym = 40.5; 
double zm = 40.5;

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

int main() {
  vector<vector<double>> rx = {{1, 0, 0, 0}, {0, cos(omega), -sin(omega), 0}, {0, sin(omega), cos(omega), 0}, {0, 0, 0, 1}};
  vector<vector<double>> ry = {{cos(phi), 0, sin(phi), 0}, {0, 1, 0, 0}, {-sin(phi), 0, cos(phi), 0}, {0, 0, 0, 1}};
  vector<vector<double>> rz = {{cos(psi), -sin(psi), 0, 0}, {sin(psi), cos(psi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  vector<vector<double>> _rxyz = dot_product(ry, rz);
  vector<vector<double>> rxyz = dot_product(rx, _rxyz);

  vector<vector<double>> t{{0, 0, 0, xm}, {0, 0, 0, ym}, {0, 0, 0, zm}, {0, 0, 0, 0}};
  vector<vector<double>> tm = t + rxyz;

  for(std::size_t i = 0; i < tm.size(); i++) {
    for(std::size_t j = 0; j < tm[0].size(); j++) {
      std::cout << std::setw(8) << tm[i][j] << " ";
    }
    std::cout << "\n";
  }
}