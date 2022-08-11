#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <iostream>
using namespace std;

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

int main() {
  vector<vector<double>> a = {{1, 1, 1, 1}, {2, 2, 2, 2}, {3, 3, 3, 3}, {4, 4, 4, 4}};
  vector<vector<double>> b = {{5, 5, 5, 5}, {6, 6, 6, 6}, {7, 7, 7, 7}, {8, 8, 8, 8}};
  vector<vector<double>> vect = a + b;

  for(std::size_t i = 0; i < vect.size(); i++) {
    for(std::size_t j = 0; j < vect[0].size(); j++) {
      std::cout << std::setw(8) << vect[i][j] << " ";
    }
    std::cout << "\n";
  }
}