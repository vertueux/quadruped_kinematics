#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <iostream>
using namespace std;


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
  vector<vector<double>> a = {{1, 1, 1, 1}, {2, 2, 2, 2}, {3, 3, 3, 3}, {4, 4, 4, 4}};
  vector<vector<double>> b = {{5, 5, 5, 5}, {6, 6, 6, 6}, {7, 7, 7, 7}, {8, 8, 8, 8}};
  vector<vector<double>> c = {{5, 5, 5, 5}, {6, 6, 6, 6}, {7, 7, 7, 7}, {8, 8, 8, 8}};
  vector<vector<double>> vect = dot_product(b, c);
  vector<vector<double>> vect2 = dot_product(a, vect);


  for(std::size_t i = 0; i < vect2.size(); i++) {
    for(std::size_t j = 0; j < vect2[0].size(); j++) {
      std::cout << std::setw(8) << vect2[i][j] << " ";
    }
    std::cout << "\n";
  }
}