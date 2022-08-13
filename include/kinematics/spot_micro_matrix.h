// Copyright (c) Virtuous. Licensed under the MIT license.
// See LICENSE.md in the project root for license information.

#ifndef SPOT_MICRO_MATRIX_H
#define SPOT_MICRO_MATRIX_H

#define N 4

#include <vector>
using namespace std;

namespace spot_micro2 {

class Matrix {
 public:
  static vector<vector<double>> dot_product(vector<vector<double>> m1, vector<vector<double>> m2);
  static vector<vector<double>> get_inverse(const vector<vector<double>> vect); // Only works with 2x2 matrices for now.
};

}
#endif // SPOT_MICRO_MATRIX_H