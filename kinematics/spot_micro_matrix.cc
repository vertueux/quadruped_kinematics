// Copyright (c) Virtuous. Licensed under the MIT license.
// See LICENSE.md in the project root for license information.

#include "kinematics/spot_micro_matrix.h"

#include <vector>

namespace sms {

vector<vector<double>> Matrix::dot_product(vector<vector<double>> m1, vector<vector<double>> m2) {
  vector<vector<double>> m;
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      double result = 0;
      for (int k = 0; k < N; k++) {
        result += m1[i][k] * m2[k][j];
      }
      m[i][j] = result;
    }
  }
  return m;
}

}