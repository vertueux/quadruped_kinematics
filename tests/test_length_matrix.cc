#include <iostream>
#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
using namespace std;

double l1 = 50, l2 = 20, l3 = 120, l4 = 155, L = 140, W = 75;

double get_determinant(const vector<vector<double>> vect) {
    if(vect.size() != vect[0].size()) {
        throw runtime_error("Matrix is not quadratic");
    } 
    int dimension = vect.size();

    if(dimension == 0) {
        return 1;
    }

    if(dimension == 1) {
        return vect[0][0];
    }

    //Formula for 2x2-matrix
    if(dimension == 2) {
        return vect[0][0] * vect[1][1] - vect[0][1] * vect[1][0];
    }

    double result = 0;
    int sign = 1;
    for(int i = 0; i < dimension; i++) {

        //Submatrix
        vector<vector<double>> subVect(dimension - 1, vector<double> (dimension - 1));
        for(int m = 1; m < dimension; m++) {
            int z = 0;
            for(int n = 0; n < dimension; n++) {
                if(n != i) {
                    subVect[m-1][z] = vect[m][n];
                    z++;
                }
            }
        }

        //recursive call
        result = result + sign * vect[0][i] * get_determinant(subVect);
        sign = -sign;
    }

    return result;
}

vector<vector<double>> get_transpose(const vector<vector<double>> matrix1) {

    //Transpose-matrix: height = width(matrix), width = height(matrix)
    vector<vector<double>> solution(matrix1[0].size(), vector<double> (matrix1.size()));

    //Filling solution-matrix
    for(size_t i = 0; i < matrix1.size(); i++) {
        for(size_t j = 0; j < matrix1[0].size(); j++) {
            solution[j][i] = matrix1[i][j];
        }
    }
    return solution;
}

vector<vector<double>> get_co_factor(const vector<vector<double>> vect) {
    if(vect.size() != vect[0].size()) {
        throw runtime_error("Matrix is not quadratic");
    } 

    vector<vector<double>> solution(vect.size(), vector<double> (vect.size()));
    vector<vector<double>> subVect(vect.size() - 1, vector<double> (vect.size() - 1));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect[0].size(); j++) {

            int p = 0;
            for(size_t x = 0; x < vect.size(); x++) {
                if(x == i) {
                    continue;
                }
                int q = 0;

                for(size_t y = 0; y < vect.size(); y++) {
                    if(y == j) {
                        continue;
                    }

                    subVect[p][q] = vect[x][y];
                    q++;
                }
                p++;
            }
            solution[i][j] = pow(-1, i + j) * get_determinant(subVect);
        }
    }
    return solution;
}

vector<vector<double>> get_inverse(const vector<vector<double>> vect) {
    if(get_determinant(vect) == 0) {
        throw runtime_error("Determinant is 0");
    } 

    double d = 1.0/get_determinant(vect);
    vector<vector<double>> solution(vect.size(), vector<double> (vect.size()));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] = vect[i][j]; 
        }
    }

    solution = get_transpose(get_co_factor(solution));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] *= d;
        }
    }

    return solution;
}

void print_matrix(const vector<vector<double>> vect) {
    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect[0].size(); j++) {
            cout << setw(8) << vect[i][j] << " ";
        }
        cout << "\n";
    }
}

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
    throw invalid_argument("Dot product not compatible.");

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

vector<double> dim_flat (vector<double> const & v)
 { return v; }

template <typename T>
vector<double> dim_flat(vector<vector<T>> const & v)
 {
   vector<double>  ret;

   for ( auto const & e : v )
    {
      auto s = dim_flat(e);

      ret.reserve( ret.size() + s.size() );
      ret.insert( ret.end(), s.cbegin(), s.cend() );
    }

   return ret;
 }

int main() {
  vector<vector<vector<vector<double>>>> a = body_kinematics(40.5, 40.5, 40.5, 40.5, 40.5, 40.5);
  vector<double> b = dim_flat(a);

  /*for(size_t i = 0; i < a.size(); i++) {
    for(size_t j = 0; j < a[0].size(); j++) {
      for (size_t h = 0; h < a[0][0].size(); h++) {
        for (size_t k = 0; k < a[0][0][0].size(); k++) {
          cout << setw(8) << a[i][j][h][k] << " ";
        }
        cout << "\n";
      }
      cout << "\n";
    }
    cout << "\n";
  }
  cout << "\n";*/

  for (size_t i = 0; i < b.size(); i++) 
    cout << b[i] << endl;
  
  return 0;
}