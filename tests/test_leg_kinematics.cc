#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <iostream>
using namespace std;

double l1 = 50, l2 = 20, l3 = 120, l4 = 155;

vector<double> leg_kinematics(vector<double> point) {
  double x = point[0], y = point[1], z = point[2];
  double _l1 = l1, _l2 = l2,_l3 = l3,_l4 = l4;
  double f = 0;
  double theta3 = 0;

  if(isnan(sqrt(pow(x, 2) + pow(y, 2) - pow(_l1, 2)))) {
    cout << "NAN in the leg kinematics with x: " << x << ", y: " << y << " and l1: " << _l1 << "\n";
    f = _l1;
  } else {
    f = sqrt(pow(x, 2) + pow(y, 2) - pow(l1, 2));
  }

  double g = f - _l2;
  double h = sqrt(pow(g, 2) + pow(z, 2));
  cout << h << endl;

  double theta1 =- atan2(y, x) - atan2(f, -_l1);
  cout << theta1 << endl;

  double d = ((pow(h, 2) - pow(_l3, 2) - pow(_l4, 2)) / (2 * _l3 * _l4));
  cout << d << endl;

  if (isnan(acos(d))) {
    cout << "NAN in the leg kinematics with x: " << x << ", y: " << y << " and d: " << d << "\n";
    theta3 = 0;
  } else {
    theta3 = acos(d);
  }
  cout << theta3 << endl;
  double theta2 = atan2(z, g) - atan2(_l4 * sin(theta3), _l3 + _l4 * cos(theta3));

  return vector<double> {theta1, theta2, theta3};
}

int main() {
  vector<double> a = {1200, 1200, 1200};
  vector<double> b = leg_kinematics(a);

  for(size_t i = 0; i < b.size(); i++) { 
    cout << b[i] << " ";
  }
  cout << "\n";
}