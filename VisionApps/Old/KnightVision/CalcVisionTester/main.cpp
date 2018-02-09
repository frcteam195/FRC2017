#include <iostream>
#include <math.h>

using namespace std;

double d2r(double degrees) {
	return degrees * (M_PI / 180);
}

double y = 99;
double Td = 110.5;
double c = 48;
double v = 209.52;	//3670RPMs on wheel
double yo = 21.08;

int main() {
	for(double r = 24; r < 66; r+= 0.1) {
		double rhs = 4.9*pow(Td*cos(d2r(c))/(v*cos(d2r(r))), 2)+tan(d2r(r))*Td*cos(d2r(c)) + yo;
		if (rhs < (y+1) && rhs > (y-1))
			cout << r << endl;
		cout << rhs << endl;
	}
	
	return 0;
}
