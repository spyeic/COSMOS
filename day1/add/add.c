#include <stdio.h>
#include <math.h>

int main() {
  double x, y, z;
  printf("Enter x: ");
  scanf("%lf", &x);
  printf("Enter y: ");
  scanf("%lf", &y);
  z = x + y;
  printf("x + y = %12.2lf \n", z);
}
