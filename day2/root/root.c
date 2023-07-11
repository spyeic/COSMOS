#include <stdio.h>
#include <math.h>

int main() {
    int j;
    double x, A;
    printf("Enter x:   ");
    scanf("%lf", &x);
    printf("Enter A:   ");
    scanf("%lf", &A);
    for (j = 0; j < 20; j++) {
        x = x / 2. + A / (2. * x);
        printf("%5i %12.8lf\n", j, x);
    }
    return 0;
}