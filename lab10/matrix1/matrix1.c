#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    const int N = 3;
    double M[N][N] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    double V[N] = {1, 2, 3};

    printf("%lf\n", M[2][1]);

    printf("%lf + %lf = %lf\n", M[2][2], M[1][2], M[2][2] + M[1][2]);
    return 0;
}