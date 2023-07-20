#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main()
{
    int N;
    double beta, x, t, dt, v, F, m;

    printf("Enter beta: ");
    scanf("%lf", &beta);
    printf("Enter x: ");
    scanf("%lf", &x);
    printf("Enter N: ");
    scanf("%d", &N);
    printf("Enter dt: ");
    scanf("%lf", &dt);
    printf("Enter initial speed v: ");
    scanf("%lf", &v);
    printf("Enter mass m: ");
    scanf("%lf", &m);

    FILE * output = fopen("spring.data", "w");

    for (int i = 0; i < N; i++)
    {
        t = t + dt;
        x = x + v * dt;
        F = -beta * pow(x, 3);
        v = v + (F / m) * dt;
        fprintf(output, "\n   %12.6lf %12.6lf", t, x);
    }
    
    return 0;
}