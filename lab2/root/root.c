#include <stdio.h>
#include <math.h>

int main()
{
    int j, n;
    double x, A;
    printf("Enter x:   ");
    scanf("%lf", &x);
    printf("Enter A:   ");
    scanf("%lf", &A);
    printf("Attempts:   ");
    scanf("%d", &n);
    int prev;
    for (j = 0; j < n; j++)
    {
        prev = x;
        x = x / 2. + A / (2. * x);
        if (x == prev)
        {
            break;
        }
        printf("%5i %12.8lf\n", j, x);
    }
    return 0;
}