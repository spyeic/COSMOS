#include <math.h>
#include <stdlib.h>
#include <stdio.h>

int main()
{
    int N = 500;

    double sum;
    for (int i = 1; i <= 5; i++)
    {
        sum = 0;
        for (int j = 0; j < N; j++)
        {
            sum += pow((double)rand() / RAND_MAX, i);
        }
        printf("%lf\n", sum / N);
    }

    return 0;
}