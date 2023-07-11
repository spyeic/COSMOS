#include <stdio.h>
#include <math.h>
#include <stdlib.h>

int main()
{
    double R;
    int i, N;
    unsigned int seed;

    printf("\nEnter number of iteration and seed\n");
    scanf("%i %u", &N, &seed);
    for (i = 0; i < N; i++)
    {
        R = (double) rand() / RAND_MAX;
        printf("%12.8lf \n", R);
    }
    return 0;
}