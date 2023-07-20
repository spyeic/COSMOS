#include <stdio.h>

int main()
{
    const int N = 3;
    double M[N][N];
    double v[N];

    printf("enter matrix for m:");

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            scanf("%lf", &M[i][j]);
        }
    }

    printf("enter matrix for v:");

    for (int i = 0; i < N; i++)
    {
        scanf("%lf", &v[i]);
    }
}