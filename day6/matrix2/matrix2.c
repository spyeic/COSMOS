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

    printf("enter matrix for v:\n");

    for (int i = 0; i < N; i++)
    {
        scanf("%lf", &v[i]);
    }

    printf("Added result: \n");
    double M_added[N][N];
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            M_added[i][j] = M[i][j] * 2;
            printf("%lf ", M_added[i][j]);
        }
        printf("\n");
    }

    printf("Multiply M by v\n");
    double Mv[N];
    for (int i = 0; i < N; i++)
    {
        double sum = 0;
        for (int j = 0; j < N; j++)
        {
            sum += v[j] * M[i][j];
        }
        Mv[i] = sum;
        printf("%lf ", sum);
    }

    printf("\n");
    return 0;
}