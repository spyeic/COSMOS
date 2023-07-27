#include <stdlib.h>
#include "../matrix_functions.h"

Mat * create_H(int N)
{
    Mat * H;
    makeMatrix(N, N, H);
    for (size_t i = 0; i < N; i++)
    {
        for (size_t j = 0; j < N; j++)
        {
            if (i == j)
            {
                H->Matrix[i][j] = -13.6;
            }
            if (i == j + 1 || i == j - 1)
            {
                H->Matrix[i][j] = 0.5;
            }
        }
        
    }
    printMatrix(H);
    return H;
}

int main()
{
    create_H(5);
}