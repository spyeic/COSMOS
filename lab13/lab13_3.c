#include <stdlib.h>
#include <stdio.h>
#include "../matrix_functions.h"

#define E0 -13.6
#define E0A -13.6
#define E0B -12.0
#define Ehop 0.5

void create_H(Mat *H)
{
    for (int i = 0; i < H->N; i++)
    {
        for (int j = 0; j < H->M; j++)
        {
            if (i == j)
            {
                if (j % 2 == 0)
                {
                    H->Matrix[i][j] = E0A;
                }
                else
                {
                    H->Matrix[i][j] = E0B;
                }
            }
            if (i == j + 1 || i == j - 1)
            {
                H->Matrix[i][j] = Ehop;
            }
        }
    }
    H->Matrix[0][H->M - 1] = Ehop;
    H->Matrix[H->N - 1][0] = Ehop;
}

int main()
{
    int N = 32;
    int i, j;
    Mat testmat, vecs;
    Vec vals_real, vals_im;

    makeMatrix(N, N, &testmat);
    makeMatrix(N, N, &vecs);
    makeVector(N, &vals_real);
    makeVector(N, &vals_im);

    create_H(&testmat);

    get_eigenvalues(&testmat, &vals_real, &vals_im, &vecs, 1);

    //  PRINT TO SCREEN:

    printf("\n Original Matrix being diagonalized :\n");
    printMatrix(&testmat);

    printf("\n Real part of Eigenvalues :\n");
    printVector(&vals_real);

    printf("\n Imaginary part of Eigenvalues :\n");
    printVector(&vals_im);

    printf("\n Eigenvectors (are *rows*):\n");
    printMatrix(&vecs);

    char name[20];
    sprintf(name, "eigenvalues%d.data", N);
    FILE *fileout = fopen(name, "w");

    for (i = 0; i < N; i++)
    {
        fprintf(fileout, "%4i %8.4lf \n", i, vals_real.Vector[i]);
    }
    return 0;
}
