#include <stdlib.h>
#include <stdio.h>
#include "../matrix_functions.h"
int main()
{

    FILE *fileout;
    fileout = fopen("buckbeak", "w");
    int N = 2;
    int i, j;
    Mat testmat, vecs;
    Vec vals_real, vals_im;

    makeMatrix(N, N, &testmat);
    makeMatrix(N, N, &vecs);
    makeVector(N, &vals_real);
    makeVector(N, &vals_im);

    //  SET UP MATRIX IN CODE ITSELF
    //    testmat.Matrix[0][0] = 3.8;
    //    testmat.Matrix[1][0] = 0.6;
    //    testmat.Matrix[0][1] = 0.6;
    //    testmat.Matrix[1][1] = 2.2;

    //  READ IN MATRIX

    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            printf("Enter matrix element %2i %2i:  ", i, j);
            scanf("%lf", &testmat.Matrix[i][j]);
        }
    }

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

    //  PRINT TO FILE:

    fprintf(fileout, "\n Original Matrix being diagonalized :\n");
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            fprintf(fileout, "%8.4lf ", testmat.Matrix[i][j]);
        }
        fprintf(fileout, "\n ");
    }

    fprintf(fileout, "\n Real part of Eigenvalues :\n");
    for (i = 0; i < N; i++)
    {
        fprintf(fileout, "%8.4lf \n", vals_real.Vector[i]);
    }

    fprintf(fileout, "\n Imag part of Eigenvalues :\n");
    for (i = 0; i < N; i++)
    {
        fprintf(fileout, "%8.4lf \n", vals_im.Vector[i]);
    }

    fprintf(fileout, "\n Eigenvectors (are *rows*):\n");
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            fprintf(fileout, "%8.4lf ", vecs.Matrix[i][j]);
        }
        fprintf(fileout, "\n ");
    }

    fclose(fileout);
    return 0;
}
