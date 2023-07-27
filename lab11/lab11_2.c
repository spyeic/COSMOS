#include <stdlib.h>
#include <stdio.h>
#include "../matrix_functions.h"

double dot(Vec *v1, Vec *v2)
{
    double sum = 0;
    for (int i = 0; i < v1->N; i++)
    {
        sum += v1->Vector[i] * v2->Vector[i];
    }
    return sum;
}

double dotFromVecs(Mat *vecs)
{
    Vec v1, v2;
    arrayToVector(vecs->N, vecs->Matrix[0], &v1);
    arrayToVector(vecs->N, vecs->Matrix[1], &v2);
    return dot(&v1, &v2);
}

int main()
{
    int N = 2;
    int i, j;
    Mat testmat, vecs;
    Vec vals_real, vals_im;

    makeMatrix(N, N, &testmat);
    makeMatrix(N, N, &vecs);
    makeVector(N, &vals_real);
    makeVector(N, &vals_im);

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

    double result = dotFromVecs(&vecs);
    printf("Dot product of eigenvector1s: %lf\n", result);

    Mat mat, new;
    double v[1][N];
    for (int i = 0; i < N; i++)
    {
        v[0][i] = vecs.Matrix[0][i];
    }
    arrayToMatrix(N, N, v, &new);
    multiplyMatrix(&testmat, &new, &mat);
    printf("Matrix * eigenvector: \n");
    printMatrix(&mat);

    return 0;
}
