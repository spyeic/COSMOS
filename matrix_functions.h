/*
Written by ENB
V1.0: 7/19/22 - initial
V1.1 = 7/22/22 - add sorting, use optimium work size
 V1.2 = 7/25/22 - add support for big arrays
  V2.0= 7/25/22 - OOP rewrite
  V2.1= 7/26/22 - Correct indexing
To use, call: call get_eigenvalues using Mat and Vec types. See test.c

N and mat are inputs, vals and vecs are outputs. If sortflag==1, the results are sorted by the magnitude of the real component of the eigenvalue.

This header requires OpenBLAS.

To compile file "test.c", run: gcc test.c -lopenblas
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern void
dgeev_(char *jobvl, char *jobvr, int *n, double *a, int *lda, double *wr, double *wi, double *vl, int *ldvl, double *vr,
       int *ldvr, double *work, int *lwork, int *info);
extern void dgetrf_(int *M, int *N, double *A, int *lda, int *IPIV, int *INFO);
extern void dgetri_(int *N, double *A, int *lda, int *IPIV, double *WORK, int *lwork, int *INFO);
extern void dgemm_(char *TRANSA, char *TRANSB, int *M, int *N, int *K, double *ALPHA, double *A, int *LDA, double *B, int *LDB, double *BETA, double *C, int *LDC);
typedef struct
{
    int N;
    int M;
    double **Matrix;
} Mat;
typedef struct
{
    int N;
    double *Vector;
} Vec;
typedef struct
{
    int N;
    int *Vector;
} VecInt;

void makeMatrix(int N, int M, Mat *Matrix)
{
    Matrix->N = N;
    Matrix->M = M;
    Matrix->Matrix = malloc(M * sizeof(double *));
    for (int i = 0; i < M; i++)
    {
        Matrix->Matrix[i] = malloc(N * sizeof(double));
        memset(Matrix->Matrix[i], 0, sizeof(Matrix->Matrix[i]));
    }
}
void arrayToMatrix(int N, int M, double arr[M][N], Mat *Matrix)
{
    makeMatrix(N, M, Matrix);
    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            Matrix->Matrix[i][j] = arr[i][j];
        }
    }
}
void destroyMatrix(Mat *Matrix)
{
    int N = Matrix->N;
    int M = Matrix->M;
    for (int i = 0; i < M; i++)
    {
        free(Matrix->Matrix[i]);
    }
    free(Matrix->Matrix);
}
void randomMatrix(Mat *Matrix)
{
    for (int i = 0; i < Matrix->M; i++)
    {
        for (int j = 0; j < Matrix->N; j++)
        {
            Matrix->Matrix[i][j] = (double)rand() / (double)RAND_MAX;
        }
    }
}
void copyMatrix(Mat *output, Mat *input)
{
    output->N = input->N;
    output->M = input->M;
    for (int i = 0; i < input->N; i++)
    {
        for (int j = 0; j < input->M; j++)
        {
        }
    }
}
void randomHMatrix(Mat *Matrix)
{
    for (int i = 0; i < Matrix->M; i++)
    {
        for (int j = i; j < Matrix->N; j++)
        {
            Matrix->Matrix[i][j] = (double)rand() / (double)RAND_MAX;
        }
    }
    for (int i = 0; i < Matrix->M; i++)
    {
        for (int j = i; j < Matrix->N; j++)
        {
            Matrix->Matrix[j][i] = Matrix->Matrix[i][j];
        }
    }
}
void printMatrix(Mat *Matrix)
{
    for (int i = 0; i < Matrix->M; i++)
    {
        for (int j = 0; j < Matrix->N; ++j)
        {
            printf("%lf ", Matrix->Matrix[i][j]);
        }
        printf("\n");
    }
}

void makeVector(int N, Vec *Vector)
{
    Vector->N = N;
    Vector->Vector = malloc(N * sizeof(double));
    for (int i = 0; i < N; i++)
    {
        Vector->Vector[i] = 0;
    }
}
void makeIntVector(int N, VecInt *Vector)
{
    Vector->N = N;
    Vector->Vector = malloc(N * sizeof(int));
    for (int i = 0; i < N; i++)
    {
        Vector->Vector[i] = 0;
    }
}
void destroyVector(Vec *Vector)
{
    free(Vector->Vector);
}
void destroyIntVector(VecInt *Vector)
{
    free(Vector->Vector);
}
void printVector(Vec *Vector)
{
    for (int i = 0; i < Vector->N; i++)
    {
        printf("%lf ", Vector->Vector[i]);
        printf("\n");
    }
}
void printIntVector(VecInt *Vector)
{
    for (int i = 0; i < Vector->N; i++)
    {
        printf("%i ", Vector->Vector[i]);
        printf("\n");
    }
}
void arrayToVector(int N, double arr[N], Vec *Vector)
{
    makeVector(N, Vector);
    for (int i = 0; i < N; i++)
    {
        Vector->Vector[i] = arr[i];
    }
}

int cmp(const void *pa, const void *pb)
{
    const double *a = *(const double **)pa;
    const double *b = *(const double **)pb;
    if (a[0] == b[0])
    {
        return 0;
    }
    if (a[0] > b[0])
    {
        return 1;
    }
    if (a[0] < b[0])
    {
        return -1;
    }
}
void multiplyMatrix(Mat *matrix1, Mat *matrix2, Mat *result)
{
    int N_1 = matrix1->N;
    int M_1 = matrix1->M;
    int N_2 = matrix2->N;
    int M_2 = matrix2->M;
    Vec vec1, vec2, res_vec;
    makeVector(N_1 * M_1, &vec1);
    for (int i = 0; i < N_1; i++)
    {
        for (int j = 0; j < M_1; j++)
        {
            vec1.Vector[N_1 * i + j] = matrix1->Matrix[i][j];
        }
    }
    makeVector(N_2 * M_2, &vec2);
    for (int i = 0; i < N_2; i++)
    {
        for (int j = 0; j < M_2; j++)
        {
            vec2.Vector[N_2 * i + j] = matrix2->Matrix[i][j];
        }
    }
    makeVector(N_1 * M_2, &res_vec);
    double one = 1;
    dgemm_("N", "N", &N_1, &M_2, &M_1, &one, vec1.Vector, &N_1, vec2.Vector, &N_2, 0, res_vec.Vector, &N_1);
    for (int i = 0; i < N_1; i++)
    {
        for (int j = 0; j < M_1; j++)
        {
            result->Matrix[i][j] = res_vec.Vector[N_1 * i + j];
        }
    }
    destroyVector(&res_vec);
    destroyVector(&vec1);
    destroyVector(&vec2);
}

void invertMatrix(Mat *inputMatrix, Mat *outputMatrix)
{
    int info;
    int N = inputMatrix->N;
    Vec inputMatrix_copy;
    VecInt ipiv;
    makeVector(N * N, &inputMatrix_copy);
    makeIntVector(N, &ipiv);
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            inputMatrix_copy.Vector[N * i + j] = inputMatrix->Matrix[i][j];
        }
    }

    dgetrf_(&N, &N, inputMatrix_copy.Vector, &N, ipiv.Vector, &info);
    int LWORK = -1;
    double work_test[1];
    dgetri_(&N, inputMatrix_copy.Vector, &N, ipiv.Vector, work_test, &LWORK, &info);
    Vec work;
    LWORK = work_test[0];
    makeVector(LWORK, &work);
    dgetri_(&N, inputMatrix_copy.Vector, &N, ipiv.Vector, work.Vector, &LWORK, &info);
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            outputMatrix->Matrix[i][j] = inputMatrix_copy.Vector[i * N + j];
        }
    }
    destroyIntVector(&ipiv);
    destroyVector(&inputMatrix_copy);
    destroyVector(&work);
}

void get_eigenvalues(Mat *input_matrix, Vec *output_vals_real, Vec *output_vals_im, Mat *output_eigenvecs,
                     int sortflag)
{
    // COPY FOR MEMORY SAFETY
    int N = input_matrix->N;
    int LWORK = -1;
    double work_test[1];
    // double mat_copy[N*N];
    double *mat_copy;
    mat_copy = malloc(N * N * sizeof(double));

    // double vecs_return_copy[N*N];
    double *vecs_return_copy;
    vecs_return_copy = malloc(N * N * sizeof(double));
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            mat_copy[N * (i) + j] = input_matrix->Matrix[j][i];
        }
    }

    int info;
    dgeev_("N", "V", &N, mat_copy, &N, output_vals_real->Vector, output_vals_im->Vector, NULL, &N, vecs_return_copy, &N, work_test,
           &LWORK, &info);
    LWORK = work_test[0];
    double *work = malloc(sizeof(double) * LWORK);
    memset(work, 0, sizeof(double) * LWORK);

    dgeev_("N", "V", &N, mat_copy, &N, output_vals_real->Vector, output_vals_im->Vector, NULL, &N, vecs_return_copy, &N, work, &LWORK,
           &info);

    free(work);
    if (info != 0)
    {
        printf("%s", "error");
    }

    if (sortflag == 1)
    {
        double **eigsortarr;
        eigsortarr = malloc(N * sizeof(double *));
        for (int i = 0; i < N; i++)
        {
            eigsortarr[i] = malloc(2 * sizeof(double));
            eigsortarr[i][0] = output_vals_real->Vector[i];
            eigsortarr[i][1] = i;
        }
        printf("sorting\n");
        qsort(eigsortarr, N, sizeof(eigsortarr[0]), cmp);
        double *vals_im_return_copy;
        vals_im_return_copy = malloc(N * sizeof(double));
        for (int i = 0; i < N; i++)
        {
            vals_im_return_copy[i] = output_vals_im->Vector[i];
        }

        for (int i = 0; i < N; i++)
        {
            output_vals_real->Vector[i] = eigsortarr[i][0];
            output_vals_im->Vector[i] = vals_im_return_copy[(int)eigsortarr[i][1]];
            for (int j = 0; j < N; j++)
            {
                output_eigenvecs->Matrix[i][j] = vecs_return_copy[N * ((int)eigsortarr[i][1]) + j];
            }
        }
        for (int i = 0; i < N; i++)
        {
            free(eigsortarr[i]);
        }
        free(eigsortarr);
        free(vals_im_return_copy);
    }
    else
    {
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {
                output_eigenvecs->Matrix[i][j] = vecs_return_copy[N * (i) + j];
            }
        }
    }

    free(mat_copy);
    free(vecs_return_copy);
}