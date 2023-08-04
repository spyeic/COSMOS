#include <stdio.h>
#include <stdio.h>
#include "../matrix_functions.h"
#include <stdbool.h>
#include <math.h>

int positionR(int N, int index)
{
    int r = index / N;
    return r;
}

int positionC(int N, int index)
{
    int c = (index + N) % N;
    return c;
}

int main()
{
    int N = 32, M, start;
    double dt = 0.01, time = 5;

    M = N * N;

    Mat H, UR, UI, vecs;
    Vec PSIR, PSII, vals_real, vals_im, prob;
    makeMatrix(M, M, &H);
    makeMatrix(M, M, &UR);
    makeMatrix(M, M, &UI);
    makeMatrix(M, M, &vecs);
    makeVector(M, &PSIR);
    makeVector(M, &PSII);
    makeVector(M, &vals_real);
    makeVector(M, &vals_im);
    makeVector(M, &prob);

    // HARD CODING SQUARE LATTICE
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            int n = i + j * N;
            if (i > 0)
            {
                H.Matrix[n][n - 1] = 0.5;
            }
            if (i < N - 1)
            {
                H.Matrix[n][n + 1] = 0.5;
            }
            if (j > 0)
            {
                H.Matrix[n][n - N] = 0.5;
            }
            if (j < N - 1)
            {
                H.Matrix[n][n + N] = 0.5;
            }
        }
        H.Matrix[i][i] = -13.6;
    }

    get_eigenvalues(&H, &vals_real, &vals_im, &vecs, 1);

    // initialize wav func and time evo operator
    for (int i = 0; i < M; i++)
    {
        PSIR.Vector[i] = 0.0;
        PSII.Vector[i] = 0.0;
        for (int j = 0; j < M; j++)
        {
            UR.Matrix[i][j] = 0.0;
            UI.Matrix[i][j] = 0.0;
        }
    }
    // ask user to input location of first electron
    
    printf("Enter the atom number the electron begins at: ");
    scanf("%i", &start);
    char filename[100];
    sprintf(filename, "qst3d_%i.data", start);
    FILE *fileout = fopen(filename, "w");
    PSIR.Vector[start] = 1.0;

    Vec tempR, tempI;
    makeVector(M, &tempR);
    makeVector(M, &tempI);

    // printx
    for (int i = 0; i < M; i++)
    {
        fprintf(fileout, "%i ", positionR(N, i));
    }
    fprintf(fileout, "\n");

    // printy
    for (int i = 0; i < M; i++)
    {
        fprintf(fileout, "%i ", positionC(N, i));
    }
    fprintf(fileout, "\n");

    // start of time loop
    for (double t = 0; t < time; t += dt)
    {
        for (int i = 0; i < M; i++)
        {
            for (int j = 0; j < M; j++) // [i][j] correspond to index [i][j] of the exponentiated matrix
            {
                for (int k = 0; k < M; k++)
                {
                    UR.Matrix[i][j] += (cos(vals_real.Vector[k] * t)) * vecs.Matrix[k][i] * vecs.Matrix[k][j];
                    UI.Matrix[i][j] += -(sin(vals_real.Vector[k] * t)) * vecs.Matrix[k][i] * vecs.Matrix[k][j];
                }
                // multiplying U * PSI
                tempR.Vector[i] += UR.Matrix[i][j] * PSIR.Vector[j] - UI.Matrix[i][j] * PSII.Vector[j];
                tempI.Vector[i] += UI.Matrix[i][j] * PSIR.Vector[j] + UR.Matrix[i][j] * PSII.Vector[j];
            }
            // calculate probabilty squaring psir psii components
            prob.Vector[i] = (tempR.Vector[i] * tempR.Vector[i]) + (tempI.Vector[i] * tempI.Vector[i]);
            printf("t: %lf i: %i\n", t, i);
        }
        // printing probabilties:
        for (int i = 0; i < M; i++)
        {
            fprintf(fileout, "%1.9lf ", prob.Vector[i]);
        }
        fprintf(fileout, "\n");

        // reset PSI and U
        for (int i = 0; i < M; i++)
        {
            PSIR.Vector[i] = tempR.Vector[i];
            PSII.Vector[i] = tempI.Vector[i];
            tempR.Vector[i] = 0.0;
            tempI.Vector[i] = 0.0;
            for (int j = 0; j < M; j++)
            {
                UR.Matrix[i][j] = 0.0;
                UI.Matrix[i][j] = 0.0;
            }
        }

        // end time loop
    }

    fclose(fileout);
    return 0;
}