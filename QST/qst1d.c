#include <stdio.h>
#include <stdio.h>
#include "../matrix_functions.h"
#include <stdbool.h>
#include <math.h>

int main()
{

    int N, M, start;
    double dt = 0.05, time = 24;
    printf("Enter N: ");
    scanf("%i", &N);

    FILE *fileout = fopen("qstai.txt", "w");
    M = N;
    Mat H, UR, UI, vecs;
    Vec PSIR, PSII, vals_real, vals_im, prob;
    makeMatrix(N, N, &H);
    makeMatrix(N, N, &UR);
    makeMatrix(N, N, &UI);
    makeMatrix(N, N, &vecs);
    makeVector(N, &PSIR);
    makeVector(N, &PSII);
    makeVector(N, &vals_real);
    makeVector(N, &vals_im);
    makeVector(N, &prob);

    // 1d chain hammy:
    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < M; j++)
        {
            H.Matrix[i][j] = 0;
            
        }
    }
    for (int i = 0; i < M; i++)
    {

        H.Matrix[i][i] = -13.6;
        // H.Matrix[i][i] = 0;
        if (i != 0)
            H.Matrix[i][i - 1] = 0.5;
        if (i != N - 1)
            H.Matrix[i][i + 1] = 0.5;
    }
    printMatrix(&H);
    get_eigenvalues(&H, &vals_real, &vals_im, &vecs, 1);

    // initialize wav func and time evo operator
    for (int i = 0; i < N; i++)
    {
        PSIR.Vector[i] = 0.0;
        PSII.Vector[i] = 0.0;
        for (int j = 0; j < N; j++)
        {
            UR.Matrix[i][j] = 0.0;
            UI.Matrix[i][j] = 0.0;
        }
    }
    // ask user to input location of first electron
    printf("Enter the atom number the electron begins at: ");
    scanf("%i", &start);
    PSIR.Vector[start] = 1.0;

    Vec tempR, tempI;
    makeVector(N, &tempR);
    makeVector(N, &tempI);

    // start of time loop
    for (double t = 0; t < time; t += dt)
    {
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++) // [i][j] correspond to index [i][j] of the exponentiated matrix
            {
                for (int k = 0; k < N; k++)
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
        }
        // printing probabilties:
        for (int i = 0; i < M; i++)
        {
            fprintf(fileout, "%1.9lf ", prob.Vector[i]);
        }
        fprintf(fileout, "\n");

        // reset PSI and U
        for (int i = 0; i < N; i++)
        {
            PSIR.Vector[i] = tempR.Vector[i];
            PSII.Vector[i] = tempI.Vector[i];
            tempR.Vector[i] = 0.0;
            tempI.Vector[i] = 0.0;
            for (int j = 0; j < N; j++)
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