#include <stdio.h>
#include <math.h>

int main()
{
    double y, v, t, dt, F, m, g = -9.8;
    int j;

    FILE *fileout;
    fileout = fopen("Hagrid", "w");
    printf("\nEnter mass m:   ");
    scanf("%lf", &m);
    printf("\nEnter starting height y and velocity v:   ");
    scanf("%lf %lf", &y, &v);
    printf("\nEnter time step dt:   ");
    scanf("%lf", &dt);
    t = 0.;
    fprintf(fileout, "\n   %12.6lf %12.6lf", t, y);

    do
    {
        t = t + dt;
        y = y + v * dt;
        F = m * g;
        v = v + (F / m) * dt;
        fprintf(fileout, "\n   %12.6lf %12.6lf", t, y);
    } while (y > 0);
    
    fclose(fileout);
    return 0;
}