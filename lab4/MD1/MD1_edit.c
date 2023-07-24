#include <stdio.h>
#include <math.h>

int main()
{
    double y, v, t, total, F, g = -9.8;
    int j;
    
    FILE *fileout;
    fileout = fopen("Hagrid", "w");
    printf("\nEnter starting height y and velocity v:   ");
    scanf("%lf %lf", &y, &v);
    printf("\nEnter total time:   ");
    scanf("%lf", &total);
    t = 0.;
    fprintf(fileout, "\n   %12.6lf %12.6lf", t, y);

    double dt = 0.01;

    for (;t < total; t += dt)
    {
        y = y + v * dt;
        v = v + g * dt;
        fprintf(fileout, "\n   %12.6lf %12.6lf", t, y);
    }
    fclose(fileout);
    return 0;
}