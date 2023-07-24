#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main() {
    double x, y, vx, vy, t, dt, Fx, Fy, m, b, g = -9.8;
    int j;
    FILE * fileout;
    fileout = fopen("a.data", "w");
    printf("\nEnter mass m, friction constant b, and time step dt:   ");
    scanf("%lf %lf %lf", &m, &b, &dt);
    printf("\nEnter starting position x, y:   ");
    scanf("%lf %lf", &x, &y);
    printf("\nEnter starting velocity vx, vy:   ");
    scanf("%lf %lf", &vx, &vy);
    t = 0.;
    do {
        t = t + dt;
        x = x + vx * dt;
        y = y + vy * dt;
        Fx = -b * vx;
        Fy = m * g - b * vy;
        vx = vx + Fx / m * dt;
        vy = vy + Fy / m * dt;
        fprintf(fileout, "\n   %12.6lf %12.6lf", x, y);
    } while (y > 0);
    fclose(fileout);
    return 0;
}