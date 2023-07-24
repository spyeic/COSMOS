#include <stdio.h>
#include <math.h>

int main()
{
    int t = 0, total = 86400 * 365, dt = 10000;
    // double x = 149600000000, y = 0;
    double x, y, v;
    printf("Enter x and y: ");
    scanf("%lf %lf", &x, &y);

    // double v = 29780;
    printf("Enter init v: ");
    scanf("%lf", &v);

    double vx, vy, r;

    FILE *output = fopen("kepler.data", "w");

    double F, Fx, Fy,
        G = 0.0000000000667,
        m_earth = 5.7922 * pow(10, 24),
        m_sun = 2 * pow(10, 30);

    vx = v * y / r;
    vy = v * x / r;
    while (t < total)
    {
        F = (G * m_earth * m_sun) / pow(r, 2);
        r = sqrt(pow(x, 2) + pow(y, 2));
        Fx = -F * x / r;
        Fy = -F * y / r;
        x += vx * dt;
        y += vy * dt;
        vx += (Fx / m_earth) * dt;
        vy += (Fy / m_earth) * dt;
        t += dt;
        fprintf(output, "\n   %12.6lf %12.6lf %12.6lf %12.6lf", x, y, vx, vy);
    }

    fclose(output);
    return 0;
}