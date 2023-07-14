#include <stdio.h>
#include <stdlib.h>
#include <time.h>

double ran() {
    return (double) rand() / RAND_MAX;
}

int main()
{
    srand(time(NULL));
    printf("%lf", ran());
    printf("%lf", ran());
    printf("%lf", ran());
    printf("%lf", ran());
    printf("%lf", ran());
}
