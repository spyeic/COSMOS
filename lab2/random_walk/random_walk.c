#include <stdlib.h>
#include <stdio.h>


int main()
{
    int x = 0;
    int N;
    printf("Enter N:  ");
    scanf("%d", &N);
    
    // srand(100);
    double R;
    for (int i = 0; i < N; i++) {
        R = (double) rand() / RAND_MAX;
        // printf("%lf", R);
        if (R < 0.5) {
            x--;
        } else if (R > 0.5) {
            x++;
        }
        printf("x is %d in time %d\n", x, i);
    }
    printf("x is %d\n", x);

    return 0;
}