#include <stdio.h>

int main() {
    int N;
    printf("Enter N: ");
    scanf("%d", &N);

    int S = 0;
    for (int i = 1; i <= N; i++) {
        S += i;
    }
    printf("%d\n", S);
}