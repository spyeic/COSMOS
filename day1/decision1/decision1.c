#include <stdio.h>
#include <math.h>

int main() {
    float x, y;
    printf("Enter x and y: ");
    scanf("%f %f", &x, &y);
    if (x > y) {
        printf("The first number was bigger\n");
    } else {
        printf("The second number was bigger\n");
    }
}
