#include <stdio.h>

int main() {
    int a, b, c, d;
    printf("First line a: ");
    scanf("%d", &a);
    printf("First line b: ");
    scanf("%d", &b);
    printf("First line is y = %dx + %d\n", a, b);
    printf("Second line a: ");
    scanf("%d", &c);
    printf("Second line b: ");
    scanf("%d", &d);
    printf("Second line is y = %dx + %d\n", c, d);
    
    double x = (d - b) / (a - c);
    double y = a * x + b;
    printf("The intersection is at (%lf, %lf)\n", x, y);
}