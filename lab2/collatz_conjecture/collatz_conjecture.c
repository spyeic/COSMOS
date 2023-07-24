#include <stdio.h>

int main() {
    int num;
    int prev;
    scanf("%d", &num);
    while (num != 1) {
        prev = num;
        if (num % 2 == 0)
        {
            num = num / 2;
        } else {
            num = num * 3 + 1;
        }
        printf("%d => %d\n", prev, num);
    }
}