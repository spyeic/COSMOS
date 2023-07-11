#include <stdio.h>
#include <math.h>

int main()
{
	int a, b, c;
	printf("First number:");
	scanf("%d", &a);
	printf("Second number:");
	scanf("%d", &b);
	printf("Third number:");
	scanf("%d", &c);
	printf("\n");
	printf("y = %dx^2 + %dx + %d \n", a, b, c);

	double result = pow(b, 2) - 4.0 * a * c;
	if (result < 0)
	{
		printf("There is no root for this quadratic.\n");
	}
	else if (result == 0)
	{
		printf("There is only one root, it is %lf\n", (-b + sqrt(result) / (2 * a)));
	}
	else
	{
		printf("First root is %lf\n", ((-b + sqrt(result)) / (2 * a)));
		printf("Second root is %lf\n", ((-b - sqrt(result)) / (2 * a)));
	}
}
