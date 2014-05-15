#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[])
{

	printf("Using Driver\n");

	FILE* file = fopen("/dev/ueb5_ad_da","r");

	//fseek(file, SEEK_SET, 0);

	short input = 0;
	float ergebnis = 0.0;


	fread(&input, sizeof(input), 1, file);
	
	
	printf("gelesen: >%d<\n", input);

	ergebnis = input / 3276.7;

	printf("ergebnis: %f\n", ergebnis);

	fclose(file);

	return(0);
}
