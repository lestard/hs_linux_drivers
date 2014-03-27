#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[])
{
	printf("Using Driver\n");

	FILE* file = fopen("/dev/blub","r");

	fseek(file, SEEK_SET, 0);
	
	char buffer[20];

	fread(buffer, 1, 1, file);

	printf("gelesen: %s\n", buffer);
	
	fclose(file);

	return(0);
}
