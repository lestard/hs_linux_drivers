#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[])
{
	int i;

	printf("Using Driver\n");

	FILE* file = fopen("/dev/ueb5_ad_da","r");

	//fseek(file, SEEK_SET, 0);
	
	char buffer[3];

		
	/**
	for(i=0 ; ; i++)
	{
		fread(&buffer[i], 1, 1, file);
		if(buffer[i] == 0)
		{
			break;
		}
	}
	*/


	fread(buffer, 3, 1, file);


	printf("gelesen: >%s<\n", buffer);
	
	fclose(file);

	return(0);
}
