#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[])
{

	printf("Using Driver\n");

	FILE* file = fopen("/dev/ueb5_ad_da","r");

	//fseek(file, SEEK_SET, 0);

	int input = 0;

	float faktor = 3320.0; // der Faktor zur Umrechnung von gelesenem Wert in Volt.
	
	float ergebnis = 0.0;


	fread(&input, sizeof(input), 1, file);
	
	
	
	printf("gelesen: >%d<\n", input);

	if(input < 0){
		printf("Fehler. Gelesener Wert sollte eigentlich nicht kleiner als 0 sein");
	}

	if(input < 32768){
		printf("positiv");
		// positiver bereich	
		ergebnis = input / faktor; 
	}else {
		// negativer bereich
		printf("negativ\n");		
		ergebnis = (65536 - input)/-faktor;
	}		



	printf("ergebnis: %f\n", ergebnis);

	fclose(file);

	return(0);
}
