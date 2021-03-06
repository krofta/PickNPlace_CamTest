#include <stdio.h>
//#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "bv.h"
#include "fonts.h"





/***********************************************************************************************************/
/*                      Sonstige Hilfsfunktionen                                                           */
/***********************************************************************************************************/
void frambuffer_test(unsigned char cMatrix[MAXYDIM][MAXXDIM]){
	init_cMatrix(img, 0);
	/*
	cMatrix[40][150] = 255;
	cMatrix[50][200] = 255;
	cMatrix[40][150] = 255;
	cMatrix[310][235] = 255;
	cMatrix[10][235] = 255;
	*/
	for(int x = 0,y = 0; x < MAXYDIM; x+=10, y+=10){
		img[y][x] = 255;
	}
}

void init_cMatrix(unsigned char cMatrix[MAXYDIM][MAXXDIM], unsigned char val)
{
	memset(cMatrix, val, MAXXDIM*MAXYDIM * sizeof(unsigned char));
	return;
}

void init_iMatrix(uint16_t iMatrix[MAXYDIM][MAXXDIM] , uint16_t val)
{
	for(int x = 0; x < MAXXDIM; x++)
		for(int y = 0; y < MAXYDIM; y++)
			iMatrix[y][x] = (val>>8) + ((val&0xFF)<<8);
}
/*
void init_fMatrix(float fMatrix[MAXYDIM][MAXXDIM])
{
	memset(fMatrix, 0, MAXXDIM*MAXYDIM*sizeof(float));
	return;
}
*/
//findet den vom Betrag gr��ten/kleinsten Wert der Matrix

int16_t  find_abs_extremum_iMatrix(int16_t min_max, int16_t iMatrix[MAXYDIM][MAXXDIM])
{
	int16_t extremum = min_max == MAX ? 0 : 0xFFFF;
	int16_t abs = 0;
	if (min_max == MAX){
		for (int i = 0; i < MAXXDIM; i++){
			for (int j = 0; j < MAXYDIM; j++){
				abs = iMatrix[j][i] < 0 ? iMatrix[j][i] * -1 : iMatrix[j][i]; //sqrt(pow(iMatrix[i][j], 2));
				if (abs > extremum)
					extremum = abs;
			}
		}
	}else{
		for (int i = 0; i < MAXXDIM; i++){
			for (int j = 0; j < MAXYDIM; j++){
				abs = iMatrix[j][i] < 0 ? iMatrix[j][i] * -1 : iMatrix[j][i]; //sqrt(pow(iMatrix[i][j], 2));
				if (abs < extremum)
					extremum = abs;
			}
		}
	}

	return extremum;
}

uint16_t  find_abs_extremum_uiMatrix(uint16_t min_max, uint16_t uiMatrix[MAXYDIM][MAXXDIM])
{
	uint16_t extremum = min_max == MAX ? 0 : 0xFFFF;
	if (min_max == MAX){
		for (int i = 0; i < MAXXDIM; i++){
			for (int j = 0; j < MAXYDIM; j++){
				if (uiMatrix[j][i] > extremum){
					extremum = uiMatrix[j][i];
				}
			}
		}
	}else{
		for (int i = 0; i < MAXXDIM; i++){
			for (int j = 0; j < MAXYDIM; j++){
				if (uiMatrix[j][i] < extremum){
					extremum = uiMatrix[j][i];
				}
			}
		}
	}
	return extremum;
}

//findet den vom Betrag gr��ten/kleinsten Wert der Matrix
/*
float  find_abs_extremum_fMatrix(int min_max, float fMatrix[MAXYDIM][MAXXDIM])
{
	float extremum = min_max == MAX ? 0.0 : 0xFFFF;
	float abs = 0;
	if (min_max == MAX ){
		for (int i = 0; i < MAXXDIM; i++){
			for (int j = 0; j < MAXYDIM; j++){
				abs = fMatrix[i][j] < 0 ? fMatrix[i][j] * -1 : fMatrix[i][j]; // sqrt(pow(iMatrix[i][j], 2));
				if(abs > extremum)
					extremum = abs;
			}
		}
	} else {
		for (int i = 0; i < MAXXDIM; i++){
			for (int j = 0; j < MAXYDIM; j++){
				abs = fMatrix[i][j] < 0 ? fMatrix[i][j] * -1 : fMatrix[i][j]; // sqrt(pow(iMatrix[i][j], 2));
				if ( abs < extremum)
					extremum = abs;
			}
		}
	}
	return extremum;
}
*/



// Bubble-Sort (Sortierverfahren), irgendwo aus dem internet kopiert :-)
// qsort(), w�re auch gegangen (Quicksort in ANSI c Bibliothek enthalten)
void bubblesort(int *array, int length)
{
	for (int i = 0; i < length - 1; ++i)
		for (int j = 0; j < length - i - 1; ++j)
			if (array[j] > array[j + 1]) {
				int tmp = array[j];
				array[j] = array[j + 1];
				array[j + 1] = tmp;
			}
}

// Formel für die Fakultät
double fakultaet(int n)
{
	double i, fak;
	for (i = 1, fak = 1; i <= n; i++)
		fak *= i;
	return fak;
}

// Binomialverteilung einer bestimmten stufe einholen
void get_bin_koeff(float bin_ver[50], int n, float normierung)
{
	if (n > 50)
		return;
	//float prev_gauss[50];

	for (int k = 0; k <= n; k++)		// n über k = n!/(n-k)!*k!
		bin_ver[k] = (float)fakultaet(n) / (float)(fakultaet(n - k) * fakultaet(k));
	// Summe aller Koeffizienten auf 100 normieren
	float sum = 0.0;
	for (int i = 0; i <= n; i++)
		sum += bin_ver[i];
	float faktor = normierung / sum;
	for (int i = 0; i <= n; i++)
		bin_ver[i] *= faktor;
}

void reset_blob_label(uint16_t iIMG[MAXYDIM][MAXXDIM], uint16_t oldLabel, uint16_t newLabel)
{
	for (int x = 0; x < MAXXDIM; x++)
		for (int y = 0; y < MAXYDIM; y++)
			if (iIMG[y][x] == oldLabel)
				iIMG[y][x] = newLabel;
}


void rgb_to_greyscale(uint16_t iIMG[MAXYDIM][MAXXDIM], unsigned char img[MAXYDIM][MAXXDIM]){
	for(int x = 0; x < MAXXDIM ; x++){
		for(int y = 0; y < MAXYDIM; y++){
			// byteorder tauschen
			uint16_t byteorder = (iIMG[y][x]>>8)  + ((iIMG[y][x]&0xFF)<<8);
			uint16_t red = ((byteorder & 0xF800)>>8);
			uint16_t green = ((byteorder & 0x07E0)>>5);
			uint16_t blue = ((byteorder & 0x001F)<<3);
			uint16_t grayscale = (red + green + blue) / 3;
			img[y][x] = grayscale;
		}
	}
}

void greyscale_to_greyrgb(uint16_t iIMG[MAXYDIM][MAXXDIM],unsigned char img[MAXYDIM][MAXXDIM]){
	for(int x = 0; x < MAXXDIM ; x++){
		for(int y = 0; y < MAXYDIM; y++){
			// BLUE    0x001F
			// GREEN   0x07E0
			// RED     0xF800
			// Byteorder muss zum display getauscht werden
			uint16_t grey =((img[y][x] & 0xF8)<<8) + ((img[y][x] & 0xFC)<<3) + ((img[y][x] & 0xF8)>>3);
			iIMG[y][x] = (grey>>8) + ((grey&0xFF)<<8);
		}
	}
}

/***********************************************************************************************************/
/*                      Binäre Bildverarbeitung                                                            */
/***********************************************************************************************************/
/*
//Shrink Algorithmus
void shrink(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM]) {
	int i,j;
	init_cMatrix(img2, 0);
	for (i = 1; i < MAXXDIM - 1; i++) 
		for (j = 1; j < MAXYDIM - 1; j++) 
			if (img[i][j] == 255 && img[i][j + 1] == 255 && img[i][j - 1] == 255 && img[i - 1][j] == 255 && img[i + 1][j] == 255) 
				img2[i][j] = 255;
}

// Blow Algorithmus
void blow(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM]) {
	int i;
	int j;
	init_cMatrix(img2, 0);
	for (i = 1; i < MAXXDIM - 1; i++) 
		for (j = 1; j < MAXYDIM - 1; j++) 
			if (img[i][j] == 255) {
				img2[i][j] = 255;
				img2[i][j + 1] = 255;
				img2[i][j - 1] = 255;
				img2[i - 1][j] = 255;
				img2[i + 1][j] = 255;
			}
}


// Open Algorithmus
void funk_open(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM]) {
	int x,y,i,j;
	//printf("Geben Sie die Anzahl der Erosion- und Dilatation- Vorgaenge an\n");
	scanf("%d", &y);
	for (x = 0; x < y; x++) {
		shrink(img, img2);
		for (i = 0; i < MAXXDIM; i++) 
			for (j = 0; j < MAXYDIM; j++) 
				img[i][j] = img2[i][j];
	}
	for (x = 0; x < y; x++) {
		blow(img, img2);
		for(i = 0; i < MAXXDIM; i++) 
			for (j = 0; j < MAXYDIM; j++) 
				img[i][j] = img2[i][j];
	}
	system("cls");
}

// Close Algorithmus
void funk_close(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM]) {
	int x,y,i,j;
	//printf("Geben Sie die Anzahl der Dilatation- und Erosion- Vorgaenge an\n");
	scanf("%d", &y);
	for (x = 0; x < y; x++) {
		blow(img, img2);
		for (i = 0; i < MAXXDIM; i++) 
			for (j = 0; j < MAXYDIM; j++) 
				img[i][j] = img2[i][j];
	}
	for (x = 0; x < y; x++) {
		shrink(img, img2);
		for (i = 0; i < MAXXDIM; i++) 
			for (j = 0; j < MAXYDIM; j++) 
				img[i][j] = img2[i][j];
	}
	system("cls");
}

#define UNDEF 0
#define BURN 1
#define BURNED 2

// Zählen von Objekten in einem Bild
void grassfire(unsigned char img[MAXYDIM][MAXXDIM]) {
	char burn[MAXYDIM][MAXXDIM];
	//Initialisierung des hilfsarrays
	for (int i = 0; i < MAXXDIM; i++)
		for (int j = 0; j < MAXXDIM; j++)
			burn[i][j] = UNDEF;

	// Objekt counter
	int object = 0;
	int burning = 0;
	for (int x = 1; x < MAXXDIM - 1; x++) {
		for (int y = 1; y < MAXYDIM - 1; y++) {
			if (img[x][y] == 255 && burn[x][y] == UNDEF)
			{
				object++;
				burning = 1;
				burn[x][y] = BURN;
				while (burning == 1)
				{
					burning = 0;
					for (int a = 1; a < MAXXDIM - 1; a++) {
						for (int b = 1; b < MAXYDIM - 1; b++) {
							if (burn[a][b] == BURN)
							{
								burning = 1;
								burn[a][b] = BURNED;
								if (img[a + 1][b] == 255 && burn[a + 1][b] == UNDEF)
									burn[a + 1][b] = BURN;

								if (img[a - 1][b] == 255 && burn[a - 1][b] == UNDEF)
									burn[a + 1][b] = BURN;

								if (img[a][b+1] == 255 && burn[a][b + 1] == UNDEF)
									burn[a][b + 1] = BURN;

								if (img[a][b - 1] == 255 && burn[a][b - 1] == UNDEF)
									burn[a][b - 1] = BURN;
							}
						}
					}
				}
			}
		}
	}
	//printf("Das Bild enthaelt %i Objekte\n", object);
	//printf("Druecken Sie eine beliebige Taste...");
	fflush(stdin);
	getch();
}

// Zählt weiße Pixel im Bild
void count_white(unsigned char img[MAXYDIM][MAXXDIM]) {
	int i,j,grey = 0;
	for (i = 0; i < MAXXDIM; i++) 
		for (j = 0; j < MAXYDIM; j++) 
			if (img[i][j] != 0) 
				grey++;
	system("cls");
	//printf("Die Anzahl der nicht schwarzen Pixel im Bild ist: %i\n", grey);
	//printf("Druecken Sie eine beliebige Taste...");
	fflush(stdin);
	getch();
}

*/



/***********************************************************************************************************/
/*                      Preprocessing                                                                      */
/***********************************************************************************************************/
// berechne normales Histogramm
void calc_absolut_histo(unsigned char img[MAXYDIM][MAXXDIM], int grey[PIXEL_DEPTH])
{
	for (int a = 0; a < PIXEL_DEPTH; a++)
		grey[a] = 0;
	for (int i = 0; i < MAXXDIM; i++)
		for (int j = 0; j < MAXXDIM; j++)
			grey[img[j][i]]++;
}
void calc_rel_histo(unsigned char img[MAXYDIM][MAXXDIM], float grey[PIXEL_DEPTH])
{
	float div = (float)(MAXXDIM*MAXYDIM);
	for (int a = 0; a < PIXEL_DEPTH; a++)
		grey[a] = 0;
	for (int i = 0; i < MAXXDIM; i++)
		for (int j = 0; j < MAXXDIM; j++)
			grey[img[j][i]]+= 1.0;
	for (int g = 0; g < PIXEL_DEPTH; g++)
		grey[g] /= div;
}

// Berechne das komulative Histogramm (monoton steigend)
void calc_kumulativ_histo(unsigned char img[MAXYDIM][MAXXDIM], int grey[PIXEL_DEPTH])
{
	calc_absolut_histo(img, grey);
	for (int i = 1; i < PIXEL_DEPTH; i++)
		grey[i] = grey[i - 1] + grey[i];
}

//berechnet und schreibt ein normales/kumulatives Histogramm
void histogramm(unsigned char img[MAXYDIM][MAXXDIM], int ART) {
	int min = 255, max = 0, min_index = 0, max_index = 0;
	int grey[PIXEL_DEPTH];
	if (ART == HISTO_NORMAL)
		calc_absolut_histo(img, grey);
	else if (ART == HISTO_KUMULATIV)
		calc_kumulativ_histo(img, grey);
	//minimale Anzahl eines Farbwertes (au�er der Anzahl 0)
	for (int b = 0; b < PIXEL_DEPTH; b++)
		if (grey[b] < min && grey[b] > 0) {
			min = grey[b];
			min_index = b;
		}
	//maximale Anzahl eines Farbwertes 
	for (int c = 0; c < PIXEL_DEPTH; c++)
		if (grey[c] > max) {
			max = grey[c];
			max_index = c;
		}
	// Faktor berechnen f�r 256 Farbwerte
	float faktor = (float)MAXYDIM / (float)max;
	//initialisierungdes histogramms
	float skaliert[PIXEL_DEPTH];
	for (int a = 0; a < PIXEL_DEPTH; a++)
		skaliert[a] = 0;
	for (int i = 0; i < PIXEL_DEPTH; i++)
		skaliert[i] = (float)grey[i] * faktor;
	init_cMatrix(img, 255);

	for (int x = 0; x < PIXEL_DEPTH; x++)	// alle pixelwerte durchgehen in horizontaler richtung
		for (int y = MAXYDIM; y > 0; y--)
		{
			if (y >(MAXYDIM - (int)skaliert[x]))
				img[y][x] = 0;
			if (skaliert[x] < 1 && skaliert[x] > 0)
				img[MAXYDIM][x] = 0;
		}
}


//Grauwerte des Bildes werden auf den vollen Umfang an Grauwerten gedehnt
void grauwert_dehnung(unsigned char img[MAXYDIM][MAXXDIM]) {
	//system("cls");
	// kleinster Grauwert
	int kl_grau = 0, gr_grau = 0;
	int grey[PIXEL_DEPTH];
	calc_absolut_histo(img, grey);
	// In Abbildung des Histogramms in den y-Zellen[255] nach pixeln suchen
	// kleinster Grau-Wert
	for (int a = 0; a < MAXXDIM; a++)
		if (grey[a] != 0){
			kl_grau = a;
			break;
		}
	//gr��ter Grau-Wert
	for (int a = 255; a > 0; a--)
		if (grey[a] != 0){
			gr_grau = a;
			break;
		}
	// Aufweiten der Grauwerte auf die maximale Breite
	for (int x = 0; x < MAXXDIM; x++)
		for (int y = 0; y < MAXYDIM; y++)
			img[y][x] = (int)((float)255 / (float)(gr_grau - kl_grau) * (float)(img[y][x] - kl_grau));
}


//linearer Hisotgrammauslgleich 
// Quelle: Digitale Bildverarbeitung - Eine algorithmische Einf�hrung mit Java
// Autor: Wilhelm Burger, Mark James Burge
void linearer_histo_ausgleich(unsigned char img[MAXYDIM][MAXXDIM], int anzGrauWerte)
{
	int grey[PIXEL_DEPTH];
	calc_kumulativ_histo(img, grey);
	int surface = MAXXDIM * MAXYDIM;
	// linearer Ausgleich des Bildes
	for (int x = 0; x < MAXXDIM; x++)
		for (int y = 0; y < MAXYDIM; y++)
		{
			// Formel aus Quelle ( �berf�hrt von Java nach c++)
			int new_px = grey[img[y][x]] * (anzGrauWerte - 1) / (surface);
			img[y][x] = new_px > 255 ? 255 : new_px;
		}
	// Aufweiten der Grauwerte auf die maximale Breite
	for (int x = 0; x < MAXXDIM; x++)
		for (int y = 0; y < MAXYDIM; y++)
			img[y][x] = (int)((float)(PIXEL_DEPTH-1) / (float)(anzGrauWerte - 1) * (float)(img[y][x]));
}


/***********************************************************************************************************/
/*                      Filterfunktionen                                                                   */
/***********************************************************************************************************/
//Medianilter mit quadratischer Dimension der Form n*n, wobei n GERADE UND UNGERADE sein darf!
void median_filter(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], int iDIMxy)
{
	// Sicherheitsabfrage, ob Eingabe ok
	if (iDIMxy < 2 || iDIMxy > 11)
		return;
	//memcpy(img2,img, sizeof(img));
	for(int x = 0; x < MAXXDIM; x++)for(int y = 0; y < MAXYDIM; y++)img2[y][x] = img[y][x];
	int MedianSort[11 * 11];
	int z2 = iDIMxy * iDIMxy;

	int i0 = iDIMxy / 2 - (iDIMxy % 2 == 0 ? 1 : 0);
	int ic = MAXXDIM - iDIMxy / 2;
	int j0 = iDIMxy / 2 - (iDIMxy % 2 == 0 ? 1 : 0);
	int jc = MAXYDIM - iDIMxy / 2;

	int gerade = iDIMxy % 2;
	int mitte = iDIMxy*iDIMxy / 2;
	int iDIxy2 = (iDIMxy / 2);

	// Anfangswerte setzen je nach gerader/ungerader Filtermatrix und Gr��e der Matrix
	for (int i = i0; i < ic; i++) {
		for (int j = j0; j < jc; j++){
			memset(MedianSort, 0, sizeof(MedianSort));
			int x0 = i - i0;
			int xc = i + iDIxy2;
			int y0 = j - i0;
			int yc = j + iDIxy2;
			// Anfangswerte setzen, Randproblem beachten, da sonst Zugriff auf nicht indizierte Zellen.
			for (int x = x0, counter = 0; x <= xc; ++x)
				for (int y = y0; y <= yc; ++y)
					MedianSort[counter++] = img2[y][x];
			bubblesort(MedianSort, z2);
			// Wenn gerade Matrix, dann Mittelwert aus den mittleren Beiden werten als Median bilden
			img[j][i] =  gerade == 0 ? ((MedianSort[mitte] + MedianSort[mitte - 1]) / 2) : MedianSort[mitte];
		}
	}
}

void median_filter3x3(unsigned char img[MAXYDIM][MAXXDIM]){
	unsigned char img2[MAXYDIM][MAXXDIM];
	int MedianSort[3 * 3];
	for(int x = 0; x < MAXXDIM; x++)for(int y = 0; y < MAXYDIM; y++)img2[y][x] = img[y][x];
	for(int x = 1; x < (MAXXDIM-1); x++){
		for(int y = 1; y < (MAXYDIM-1); y++){
			for(int xx = x-1, counter = 0; xx <= x+1; xx++){
				for(int yy = y-1; yy <= y+1; yy++){
					MedianSort[counter++] = img2[yy][xx];
				}
			}
			// Bubblesort nach der hälfte abbrechen um rechenzeit zu sparen. Es wird nur der median benötigt

			for (int i = 0; i < 5 - 1; ++i)
				for (int j = 0; j < 9 - i - 1; ++j)
					if (MedianSort[j] > MedianSort[j + 1]) {
						int tmp = MedianSort[j];
						MedianSort[j] = MedianSort[j + 1];
						MedianSort[j + 1] = tmp;
					}
			//bubblesort(MedianSort, 9);
			img[y][x] =  MedianSort[4];
		}
	}
}

//Mittelwertfiler f�r Filter-Matrizen der Form n*n, wobei n GERADE UND UNGERADE sein darf!
void mittelwert_filter(unsigned char img[MAXYDIM][MAXXDIM], int iDIMxy, int Gewichtung)
{
	// Sicherheitsabfrage, ob Eingabe ok
	if (iDIMxy < 2 || iDIMxy > 11)
		return;
	unsigned char img2[MAXYDIM][MAXXDIM];
	//init_cMatrix(img2,255);
	//memcpy(img2,img, sizeof(img));
	for(int x = 0; x < MAXXDIM; x++)for(int y = 0; y < MAXYDIM; y++)img2[y][x] = img[y][x];
	uint32_t zw;
	// Anfangswerte setzen je nach gerader/ungerader Filtermatrix und Gr��e der Matrix
	int i0 = iDIMxy / 2 - (iDIMxy % 2 == 0 ? 1 : 0);
	int ic = MAXXDIM - iDIMxy / 2;
	int j0 = iDIMxy / 2 - (iDIMxy % 2 == 0 ? 1 : 0);
	int iDIxy2 = (iDIMxy / 2);
	int gerade = iDIMxy % 2;
	for (int i = i0; i < ic; i++) {
		for (int j = j0; j < ic; j++)
		{
			int x0 = i - i0;
			int xc = i + iDIxy2;
			int y0 = j - i0;
			int yc = j + iDIxy2;
			// Anfangswerte setzen je nach gerader/ungerader Filtermatrix, Randproblem beachten
			for (int x = x0, a = zw = 0; x <= xc; ++x, ++a) {
				for (int y = y0, b = 0; y <= yc; ++y, ++b) {
					if (gerade == 0)
						// Wenn a == b und iDIMxy/2-1 erreicht ist, ist dies der Mittelpunkt pixel f�r gerade Matrix
						zw += a == b && a == (iDIMxy / 2 - 1) ? img2[y][x] * Gewichtung : img2[y][x];
					else
						//  Wenn a == b und iDIMxy/2 erreicht ist, ist dies der Mittelpunkt pixel f�r ungerade Matrix
						zw += a == b && a == (iDIMxy / 2) ? img2[y][x] * Gewichtung : img2[y][x];
				}
			}
			// Mittelwert bilden
			zw /= (iDIMxy*iDIMxy) + (Gewichtung - 1);
			img[j][i] = (unsigned char)(zw > 255 ? 255 : zw < 0 ? 0 : zw);
		}
	}
	////writeImage_ppm(img2, MAXXDIM, MAXYDIM);
}
/*
// Gaussfilter: img - Eingabebild, img2 - Ausgabebild, scale - Gr��e des Filters
void gauss_filter(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], int scale)
{
	// Sicherheitsabfrage
	if (scale > 50 || scale < 3 || scale % 2 == 0)
		return;
	float bin_ver[50];
	for (int i = 0; i < 50; i++)
		bin_ver[i] = 0;
	// Hole binomialkoeffizienten (Summe normiert auf 100)
	get_bin_koeff(bin_ver, scale - 1, 1.0);
	// dynamisch Speicher reservieren, direkt mit 0 initialisiert
	float **gauss_filter = (float**)calloc(scale + 3, sizeof(float*));
	for (int i = 0; i < scale + 3; i++)
		gauss_filter[i] = (float*)calloc(scale + 3, sizeof(float));
	// 2D-Gauss-Filtermatrix berechnen
	for (int x = 0, a = 0; x < scale; x++)
		for (int y = 0; y < scale; y++, a++)
			gauss_filter[x][y] = bin_ver[x] * bin_ver[y];
	//Filter auf Bild anwenden
	//int iimg[MAXYDIM][MAXXDIM];
	//init_iMatrix(iIMG, 0);
	// Anfangswerte setzen je nach gerader/ungerader Filtermatrix und Gr��e der Matrix
	for (int i = scale / 2; i < MAXXDIM - (scale / 2); i++) {
		for (int j = scale / 2; j < MAXYDIM - (scale / 2); j++)
		{
			// zwischenwert der Filterposition
			float zw = 0.0;
			// Anfangswerte setzen je nach gerader/ungerader Filtermatrix, Randproblem beachten
			for (int x = i - (scale / 2), a = 1; x <= i + (scale / 2); x++, a++)
				for (int y = j - (scale / 2), b = 1; y <= j + (scale / 2); y++, b++)
					zw += gauss_filter[a][b] * (float)img[x][y];
			iIMG[i][j] = (int)zw;
		}
	}
	// Speicher freigeben
	for (int i = 0; i < scale; i++)
		free(gauss_filter[i]);
	free(gauss_filter);
	//init_cMatrix(img2, 0);
	memcpy(&img2, &img, sizeof(img));

	//for (int x = 0; x < MAXXDIM; x++)
	//	for (int y = 0; y < MAXYDIM; y++)
	//		img2[x][y] = img[x][y];

	for (int x = (scale / 2); x < MAXXDIM - (scale / 2); x++)
		for (int y = (scale / 2); y < MAXYDIM - (scale / 2); y++)
			img2[x][y] = (unsigned char)(iIMG[x][y] > 255 ? 255 : iIMG[x][y] < 0 ? 0 : iIMG[x][y]);
	////writeImage_ppm(img2, MAXXDIM, MAXYDIM);
}
*/

/***********************************************************************************************************/
/*                      Kantendetektion                                                                    */
/***********************************************************************************************************/

// Erste Ableitung in x Richtung, Sobeloperator
void sobelx(unsigned char img[MAXYDIM][MAXXDIM], int16_t sobelx[MAXYDIM][MAXXDIM])
{
	//init_cMatrix(img2, (PIXEL_DEPTH / 2));
	unsigned char px_dp2 = PIXEL_DEPTH / 2;
	//init_iMatrix(sobelx,0);
	for (int x = 1; x < MAXXDIM - 1; x++) {
		for (int y = 1; y < MAXYDIM - 1; y++) {
			// first instruction initializes the array cell
			sobelx[y][x] = img[y][x + 1] * -2;
			// remaining just add its value to it
			sobelx[y][x] += img[y][x - 1] * 2;
			sobelx[y][x] += img[y + 1][x + 1] * -1;
			sobelx[y][x] += img[y - 1][x - 1];
			sobelx[y][x] += img[y - 1][x + 1] * -1;
			sobelx[y][x] += img[y + 1][x - 1];
		}
	}
	// Pixelwerte skalieren und Bild schreiben
	int max = find_abs_extremum_iMatrix(MAX, sobelx);
	// Skalierung 0<127<255
	float faktor;
	faktor = (float)(PIXEL_DEPTH / 2) / (float)max;
	for (int x = 1; x < MAXXDIM - 1; x++)
		for (int y = 1; y < MAXYDIM - 1; y++)
			img[y][x] = px_dp2 + (signed char)((float)sobelx[y][x] * faktor);
	// ränder einfärben
	for(int x = 0; x < MAXXDIM; x++){
		img[0][x] = px_dp2;
		img[MAXYDIM][x] = px_dp2;
	}
	for(int y = 0; y < MAXYDIM; y++){
		img[y][0] = px_dp2;
		img[y][MAXXDIM] = px_dp2;;
	}

}
// Erste Ableitung in y-Richtung, Sobeloperator
void sobely(unsigned char img[MAXYDIM][MAXXDIM], int16_t sobely[MAXYDIM][MAXXDIM]) {
	//init_cMatrix(img2, (PIXEL_DEPTH / 2));
	unsigned char px_dp2 = PIXEL_DEPTH / 2;
	//init_iMatrix(sobely,0);
	for (int x = 1; x < MAXXDIM - 1; x++) {
		for (int y = 1; y < MAXYDIM - 1; y++) {
			sobely[y][x] = img[y + 1][x] * -2;
			sobely[y][x] += img[y - 1][x] * 2;
			sobely[y][x] += img[y + 1][x + 1] * -1;
			sobely[y][x] += img[y - 1][x - 1];
			sobely[y][x] += img[y - 1][x + 1];
			sobely[y][x] += img[y + 1][x - 1] * -1;
		}
	}
	// Pixelwerte skalieren und Bild schreiben
	int max = find_abs_extremum_iMatrix(MAX, sobely);
	// Skalierung 0<127<255
	float faktor;
	faktor = (float)(PIXEL_DEPTH / 2) / (float)max;
	for (int x = 1; x < MAXXDIM - 1; x++)
		for (int y = 1; y < MAXYDIM - 1; y++)
			img[y][x] = px_dp2 + (signed char)((float)sobely[y][x] * faktor);
	// Ränder
	for(int x = 0; x < MAXXDIM; x++){
		img[0][x] = px_dp2;
		img[MAXYDIM][x] = px_dp2;;
	}
	for(int y = 0; y < MAXYDIM; y++){
		img[y][0] = px_dp2;
		img[y][MAXXDIM] = px_dp2;;
	}
}

// Kobination der Operatoren Sobel in x und y Richtung
void sobelxy(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], int sobelx[MAXYDIM][MAXXDIM], int sobely[MAXYDIM][MAXXDIM]) {
	init_cMatrix(img2, (PIXEL_DEPTH / 2));
	int iIMG[MAXYDIM][MAXXDIM];
	init_iMatrix(iIMG,0);
	for (int i = 0; i < MAXXDIM; i++)
		for (int j = 0; j < MAXYDIM; j++)
			iIMG[i][j] = sqrt(pow(sobely[i][j], 2) + pow(sobelx[i][j], 2));
	// Pixelwerte skalieren und Bild schreiben
	int max = find_abs_extremum_iMatrix(MAX, iIMG);
	// Skalierung 0<127<255
	float faktor;
	faktor = (float)(PIXEL_DEPTH - 1) / (float)max;
	for (int x = 1; x < MAXXDIM - 1; x++)
		for (int y = 1; y < MAXYDIM - 1; y++)
			img2[x][y] = (unsigned char)((float)iIMG[x][y] * faktor);

}

// zweite Ableitung durch Laplace Operator
void laplace(unsigned char img[MAXYDIM][MAXXDIM], uint16_t framebuffer[MAXYDIM][MAXXDIM], int Umgebung) {
	unsigned char px_dp2 = PIXEL_DEPTH / 2;
	uint16_t half_uint16_t= 0xFFFF / 2;
	// Matrix der Laplace Operation berechnen
	//Filter auf Bild anwenden
	//init_iMatrix(iIMG,0);
	// Ränder initialiseren
	for(int x = 0; x < MAXXDIM; x++){
		for(int y = 0; y < MAXYDIM; y++){
			framebuffer[y][x] = half_uint16_t;
		}
	}
	//uint16_t max = find_abs_extremum_uiMatrix(MAX, framebuffer);
	//uint16_t min = find_abs_extremum_uiMatrix(MIN, framebuffer);
	for (int y = 1; y < MAXYDIM- 1; y++) {
		for (int x = 1; x < MAXXDIM - 1; x++) {
			if (Umgebung == LAPLACE_4) {
				framebuffer[y][x] = half_uint16_t + img[y - 1][x];
				framebuffer[y][x] += img[y][x - 1] + (img[y][x] * (-4)) + img[y][x + 1];
				framebuffer[y][x] += img[y + 1][x];
			}
			else if (Umgebung == LAPLACE_8)
			{
				framebuffer[y][x] = half_uint16_t + img[y - 1][x - 1] + img[y - 1][x] + img[y - 1][x + 1];
				framebuffer[y][x] += img[y][x - 1] + (img[y][x] * (-8)) + img[y][x + 1];
				framebuffer[y][x] += img[y + 1][x - 1] + img[y + 1][x] + img[y + 1][x + 1];
			}
		}
	}
	// Pixelwerte skalieren und Bild schreiben
	uint16_t max = find_abs_extremum_uiMatrix(MAX, framebuffer);
	uint16_t min = find_abs_extremum_uiMatrix(MIN, framebuffer);
	float faktor;
	if( (max - half_uint16_t) > (half_uint16_t - min)){
		if( (half_uint16_t - min) > 0)
			faktor = (float)(PIXEL_DEPTH / 2) / ((float)(half_uint16_t - min));
		else
			faktor = 0.0;
	}
	else
	{
		if( (max - half_uint16_t) > 0)
			faktor = (float)(PIXEL_DEPTH / 2) / ((float)(max - half_uint16_t));
		else
			faktor = 0.0;
	}
	// Skalierung 0<127<255

	faktor = (float)(PIXEL_DEPTH / 2) / ((float)max - (float)min);
	for (int x = 1; x < MAXXDIM - 1; x++){
		for (int y = 1; y < MAXYDIM - 1; y++){
			signed char tmp = (signed char)(((float)(framebuffer[y][x] - half_uint16_t))  * faktor);
			unsigned char erg = px_dp2 + tmp;
			img[y][x] = erg;
		}
	}
	// Ränder
	for(int x = 0; x < MAXXDIM; x++){
		img[0][x] = px_dp2;
		img[MAXYDIM][x] = px_dp2;
	}
	for(int y = 0; y < MAXYDIM; y++){
		img[y][0] = px_dp2;
		img[y][MAXXDIM] = px_dp2;
	}
}
// Difference of gaussian
void difference_of_gaussian(unsigned char img[MAXYDIM][MAXXDIM], uint16_t iIMG[MAXYDIM][MAXXDIM], int scale, int grundton)
{
	// TODO: reduce ram usage
	return ; // TOO LESS RAM...
#if RAM > 320000
	// Sicherheitsabfrage
	if (scale > 50 || scale < 3 || scale % 2 == 0)
		return;
	float bin_ver[2][50], diff[50];
	uint16_t x7fff = 0xFFFF /2;
	for (int i = 0; i < 50; i++)
		bin_ver[0][i] = bin_ver[1][i] = diff[i] = 0;
	// Hole binomialkoeffizienten (Summe normiert auf 100)
	get_bin_koeff(bin_ver[0], scale - 1, 100.0);
	get_bin_koeff(bin_ver[1], scale - 3, 100.0);
	// Differenz der Koeffizienten in 1D
	for (int i = 0; i < scale - 1; i++)
	{
		if (i == 0 || i == scale - 1)
		{
			diff[i] = bin_ver[0][i];
			continue;
		}
		diff[i] = bin_ver[0][i + 1] - bin_ver[1][i];
	}
	// dynamisch Speicher reservieren, direkt mit 0 initialisiert
	float **diff_of_gauss = (float**)calloc(scale + 3, sizeof(float*));
	for (int i = 0; i < scale + 3; i++)
		diff_of_gauss[i] = (float*)calloc(scale + 3, sizeof(float));
	// 2D-Gauss-Filtermatrix berechnen
	for (int x = 0, a = 0; x < scale; x++)
		for (int y = 0; y < scale; y++, a++)
			diff_of_gauss[x][y] = diff[x] * diff[y];
	//Filter auf Bild anwenden
	for(int x = 0; x < MAXXDIM; x++)
		for(int y = 0; y < MAXYDIM; y++)
			iIMG[y][x] = x7fff;
	// Anfangswerte setzen je nach gerader/ungerader Filtermatrix und Gr��e der Matrix
	for (int i = scale / 2; i < MAXXDIM - (scale / 2); i++) {
		for (int j = scale / 2; j < MAXYDIM - (scale / 2); j++)
		{
			// zwischenwert der Filterposition
			float zw = 0.0;
			// Anfangswerte setzen je nach gerader/ungerader Filtermatrix, Randproblem beachten
			for (int x = i - (scale / 2), a = 1; x <= i + (scale / 2); x++, a++) {
				for (int y = j - (scale / 2), b = 1; y <= j + (scale / 2); y++, b++) {
					 zw += diff_of_gauss[a][b] * (float)img[y][x];
				}
			}
			iIMG[j][i] = x7fff + (int16_t)(zw/10);
		}
	}
	// Speicher freigeben
	for (int i = 0; i < scale; i++)
		free(diff_of_gauss[i]);
	free(diff_of_gauss);
	grundton = grundton < 0 ? 0 : grundton > 255 ? 255 : grundton;
	init_cMatrix(img, grundton);
	// Pixelwerte skalieren und Bild schreiben
	uint16_t max  = find_abs_extremum_uiMatrix(MAX, iIMG);
	uint16_t min  = find_abs_extremum_uiMatrix(MIN, iIMG);

	max = x7fff - min > max - x7fff ? x7fff - min : max - x7fff;
	// div durch 0 verhindern
	if(max == 0)
		return;

	// Skalierung 0<127<255
	float faktor;
	faktor = grundton == 0 || grundton == 255 ? (float)255 / (float)max : (float)grundton / (float)max;
	for (int x = 1; x < MAXXDIM - 1; x++)
		for (int y = 1; y < MAXYDIM - 1; y++)
		{
			if (grundton != 0 && grundton != 255)
				img[x][y] += (unsigned char)((float)(iIMG[x][y]-x7fff) * faktor);
			else if (grundton == 0){
				float tmp = (float)(iIMG[x][y]-x7fff) * faktor;
				if(tmp < 0)
					tmp *= -1;
				if( tmp > PIXEL_DEPTH-1)
					tmp = PIXEL_DEPTH-1;
				img[x][y] = (unsigned char)tmp;
			}
			else if (grundton == 255){
				float tmp = (float)(iIMG[x][y]-x7fff) * faktor;
				img[x][y] = (unsigned char) tmp < 0? tmp * -1: tmp;
			}
		}
#endif
}


/***********************************************************************************************************/
/*                      Texturenerkennung                                                                  */
/***********************************************************************************************************/

// Texturenerkennung mit den Laws-Masken
void laws_textur(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM])
{
	// TODO: reduce ram usage
	return;
#if RAM > 320000
	init_cMatrix(img2,255);
	float fArray[MAXYDIM][MAXXDIM];
	init_fMatrix(fArray);

	int8_t masken[9][9] = {
		{-1,0,1,-2,0,2,-1,0,1},		// L3E3
		{-1,2,-1,-2,4,-2,-1,2,-1},	// L3S3
		{-1,2,-1,0,0,0,1,-2,1},		// E3L3
		{1,0,-1,0,0,0,-1,0,1},		// E3E3
		{1,-2,1,0,0,0,-1,2,-1},		// E3S3
		{-1,-2,-1,2,4,2,-1,-2,-1},	// S3L3
		{1,0,-1,-2,0,2,1,0,-1},		// S3E3
		{1,-2,1,-2,4,-2,1,-2,1},	// S3S3
		{1,2,1,2,4,2,1,2,1 }		// L3L3
	};
	for (int x = 1; x < MAXXDIM - 1; x++) {
		for (int y = 1; y < MAXYDIM - 1; y++) {
			// Werte aus dem Bild holen
			float zw[9] = { 0,0,0,0,0,0,0,0,0 };
			for (int mask = 0; mask < 8; mask++) // L3L3 wird nicht angewandt
				for (int i = x - 1,mask_val = 0; i <= x + 1; i++) 
					for (int j = y - 1; j <= y + 1; j++,mask_val++) 
						zw[mask] += ((float)masken[mask][mask_val] * (float)img[i][j]);
			for (int zw_mask = 0; zw_mask < 8; zw_mask++)		// zw_mask = zwischenwert der Maske
				fArray[x][y] += zw[zw_mask]* zw[zw_mask];		// Alle quadratisch addieren
			fArray[x][y] = sqrt(fArray[x][y]);	// Qurzel ziehen //Pytagoras
		}
	}
	float max = find_abs_extremum_fMatrix(MAX, fArray);			// vom Betrag her das Max finden
	float faktor = (float)PIXEL_DEPTH / max;					// Skalierung 0-255
	for (int x = 1; x < MAXXDIM - 1; x++)
		for (int y = 1; y < MAXYDIM - 1; y++)
			img2[x][y] = (unsigned char)(fArray[x][y] * faktor);
#endif
}

// Statistik 2. Ordnung
void cooccurence_matrix(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], float fimg[MAXYDIM][MAXXDIM], int direction, int save)
{
	// TODO: reduce ram usage
	return;
#if RAM > 320000
	init_cMatrix(img2, 255);
	if(save == 1)
		init_fMatrix(fIMG);
	int begin_x = direction != 135 ? 0 : 1, begin_y = direction == 135 ? 1 : 0;
	int end_x = direction == 0 || direction == 45 ? MAXXDIM - 1 : MAXXDIM, end_y = direction != 0 ? MAXYDIM - 1 : MAXYDIM;

	for (int x = begin_x; x < end_x; x++)
		for (int y = begin_y; y < end_y; y++)
			if (direction == 0) {
				fIMG[img[x + 1][y]][img[x][y]] += 1.0;
				fIMG[img[x][y]][img[x + 1][y]] += 1.0;
			}
			else if (direction == 45) {
				fIMG[img[x + 1][y + 1]][img[x][y]] += 1.0;
				fIMG[img[x][y]][img[x + 1][y + 1]] += 1.0;
			}
			else if (direction == 90) {
				fIMG[img[x][y + 1]][img[x][y]] += 1.0;
				fIMG[img[x][y]][img[x][y + 1]] += 1.0;
			}	
			else if (direction == 135) {
				fIMG[img[x - 1][y + 1]][img[x][y]] += 1.0;
				fIMG[img[x][y]][img[x - 1][y + 1]] += 1.0;
			}	
	if (save == 1)
	{
		int max = find_abs_extremum_fMatrix(MAX, fIMG);
		float faktor = (float)(PIXEL_DEPTH / 16) / (float)max;
		for (int x = 0; x < MAXXDIM; x++)
			for (int y = 0; y < MAXYDIM; y++) {
				float scale = (fIMG[x][y] * faktor);
				if (scale > 0.0 && scale < 1.0)
					scale = 1.0;
				int val = (int)scale * (PIXEL_DEPTH / 16);
				img2[x][y] = 255 - (unsigned char)(val < 0 ? 0 : val > (PIXEL_DEPTH - 1) ? (PIXEL_DEPTH - 1) : val);
			}
		//writeImage_ppm(img2, MAXXDIM, MAXYDIM);
	}
	float sum = 0.0;
	for (int i = 0; i < MAXXDIM; i++)
		for (int j = 0; j < MAXYDIM; j++)
			sum += fIMG[i][j];
#endif
}

void cooc_matrix_kombi_asm(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], float fimg[MAXYDIM][MAXXDIM], int graustufen)
{
	// TODO: reduce ram usage
	return;
#if RAM > 320000
	init_cMatrix(img2, 255);
	init_fMatrix(fIMG);
	for (int dir = 0; dir < 4; dir++)
		cooccurence_matrix(img, img2, fIMG, dir * 45, 0);
	// Mittelwert bilden
	for (int x = 0; x < MAXXDIM; x++)
		for (int y = 0; y < MAXYDIM; y++)
			fIMG[x][y] /= 4.0;

	int max = find_abs_extremum_fMatrix(MAX, fIMG);
	float faktor = (float)(PIXEL_DEPTH / graustufen) / (float)max;
	for (int x = 0; x < MAXXDIM; x++)
		for (int y = 0; y < MAXYDIM; y++) {
			float scale = (fIMG[x][y] * faktor);
			if (scale > 0.0 && scale < 1.0)
				scale = 1.0;
			int val = (int)scale * (PIXEL_DEPTH / graustufen);
			img2[x][y] = 255 - (unsigned char)(val < 0 ? 0 : val >(PIXEL_DEPTH - 1) ? (PIXEL_DEPTH - 1) : val);
		}
	//writeImage_ppm(img2, MAXXDIM, MAXYDIM);
	calc_asm_energie(fIMG);
#endif
}


void calc_asm_energie(float fIMG[MAXYDIM][MAXXDIM])
{
	// TODO: reduce ram usage
	return;
	float fASM = 0,sum = 0;
	for (int x = 0; x < MAXXDIM; x++)
		for (int y = 0; y < MAXYDIM; y++)
			sum += fIMG[x][y];
	for (int x = 0; x < MAXXDIM; x++)
		for (int y = 0; y < MAXYDIM; y++) 
			fASM += ((float)pow(fIMG[x][y] / sum, 2));
			
	////printf(" �  Anzahl der Eintraege in der Matrix:%20f   �\n", sum);
	////printf(" �  Der ASM Wert des Bildes betraegt:%22f   �\n",fASM);
	////printf(" �  Druecken Sie eine Taste um zurueck zum Menu zu gelangen.  �\n");

}

/***********************************************************************************************************/
/*                      Segmentierung                                                                      */
/***********************************************************************************************************/

uint16_t segmentierung_von_otsu(unsigned char img[MAXYDIM][MAXXDIM])
{

	// Verfahren von Otsu
	// optimaler Schwellwert
	uint16_t iSchwelle = 0;
	float dim_xy = (float)(MAXXDIM*MAXYDIM);
	float grey[PIXEL_DEPTH];
	calc_rel_histo(img, grey);
	// zwischenspeicher f�r Maximalen Quotienten
	float quotient = 0.0;
	// jeden Grauwert ausprobieren
	for (int g = 0; g < PIXEL_DEPTH; g++)
	{
		// arithmeithscer Mittelwert gesamt, Klasse 0, Klasse 1, Auftrittswahrscheinlichkeit Klasse 0, Auftrittswahrscheinlicheit Klasse 1
		float gx = 0, g0 = 0, g1 = 0, p0 = 0, p1 = 0;
		int counter = 0;
		// arithmetische Mittelwerte berechnen
		for (int x = 0; x < MAXXDIM; x++)
			for (int y = 0; y < MAXYDIM; y++)
			{
				gx += (float)img[x][y];
				if (img[x][y] <= g) {
					g0 += (float)img[x][y];
					counter++;
				}
				else
					g1 += (float)img[x][y];
			}
		gx /= (float)(dim_xy);				// arithmetischer Mittelwert aller Grauwerte
		g0 /= (float)counter;				// arithmeithscer Mittelwert der Grauwerte der Klasse 0
		g1 /= (float)(dim_xy - counter);	// arithmetischer Mittelwert der Grauwerte der Klasse 1

		for (int p = 0; p < g; p++)
			p0 += grey[p];					// Auftrittswahrscheinlichkeit der Klasse 0					
		p1 = 1.0 - p0;						// Auftrittswahrscheinlichkeit der Klasse 1

		float var0 = 0, var1 = 0;
		for (int i = 0; i < g; i++)
			var0 += pow(i - g0, 2) * grey[i];	// Varianz innerhalb der Klasse 0
		for (int i = g + 1; i < PIXEL_DEPTH; i++)
			var1 += pow(i - g1, 2) * grey[i]; // Varianz innerhalb der Klasse 1

		float varianz_zw = p0 * pow(g0 - gx, 2) + p1 * pow(g1 - gx, 2);	// Varainz zwischen den Klassen
		float varianz_in = p0 * var0 + p1 * var1;						// Varianz in den Klassen

		// Maximalen Qutienten zwischen Varianz in den Klassen zu Varianz zwischen den klassen suchen
		if ((varianz_zw / (varianz_in == 0.0 ? 1.0 : varianz_in)) > quotient) {
			quotient = (varianz_zw / (varianz_in == 0.0 ? 1.0 : varianz_in));
			iSchwelle = g;
		}
	}
	segmentierung_binaer(img, iSchwelle);
	return iSchwelle;
}

void segmentierung_binaer(unsigned char img[MAXYDIM][MAXXDIM], uint16_t threshold)
{
	for (int x = 0; x < MAXXDIM; x++)
		for (int y = 0; y < MAXYDIM; y++)
			img[y][x] = img[y][x] <= threshold ? 0 : 255;
}

void invert(unsigned char img[MAXYDIM][MAXXDIM]){
	for(int x = 0 ; x < MAXXDIM; x++)
		for(int y = 0; y < MAXYDIM; y++){

			img[y][x] = (255) - img[y][x];
		}
}


//Labling connected components in an image, where non-zero pixels are
// deemed as foreground, and will be labeled with an positive integer
// while background pixels will be labled with zeros.
//Input and output are 2D matrices of size h-by-w.
//Return maxLabel. Output labels are continuously ranged between [0,maxLabel).
//Assume each pixel has 4 neighbors.
//yuxianguo, 2018/3/27

int bwLabel(unsigned char img[MAXYDIM][MAXXDIM],uint16_t label[MAXYDIM][MAXXDIM], BlobColoring *ColInfo)
{
	//memset(label, 0, MAXBLOBS*sizeof(uint16_t));

	//link[i]:
	//(1) link label value "i" to its connected component (another label value);
	//(2) if link[i] == i, then it is a root.
	int16_t link[MAXBLOBS];
	int16_t lb = 1, x, y, a, b, t;
	link[0] = 0;
	//first row
	for(int i = 0; i < MAXBLOBS; i++)link[i] = 0;
	for(int i = 0; i < MAXXDIM; i++)for(int j = 0; j < MAXYDIM; j++)label[j][i] = 0;
	if(img[0][0]) {
		label[0][0] = lb;
		link[lb] = lb;
		lb++;
	}
	for(x = 1; x < MAXXDIM; x++)
		if(img[0][x]) {
			if(label[0][x - 1])
				label[0][x] = label[0][x - 1];
		else {
			label[0][x] = lb;
			link[lb] = lb;
			if(lb == MAXBLOBS-1)
				return -1;
			lb++;
		}
	}
	for(y = 1; y < MAXYDIM; y++) {
		if(img[y][0]) {
			if(label[y-1][x])
				label[y][0] = label[y-1][0];
			else {
				label[y][0] = lb;
				link[lb] = lb;
				if(lb == MAXBLOBS-1)
					return -1;
				lb++;
			}
		}
		for(x = 1; x < MAXXDIM; x++){
			if(img[y][x]) {
				a = label[y][x - 1];
				b = label[y-1][x]; //left & top
				if(a) {
					if(a == b)
						label[y][x] = a;
					else {
						//find root of a
						t = a;
						while(a != link[a])
							a = link[a];
						label[y][x] = link[t] = a;
						if(b) {
							//find root of b
							t = b;
							while(b != link[b])
								b = link[b];
							link[t] = b;
							//link b to a or link a to b, both fine
							if(a < b) link[b] = a; else link[a] = b;
						}
					}
				}
				else if(b) {
					//find root of b
					t = b;
					while(b != link[b])
						b = link[b];
					label[y][x] = link[t] = b;
				}
				else {
					//generate a new component
					label[y][x] = lb;
					link[lb] = lb;
					if(lb == MAXBLOBS-1)
						return -1;
					lb++;
				}
			}
		}
	}

	//Rearrange the labels with continuous numbers
	t = 1;
	for(x = 1; x < lb; x++)
		if(x == link[x]) {
			link[x] = -t; //using negative values to denote roots
			t++;
		}
	for(x = 1; x < lb; x++) {
		//find the root of x
		y = x;
		while(link[y] >= 0)
			y = link[y];
		//set the value of label x
		link[x] = link[y];
	}
	//Negative to positive
	for(x = 1; x < lb; x++)
		link[x] = -link[x];

	//Replace existing label values by the corresponding root label values
	//p = label;
	for(y = 0; y < MAXYDIM; y++)
		for(x = 0; x < MAXXDIM; x++)
			label[y][x] = link[label[y][x]];

	ColInfo->BlobCount = t;
	return 0; //num components (maxLabel + 1)
}

int bwLabelDeleteSmallBlobs(uint16_t label[MAXYDIM][MAXXDIM], int minBlobSize, BlobColoring *ColInfo){
	int16_t link[MAXBLOBS];
	int16_t x,y,w = MAXXDIM, h = MAXYDIM, lb = (int16_t)ColInfo->BlobCount,new_labels;
	//memset(link, 0, sizeof(int16_t) * MAXBLOBS);
	for(int i = 0; i < MAXBLOBS; i++)link[i] = 0;
	// calculate blobsizes
	for(x = 0; x < w; x++){
		for(y=0;y<h;y++){
			link[label[y][x]]++;
		}
	}
	// find biggest blob -> must be background
	int max = 0;
	int max_index = 0;
	for(int i = 0; i < lb; i++){
		if(link[i] > max){
			max = link[i];
			max_index = i;
		}
	}
	ColInfo->biggestBlobPxCount = max;
	// set labels to background if they are smaller as accepted, 1:background, 0: blob
	// mark too small blobs
	for(int i = 0; i < lb; i++){
		if(link[i]<minBlobSize){
			ColInfo->biggestBlobPxCount += link[i];
			link[i] = -link[i];
		}
	}
	// relabel accepted blobs
	new_labels = 0;
	for(int i = 0; i < lb; i++)
		if(link[i]>=0)
			link[i] = new_labels++;
	// set too small blobs to background
	for(int i = 0; i < lb; i++)
		if(link[i]<0)
			link[i] = link[max_index];

	// relabel blobmatrix
	for(x = 0; x < w; x++)
		for(y=0;y<h;y++)
			label[y][x] = link[label[y][x]];
	// return label of the biggest blob <-> background
	ColInfo->biggestBlobLabel = (unsigned int)link[max_index];
	ColInfo->BlobCount = new_labels;

	return 0;
}
// joins all blobs, that are not background
int bwLabelJoinBlobs(uint16_t label[MAXYDIM][MAXXDIM], BlobColoring *ColInfo){
	for(int x = 0; x < MAXXDIM; x++)
		for(int y = 0; y < MAXYDIM; y++)
				label[y][x] = label[y][x] != ColInfo->biggestBlobLabel ? 1 : 0;
	// nur noch 2 blob: hintergrund und vordergrund
	ColInfo->BlobCount = 2;
	ColInfo->biggestBlobPxCount = ColInfo->biggestBlobPxCount < (MAXXDIM*MAXYDIM/2) ?
		(MAXXDIM*MAXYDIM - ColInfo->biggestBlobPxCount) : ColInfo->biggestBlobPxCount;
	ColInfo->biggestBlobLabel = 1;
	return 0;
}

void labelMatrixToImage(uint16_t label[MAXYDIM][MAXXDIM], unsigned char img[MAXYDIM][MAXXDIM],BlobColoring *ColInfo){
	int x,y;
	float faktor = (float)(PIXEL_DEPTH - 1) / (float)(ColInfo->BlobCount-1);
	for (x = 0; x < MAXXDIM; x++)
		for (y = 0; y < MAXYDIM; y++)
			img[y][x] = (unsigned char)((float)label[y][x] * faktor);
}


int schwerpunkt(unsigned char img[MAXYDIM][MAXXDIM], Blob *s){
	//printf("blob label %u\n", s->blob_label);
	s->b.x2 = s->b.y2 = 0;
	s->b.x1 = MAXXDIM;
	s->b.y1 = MAXYDIM;
	float sx = 0, sy = 0;
	for (int x = 0; x < MAXXDIM; x++){
		for (int y = 0; y < MAXYDIM; y++){
			if((unsigned int)img[y][x] == s->blob_label){
				++s->A;
				// für boundary box
				s->b.x1 = x < s->b.x1 ? x : s->b.x1;
				s->b.x2 = x > s->b.x2 ? x : s->b.x2;
				s->b.y1 = y < s->b.y1 ? y : s->b.y1;
				s->b.y2 = y > s->b.y2 ? y : s->b.y2;
				// für flächenschwerpunkt
				// +0,5 weil der 0. Pixel einen abstand zu 0 von 0,5 hat.
				sx += (float)x+0.5;
				sy += (float)y+0.5;
			}
		}
	}
	if(s->A > 0){
		sx /= (float)s->A;
		sy /= (float)s->A;
	}
	s->s.fx = sx;
	s->s.fy = sy;
	s->s.x = (int)sx;
	s->s.y = (int)sy;
	return 0;
}

int zeige_schwerpunkt(unsigned char img[MAXYDIM][MAXXDIM],Blob *blob, unsigned char drawlabel){
	if(blob->A > 0){
		//Markiere Schwerpunkt im Bild
		for(int x = 0; x< MAXXDIM; x++)
			img[blob->s.y][x] = drawlabel;
		for(int y = 0; y< MAXXDIM; y++)
			img[y][blob->s.x] = drawlabel;

		// Boundary Box im Bild Markieren
		for(int x = blob->b.x1; x <= blob->b.x2; x++){
			img[blob->b.y1][x] = drawlabel;
			img[blob->b.y2][x] = drawlabel;
		}
		for(int y = blob->b.y1; y <= blob->b.y2; y++){
			img[y][blob->b.x1] = drawlabel;
			img[y][blob->b.x2] = drawlabel;
		}
		return 0;
	}
	else{
		return 1;
	}
}


int solveQuadricFunction(float a, float b, float c,float *n1, float *n2){

	float d;
	d = b * b - 4 * a * c;
	if(d < 0){
	// komplexe zahl- wurzel aus negativer zahl....
	//printf("%.3f%+.3fi",-b/(2*a),sqrt(-d)/(2*a));
	//printf(", %.3f%+.3fi",-b/(2*a),-sqrt(-d)/(2*a));
	return -1;
	}
	else if(d==0){
		// doppelte nullstelle
		*n1 = *n2 = -b /(2* a);
		return 0;
	}
	else{
		// zwei nullstellen
		*n1 = ( -b + sqrt(d)) / (2* a);
		*n2 = ( -b - sqrt(d)) / (2* a);
		return 0;
	}
}
/*
 * principal componente analysis
 * breaks down the pixel matrix to a 2x2 covariance matrix
 * out of this matrix, the eigenvalues and eigenvectors are calculated
 * with the vector the orientation of the blob is calculated
 */
int blobOrientationPCA(unsigned char img[MAXYDIM][MAXXDIM], Blob *blob){
	// evtl dynamisch allozieren um speicher zu sparen
	int iaverage_x = 0, iaverage_y = 0;
	int x, y;
	unsigned int cnt = 0;
	float fvar_x = 0, fvar_y = 0, fvar_xy = 0, ftmp1, ftmp2;
	float faverage_x = 0, faverage_y = 0;
	float alpha;

	for(x = blob->b.x1; x <= blob->b.x2; x++){
		for(y = blob->b.y1; y <= blob->b.y2; y++){
			if(img[y][x] == blob->blob_label){
				iaverage_x += x;
				iaverage_y += y;
				cnt++;
			}
		}
	}

	// Durchschnitt errechnen, auf 0,5 px genau
	faverage_x = (float)(iaverage_x / cnt);
	faverage_y = (float)(iaverage_y / cnt);


	// calculate variance xx, yy and xy/yx
	for(x = blob->b.x1; x <= blob->b.x2; x++){
		for(y = blob->b.y1; y <= blob->b.y2; y++){
			if(img[y][x] == blob->blob_label){
				ftmp1 = ((float)x - faverage_x);
				fvar_x += ftmp1 * ftmp1;
				ftmp2 = ((float)y - faverage_y);
				fvar_y += ftmp2 * ftmp2;
				fvar_xy += ftmp1*ftmp2;
			}
		}
	}
	fvar_x /= (cnt - 1);
	fvar_y /= (cnt - 1);
	fvar_xy/= (cnt - 1);


	if((fvar_x - fvar_y) != 0.0)
		blob->o = 0.5 * atan((2*fvar_xy)/(fvar_x - fvar_y));
	else
		blob->o = 0;
	blob->o_deg = blob->o *180 /M_PI;

	// die anderen wer
	return 0;

	// quadric function to solve eigenvalues: 0 = ax²+bx+c
	// det(a) = | (cov(xx) - l) cov(xy)      |
	//          |  cov(xy)     (cov(yy) - l) |
	// al²+bl+c = l² - (cov(xx) + cov(yy))*l + (cov(xx)*cov(yy)) - (cov(xy)*cov(xy))

	float a = 1.0;
	float b = -(fvar_x + fvar_y);
	float c = (fvar_x*fvar_y) - (fvar_xy*fvar_xy);
	float d = 0;
	float n1 = 0;	// eigenwert 1
	float n2 = 0;	// eigenwert 2
	if(solveQuadricFunction(a,b,c, &n1, &n2))
		return -1;	// quadratische funktion ergibt keine 2 lösungen

	a = fvar_x;
	b = fvar_xy;
	c = fvar_xy;
	d = fvar_y;

	// | a  b |  d*k=b => k=b/d   ->  | a  b | -II -> | a-ck  0  |
	// | c  d |                       | ck dk|        | ck    bk |
	//
	// (lambda*E -A)*x=0
	// | n1-a      b    | * |x| = |0|
	// | c         n1-d |   |y|   |0|
	//
	// y = ((n1-a)/b)*x  = (c/(n1-d))*x
	// v = {b, n1-a}, {n1-d,c}
	blob->v1.x = b;
	blob->v1.y = n1 - a;
	blob->v1.eigenval = n1;
	blob->v2.x = n2 - d;
	blob->v2.y = c;
	blob->v2.eigenval = n2;

	float tmp = sqrt((blob->v1.x*blob->v1.x)+(blob->v1.y*blob->v1.y));
	float tmp_x = blob->v1.x<0? -blob->v1.x : blob->v1.x;
	float tmp_y = blob->v1.y<0? -blob->v1.y : blob->v1.y;
	blob->v1.alpha =  blob->v1.x/tmp; //tmp_x/tmp;
	blob->v1.alpha = acos(blob->v1.alpha);
	blob->v1.alpha_deg = blob->v1.alpha*180/M_PI;

	blob->v1.beta = blob->v1.y/tmp;//tmp_y/tmp;
	blob->v1.beta = acos(blob->v1.beta);
	blob->v1.beta_deg = blob->v1.beta*180/M_PI;

	tmp = sqrt((blob->v2.x*blob->v2.x)+(blob->v2.y*blob->v2.y));
	tmp_x = blob->v2.x<0? -blob->v2.x : blob->v2.x;
	tmp_y = blob->v2.y<0? -blob->v2.y : blob->v2.y;
	blob->v2.alpha = blob->v2.x/tmp;//tmp_x/tmp;
	blob->v2.alpha = acos(blob->v2.alpha);
	blob->v2.alpha_deg = blob->v2.alpha*180/M_PI;

	blob->v2.beta = blob->v2.y/tmp;//tmp_y/tmp;
	blob->v2.beta = acos(blob->v2.beta);
	blob->v2.beta_deg = blob->v2.beta*180/M_PI;

	return 0;
}


void show_orientation(unsigned char img[MAXYDIM][MAXXDIM], Blob *blob, unsigned char draw_label){
	// Linie errechnen, die der rotation der Eigenvektoren entsprechen
	int a ,b,a2,b2, x0,y0,x1,y1;
	int xl = (blob->b.x2 - blob->b.x1)/2;
	int yl = (blob->b.y2 - blob->b.y1)/2;
	int radius = xl < yl ? xl : yl;

	drawCircle(img, (uint16_t)blob->s.x, (uint16_t)blob->s.y,(uint8_t)radius, draw_label);

	a = (int)(sin(blob->o)*(float)radius);	// delta y
	// cos(alpha)=b/c
	b = (int)(cos(blob->o)*(float)radius);	// delta x

	a2 = (int)(sin((M_PI/2)-blob->o)*(float)radius);	// delta y
	b2 = (int)(cos((M_PI/2)-blob->o)*(float)radius);

	if(blob->o > 0){
		x0 = blob->s.x - b;
		y0 = blob->s.y - a;
		x1 = blob->s.x + b;
		y1 = blob->s.y + a;
	}else{
		x0 = blob->s.x + b;
		y0 = blob->s.y + a;
		x1 = blob->s.x - b;
		y1 = blob->s.y - a;
	}
	drawLine(img,x0,y0,x1,y1,draw_label);

	if(blob->o > 0){
		x0 = blob->s.x - b2;
		y0 = blob->s.y + a2;
		x1 = blob->s.x + b2;
		y1 = blob->s.y - a2;
	}else{
		x0 = blob->s.x + b2;
		y0 = blob->s.y - a2;
		x1 = blob->s.x - b2;
		y1 = blob->s.y + a2;
	}
	drawLine(img,x0,y0,x1,y1,draw_label);
	char buf[21];
	int vk = (int)blob->o_deg;
	int nk = (int)((blob->o_deg - (float)vk)*1000);
	nk = nk < 0 ? -nk : nk;
	nk/=100;
	sprintf(buf, "%i.%i*", vk, nk);
	writeString(img,1, MAXYDIM - Font_7x10.height- 1,&buf,Font_7x10 );

	return;
}


/**
 * @brief Draw a line with single color
 * @param x1&y1 -> coordinate of the start point
 * @param x2&y2 -> coordinate of the end point
 * @param color -> color of the line to Draw
 * @return none
 */
void drawLine(unsigned char img[MAXYDIM][MAXXDIM], uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, unsigned char greyval) {
	uint16_t swap;
    uint16_t steep; 	// = ABS(y1 - y0) > ABS(x1 - x0);
    uint16_t a = y1 - y0;
    uint16_t b = x1 - x0;
    a = a < 0 ? -a : a;
    b = b < 0 ? -b : b;
    steep = a > b;

    if (steep) {
		swap = x0;
		x0 = y0;
		y0 = swap;
		swap = x1;
		x1 = y1;
		y1 = swap;
        //_swap_int16_t(x0, y0);
        //_swap_int16_t(x1, y1);
    }
    if (x0 > x1) {
		swap = x0;
		x0 = x1;
		x1 = swap;
		swap = y0;
		y0 = y1;
		y1 = swap;
        //_swap_int16_t(x0, x1);
        //_swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = y1 - y0;
    dy = dy < 0? -dy:dy;
    //dy = ABS(y1 - y0);
    int16_t err = dx / 2;
    int16_t ystep;
    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }
    for (; x0<=x1; x0++) {
        if (steep) {
        	img[x0][y0] = greyval;
            //ST7789_DrawPixel(y0, x0, color);
        } else {
        	img[y0][x0] = greyval;
            //ST7789_DrawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}


/**
 * @brief Draw a circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle line
 * @return  none
 */
void drawCircle(unsigned char img[MAXYDIM][MAXXDIM], uint16_t x0, uint16_t y0, uint8_t r, unsigned char  greyval)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;
	if( (x0-r) < 0 || 0 > (y0-r))
		return;

	if( (x0+r) > MAXXDIM || MAXYDIM < (y0+r))
		return;

	img[y0+r][x0] = greyval;
	img[y0-r][x0] = greyval;
	img[y0][x0+r] = greyval;
	img[y0-r][x0] = greyval;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		img[y0 + y][x0 + x] = greyval;
		img[y0 + y][x0 - x] = greyval;
		img[y0 - y][x0 + x] = greyval;
		img[y0 - y][x0 - x] = greyval;

		img[y0 + x][x0 + y] = greyval;
		img[y0 + x][x0 - y] = greyval;
		img[y0 - x][x0 + y] = greyval;
		img[y0 - x][x0 - y] = greyval;
	}
}


/**
 * @brief Write a char
 * @param  x&y -> cursor of the start point.
 * @param ch -> char to write
 * @param font -> fontstyle of the string
 * @param color -> color of the char
 * @param bgcolor -> background color of the char
 * @return  none
 */
void writeChar(unsigned char img[MAXYDIM][MAXXDIM], uint16_t x0, uint16_t y0, char ch, FontDef font)
{
	uint32_t b, x, y, y_n, x_n;
	for (y = y0, y_n = 0; y < y0 + font.height; y++, y_n++) {
		b = font.data[(ch - 32) * font.height + y_n];
		for (x = x0, x_n = 0; x < x0 + font.width; x++, x_n++) {
			if ((b << x_n) & 0x8000) {
				img[y][x] = 255;
			}
			else {
				img[y][x] = 0;
			}
		}
	}
}

/**
 * @brief Write a string
 * @param  x&y -> cursor of the start point.
 * @param str -> string to write
 * @param font -> fontstyle of the string
 * @param color -> color of the string
 * @param bgcolor -> background color of the string
 * @return  none
 */
void writeString(unsigned char img[MAXYDIM][MAXXDIM], uint16_t x, uint16_t y, const char *str, FontDef font)
{
	while (*str) {
		if (x + font.width >= MAXXDIM) {
			x = 0;
			y += font.height;
			if (y + font.height >= MAXYDIM) {
				break;
			}
			if (*str == ' ') {
				// skip spaces in the beginning of the new line
				str++;
				continue;
			}
		}
		writeChar(img, x, y, *str, font);
		x += font.width;
		str++;
	}
}









