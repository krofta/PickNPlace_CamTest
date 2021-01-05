

#include "globals.h"


#define LAPLACE_4 4
#define LAPLACE_8 8
#define BINAER 0
#define BIT8 1
#define MIN 0
#define MAX 1

#define SKALIERT 0
#define NICHT_SKALIERT 1

// Strukturen zur vereinfachung

typedef struct {
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
} Box;

typedef struct {
	unsigned int x;	// Schwerpunkt x
	unsigned int y;	// Schwerpunkt y
	unsigned int A;	// Fläche
	Box boundary_box;	// Boundary Box des Objektes
} Schwerpunkt;

typedef struct{
	long int Ix;	// Widerstandsmoment in x Richtung
	long int Iy;	// Widerstandsmoment in y Richtung
	long int Ixy;	// Deviationsmoment in xy Richtung
} Momente;

typedef struct{
	unsigned int biggestBlobPxCount;
	unsigned int biggestBlobLabel;
	unsigned int BlobCount;
}BlobColoring;



// bin�re Bildverarbeitung
/*
void shrink(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM]);
void blow(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM]);
void funk_open(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM]);
void funk_close(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM]);
void grassfire(unsigned char img[MAXYDIM][MAXXDIM]);
void count_white(unsigned char img[MAXYDIM][MAXXDIM]);
*/

// Preprocessing
void histogramm(unsigned char img[MAXYDIM][MAXXDIM], int ART);
void grauwert_dehnung(unsigned char img[MAXYDIM][MAXXDIM]);
void linearer_histo_ausgleich(unsigned char img[MAXYDIM][MAXXDIM], int anzGrauWerte);
void calc_absolut_histo(unsigned char img[MAXYDIM][MAXXDIM], int grey[PIXEL_DEPTH]);
void calc_kumulativ_histo(unsigned char img[MAXYDIM][MAXXDIM], int grey[PIXEL_DEPTH]);
void median_filter(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], int iDIMxy);
void median_filter3x3(unsigned char img[MAXYDIM][MAXXDIM]);
void mittelwert_filter(unsigned char img[MAXYDIM][MAXXDIM], int iDIMxy, int Gewichtung);
//void gauss_filter(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], int scale);

// Kantendetektion

//void sobelx(unsigned char img[MAXYDIM][MAXXDIM], int16_t sobelx[MAXYDIM][MAXXDIM]);
void sobelx(unsigned char img[MAXYDIM][MAXXDIM], int16_t sobelx[MAXYDIM][MAXXDIM]);
void sobely(unsigned char img[MAXYDIM][MAXXDIM], int16_t sobely[MAXYDIM][MAXXDIM]);
// too less ram
//void sobelxy(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], int16_t sobelx[MAXYDIM][MAXXDIM], int16_t sobely[MAXYDIM][MAXXDIM]);
void laplace(unsigned char img[MAXYDIM][MAXXDIM],uint16_t framebuffer[MAXYDIM][MAXXDIM], int Umgebung);
void difference_of_gaussian(unsigned char img[MAXYDIM][MAXXDIM], uint16_t iIMG[MAXYDIM][MAXXDIM], int scale, int grundton);

// Texturen
void laws_textur(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM]);
void cooccurence_matrix(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], float fIMG[MAXYDIM][MAXXDIM], int direction, int save);
void cooc_matrix_kombi_asm(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], float fIMG[MAXYDIM][MAXXDIM], int graustufen);
void calc_asm_energie(float fIMG[MAXYDIM][MAXXDIM]);

//Segmentierung
uint16_t segmentierung_von_otsu(unsigned char img[MAXYDIM][MAXXDIM]);
void segmentierung_binaer(unsigned char img[MAXYDIM][MAXXDIM], uint16_t threshold);
void invert(unsigned char img[MAXYDIM][MAXXDIM]);

int bwLabel(unsigned char img[MAXYDIM][MAXXDIM],uint16_t label[MAXYDIM][MAXXDIM], BlobColoring *ColInfo);
int bwLabelDeleteSmallBlobs(uint16_t label[MAXYDIM][MAXXDIM], int minBlobSize, BlobColoring *ColInfo);
int bwLabelJoinBlobs(uint16_t label[MAXYDIM][MAXXDIM], BlobColoring *ColInfo);
void labelMatrixToImage(uint16_t label[MAXYDIM][MAXXDIM], unsigned char img[MAXYDIM][MAXXDIM],BlobColoring *ColInfo);




// Merkmalsextraktion
void zeige_schwerpunkt(unsigned char img[MAXYDIM][MAXXDIM],unsigned char bloblabel);
Schwerpunkt schwerpunkt(unsigned char img[MAXYDIM][MAXXDIM],unsigned char bloblabel);
Momente widerstandsmomente(unsigned char img[MAXYDIM][MAXXDIM],Schwerpunkt s, unsigned char bloblabel);
void zeige_rotation(unsigned char img[MAXYDIM][MAXXDIM], unsigned char bloblabel);
double orientierung(Momente m);
double winkel_rechteck(unsigned char img[MAXYDIM][MAXXDIM],Schwerpunkt s, unsigned int bloblabel);

// Anderes
void frambuffer_test(unsigned char cMatrix[MAXYDIM][MAXXDIM]);
void init_cMatrix(unsigned char cMatrix[MAXYDIM][MAXXDIM], unsigned char val);
void init_iMatrix(uint16_t iMatrix[MAXYDIM][MAXXDIM] , uint16_t val);
//void init_fMatrix(float fMatrix[MAXYDIM][MAXXDIM]);
int16_t find_abs_extremum_iMatrix(int16_t min_max, int16_t iMatrix[MAXYDIM][MAXXDIM]);
uint16_t find_abs_extremum_uiMatrix(uint16_t min_max, uint16_t uiMatrix[MAXYDIM][MAXXDIM]);
//float  find_abs_extremum_fMatrix(int min_max, float fMatrix[MAXYDIM][MAXXDIM]);
void get_bin_koeff(float bin_ver[50], int n, float normierung);
double fakultaet(int n);
void bubblesort(int *array, int length);
void reset_blob_label(uint16_t iIMG[MAXYDIM][MAXXDIM], uint16_t oldLabel, uint16_t newLabel);
void rgb_to_greyscale(uint16_t iIMG[MAXYDIM][MAXXDIM], unsigned char img[MAXYDIM][MAXXDIM]);






