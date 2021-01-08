

#include "globals.h"
#include "fonts.h"


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
} BoundaryBox;

typedef struct {
	float fx;	// Schwerpunkt x
	float fy;	// Schwerpunkt y
	int x;
	int y;
	//Box boundary_box;	// Boundary Box des Objektes
} Schwerpunkt;

typedef struct{
	double fIx;	// axiales Flächenmoment in x Richtung
	double fIy;	// axiales Flächenmoment in y Richtung
	double fIxy;	// biaxiales Flächenmoment in xy Richtung

	double orientation;	// drehung der Hauptachsen radiant
	double orientation_deg;	// drehung der Hauptachsen degrees
} Momente;

typedef struct{
	float x;
	float y;
	float eigenval;	// eigenwert
    float alpha;	// angle from x-axis to vector
    float alpha_deg;
    float beta;		// angle from y-axis to vector
    float beta_deg;
}Vertex;


typedef struct {
     unsigned int blob_label;
 	 unsigned int A;	// Fläche
     Momente m;
     BoundaryBox b;
     Schwerpunkt s;
     Vertex v1;	// Eigenvektor 1
     Vertex v2; // Eigenvektor 2
     float o; 	// Orientation in rad
     float o_deg;	// orientation in deg;
} Blob;


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
int zeige_schwerpunkt(unsigned char img[MAXYDIM][MAXXDIM],Blob *blob, unsigned char drawlabel);
int schwerpunkt(unsigned char img[MAXYDIM][MAXXDIM], Blob *s);
//int widerstandsmomente(unsigned char img[MAXXDIM][MAXYDIM],Blob *b);
int zeige_rotation(unsigned char img[MAXYDIM][MAXXDIM], Blob *blob);
//int blobOrientationMoments(Blob *blob);
int blobOrientationPCA(unsigned char img[MAXYDIM][MAXXDIM], Blob *blob);

// Annotation Functions
void show_orientation(unsigned char img[MAXYDIM][MAXXDIM], Blob *blob, unsigned char label);
void drawLine(unsigned char img[MAXYDIM][MAXXDIM], uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, unsigned char greyval) ;
void drawCircle(unsigned char img[MAXYDIM][MAXXDIM], uint16_t x0, uint16_t y0, uint8_t r, unsigned char  greyval);
void writeChar(unsigned char img[MAXYDIM][MAXXDIM], uint16_t x0, uint16_t y0, char ch, FontDef font);
void writeString(unsigned char img[MAXYDIM][MAXXDIM], uint16_t x, uint16_t y, const char *str, FontDef font);

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






