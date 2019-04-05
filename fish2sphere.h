#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <limits.h>
#include "bitmaplib.h"
#include "jpeglib.h"
#include "tiffio.h"
#include "tiff.h"
#include "camodocal/camera_models/CataCamera.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

typedef struct {
	int axis;
	double value;
	double cvalue,svalue;
} TRANSFORM;

typedef struct {
   double x,y,z;
} XYZ;

typedef struct {
	short int scale;
	short int i,j;
} MAP;

typedef struct {
	int fishwidth,fishheight;    // Fisheye input image dimensions
	int outwidth;                // Dimension of the output equirectangular image
	int antialias;               // Degree of antialiasing, 1=none, rarely any point above 3
	int fishcenterx,fishcentery; // Centre of the fisheye circle in the image
	int fishradius;              // The radius of the fisheye circle in the image
	double fishfov;              // Fisheye field of view
	TRANSFORM *transform;        // Transformations to correct for non-aligned optical axes
	int ntransform;
	double longblend1,longblend2;// Two longitude angles over which to do the blend
	double latblend1,latblend2;  // Two latitude angles for blending
	double latmax,longmax;       // +- bounds in longitude and latitude
	int rcorrection;             // Apply tru-theta lens correction or not
	double a1,a2,a3,a4;          // Tru-theta lens correction parameters
	BITMAP4 background;          // Background colour for output image;
	int stmap16;                 // Save STMap for Nuke
   int stmap32;
	int hammer;                  // Create Hammer projection

	// Internally managed
	int outheight;
	int inputformat;             // TGA or JPG, derived from the input image extension
   int makeobj;                 // Create textured mesh as obj file, one of three types
   int makeremap;               // Save PGM files for remap filter for ffmpeg
	int antialias2;              // Square of antialiasing
} VARS;

#define TRUE  1
#define FALSE 0

#define SQRT2 1.4142135624

#define XTILT 0 // x
#define YROLL 1 // y
#define ZPAN  2 // z

// The following are for obj generation
#define NOBJ 220
#define SPHERE     0
#define HEMISPHERE 1
#define APERTURE   2

// Prototypes
void GiveUsage(char *);
int FindFishPixel(double,double,int *,int *,double *,double *);
int FindFishPixel2(double,double,int *,int *,double *,double *,camodocal::CataCamera &cata_camera);
double GetTime(void);
void MakeObj(char *,int);
void MakeRemap(char *);
void InitVars(void);

