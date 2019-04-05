#include "fish2sphere.h"
#include <fstream> 
#include <iostream>

int debug = FALSE;

/*
   Convert fisheye to spherical map (partial one anyway)
   On command line can vary size of the output image, antialiasing (supersampling)
   and field of view (of the fisheye in degrees).
   In the case where the fisheye is not an inscribed circle in a square then the
   centre and radius of the fisheye circle can be specified.
   Axis: x to the right, camera points in y, z up
   Note: performance is proportional to the number of pixels in the output image.
   Feb 2016: added support for latitude rotation of the fisheye.
   Feb 2016: added support for polar axis rotation, for the Ricoh Theta S
   Jul 2016: added edge blending between 2 latitudes for 360 formation
             added pan rotation, fishpan
             added order for transformations
   Aug 2016: added mesh export
             flipped the vertical axis, origin is now top left corner
   Aug 2016: Add mapping file to test realtime option
   Oct 2016: Added jpeg using small libjpeg, styled after tga calls
   Oct 2016: Added obj export as permanent
   Nov 2016: Add third order polynomial radial correction a1 x + a2 x^2 + a3 x^3
             Cleaned up parameters into a structure
   Feb 2017: Added remap support for ffmpeg
             For example: ffmpeg -i sample.jpg -i x.pgm -i y.pgm -lavfi remap sample.png
   Oct 2017: Changed lens correction to map from radians to normalised fisheye coordinates
             Added fourth order correction, needed for Entaniya 250 lens
   Feb 2018: Added clipping in longitude and latitude
             Add STMap support for Nuke image warping.
   Apr 2018: Added 32 bit stmap support, requires libtiff, see ADDTIFF in fish2sphere.h
   Oct 2018: Added Hammer projection, only for main conversion, not extras
*/

BITMAP4 *fisheye = NULL;
BITMAP4 *spherical = NULL;
BITMAP4 *tiff_final = NULL;
VARS vars;

void jpg2tiff_alfa();

int main(int argc,char **argv)
{
   camodocal::CataCamera::Parameters parameters;
   parameters.readFromYamlFile("camera_camera_calib.yaml");
   camodocal::CataCamera cata_camera(parameters);
   std::cout <<"--------"<< parameters.xi() << '\n';

   int i,j,k,u,v,aj,ai,w,h,depth;
   int index;
   char basename[128],fname[128];
   FILE *fptr;
   BITMAP4 black = {0,0,0,255};
   double z,x,y,fu,fv;
   double latitude,longitude;
   double latblend = 1,longblend = 1;
   int rsum,gsum,bsum;
   double starttime,stoptime=0;
  
#ifdef ADDTIFF
   COLOUR16 *stmap16 = NULL,black16 = {0,0,0};
   COLOUR32 *stmap32 = NULL,black32 = {0,0,0};
#endif

   InitVars();
   if (argc < 2) 
      GiveUsage(argv[0]);

   // Base output name on input name
   strcpy(basename,argv[argc-1]);
   for (k=0;k<(int)strlen(basename);k++)
      if (basename[k] == '.')
         basename[k] = '\0';

   // Parse the command line arguments 
   //const char fov = 200;
   vars.fishcenterx = parameters.u0();
   vars.fishcentery = parameters.v0();
   vars.outwidth = 2048;
   vars.outheight = 1024;
   vars.fishradius = 2304;
   cv::Mat mask,map;
   mask.create(vars.outheight,vars.outwidth,CV_8UC1);
   mask.setTo(0);
  // map.create(vars.outheight,vars.outwidth);
  bool uv_valide = false;

   //外参转换 
   std::ofstream outfile("out.txt", std::ios::app); 
   std::fstream f("//home//dorothy//Tools//camodocal//build//bin//timestamps.txt");
   std::vector<std::string> filename;
   std::string line;
   while(std::getline(f,line)){
         filename.push_back(line);
         std::string newid = filename[filename.size() - 1].substr(1,9);
         outfile << newid << std::endl;
   }
   
  
   std::vector<Eigen::Matrix4d> T(4);
   std::vector<Eigen::Vector3d> eulerAngle;
   std::vector<Eigen::Matrix3d> rotation_matrix; 
   std::vector<cv::Mat> cv_T{parameters.cv_T1, parameters.cv_T2, parameters.cv_T3, parameters.cv_T4};
 
   for(unsigned int i = 0; i < 4; i++)
   {
     cv::cv2eigen(cv_T[i], T[i]);  
     Eigen::Matrix3d R = T[i].block(0,0,3,3);
     rotation_matrix.push_back(R);
     eulerAngle.push_back(rotation_matrix[i].eulerAngles(2,1,0));//旋转矩阵转欧拉角(Z-Y-X，即RPY)
     std::cout << "r-p-y:" << eulerAngle[i] << '\n';
     outfile << "z: " <<  eulerAngle[i](0) * (180/CV_PI)  << " y: " << eulerAngle[i](1) * (180/CV_PI) << " x: " << eulerAngle[i](2) * (180/CV_PI) << std::endl;
   }

   outfile.close();

   for (i=1;i<argc;i++) {
      if (strcmp(argv[i],"-w") == 0) {
         i++;
         vars.outwidth = 2 * (atoi(argv[i]) / 2); // Ensure even
         vars.outheight = vars.outwidth / 2; // Default for equirectangular images
      } else if (strcmp(argv[i],"-a") == 0) {
         i++;
         vars.antialias = atoi(argv[i]);
         if (vars.antialias < 1)
            vars.antialias = 1;
         vars.antialias2 = vars.antialias * vars.antialias;
      } else if (strcmp(argv[i],"-r") == 0) {
         i++;
         vars.fishradius = atoi(argv[i]);
      } else if (strcmp(argv[i],"-s") == 0) {
         i++;
         vars.fishfov = DTOR*atof(argv[i])/2;
      } else if (strcmp(argv[i],"-c") == 0) {
         i++;
         vars.fishcenterx = atoi(argv[i]);
         //vars.fishcenterx = parameters.u0();
         //std::cout << vars.fishcenterx << '\n';
         i++;
         vars.fishcentery = atoi(argv[i]);
         //vars.fishcentery = parameters.v0();
      } else if (strcmp(argv[i],"-d") == 0) {
         debug = TRUE;
#ifdef ADDTIFF
      } else if (strcmp(argv[i],"-t16") == 0) {
         vars.stmap16 = TRUE;
      } else if (strcmp(argv[i],"-t32") == 0) {
         vars.stmap32 = TRUE;
#endif
      } else if (strcmp(argv[i],"-x") == 0) {
         i++;
         vars.transform = (TRANSFORM*)realloc(vars.transform,(vars.ntransform+1)*sizeof(TRANSFORM));
         vars.transform[vars.ntransform].axis = XTILT;
         vars.transform[vars.ntransform].value = DTOR*atof(argv[i]);
         vars.ntransform++;
      } else if (strcmp(argv[i],"-y") == 0) {
         i++;
         vars.transform = (TRANSFORM*)realloc(vars.transform,(vars.ntransform+1)*sizeof(TRANSFORM));
         vars.transform[vars.ntransform].axis = YROLL;
         vars.transform[vars.ntransform].value = DTOR*atof(argv[i]);
         vars.ntransform++;
      } else if (strcmp(argv[i],"-z") == 0) {
         i++;
         vars.transform = (TRANSFORM*)realloc(vars.transform,(vars.ntransform+1)*sizeof(TRANSFORM));
         vars.transform[vars.ntransform].axis = ZPAN;
         vars.transform[vars.ntransform].value = DTOR*atof(argv[i]);
         vars.ntransform++;
      } else if (strcmp(argv[i],"-blo") == 0) {
         i++;
         vars.longblend1 = DTOR*atof(argv[i]);
         i++;
         vars.longblend2 = DTOR*atof(argv[i]);
      } else if (strcmp(argv[i],"-bla") == 0) {
         i++;
         vars.latblend1 = DTOR*atof(argv[i]);
         i++;
         vars.latblend2 = DTOR*atof(argv[i]);
      } else if (strcmp(argv[i],"-bg") == 0) {
         i++;
         vars.background.r = atoi(argv[i]);
         i++;
         vars.background.g = atoi(argv[i]);
         i++;
         vars.background.b = atoi(argv[i]);
      } else if (strcmp(argv[i],"-lam") == 0) {
         i++;
         vars.latmax = DTOR*atof(argv[i]);
      } else if (strcmp(argv[i],"-lom") == 0) {
         i++;
         vars.longmax = DTOR*atof(argv[i]);
      } else if (strcmp(argv[i],"-o") == 0) {
         i++;
         vars.makeobj = atoi(argv[i]);
      } else if (strcmp(argv[i],"-f") == 0) {
         vars.makeremap = TRUE;
      } else if (strcmp(argv[i],"-h") == 0) {
         vars.hammer = TRUE;
      } else if (strcmp(argv[i],"-p") == 0) {
         i++;
         vars.a1 = atof(argv[i]);
         i++;
         vars.a2 = atof(argv[i]);
         i++;
         vars.a3 = atof(argv[i]);
         i++;
         vars.a4 = atof(argv[i]);
         vars.rcorrection = TRUE;
      }
   }

   //just for test.
   jpg2tiff_alfa();

   // Precompute sine and cosine transformation angles
   for (j=0;j<vars.ntransform;j++) {
      vars.transform[j].cvalue = cos(vars.transform[j].value);
      vars.transform[j].svalue = sin(vars.transform[j].value);
   }

   // Image type
#ifdef ADDJPEG
   if (IsJPEG(argv[argc-1]))
      vars.inputformat = JPG;
#endif

   // Malloc images 
   if ((fptr = fopen(argv[argc-1],"rb")) == NULL) {
      fprintf(stderr,"Failed to open file\n");
      exit(-1);
   }
#ifdef ADDJPEG
   if (vars.inputformat == JPG)
      JPEG_Info(fptr,&vars.fishwidth,&vars.fishheight,&depth);
   else
#endif
      TGA_Info(fptr,&vars.fishwidth,&vars.fishheight,&depth);
   fisheye = Create_Bitmap(vars.fishwidth,vars.fishheight);
#ifdef ADDJPEG
   if (vars.inputformat == JPG) {
      if (JPEG_Read(fptr,fisheye,&w,&h) != 0) {
         fprintf(stderr,"Error: Failed to correctly read JPEG image\n");
         exit(-1);
      }
   }
#endif
	if (vars.inputformat == TGA) {
      if (TGA_Read(fptr,fisheye,&w,&h) != 0) {
         fprintf(stderr,"Error: Failed to correctly read TGA image\n");
         exit(-1);
      }
   }
   fclose(fptr);

   // Sort out output image
   spherical = Create_Bitmap(vars.outwidth,vars.outheight);
   if (vars.longblend1 >= 0 && vars.longblend2 >= 0)
      Erase_Bitmap(spherical,vars.outwidth,vars.outheight,black);
   else if (vars.latblend1 >= 0 && vars.latblend2 >= 0) 
      Erase_Bitmap(spherical,vars.outwidth,vars.outheight,black);
   else
      Erase_Bitmap(spherical,vars.outwidth,vars.outheight,vars.background);

   // Prepare for STMaps
#ifdef ADDTIFF
   if (vars.stmap16) {
      stmap16 = malloc(vars.outwidth*vars.outheight*sizeof(COLOUR16));
      for (i=0;i<vars.outwidth*vars.outheight;i++)
         stmap16[i] = black16;
   }
   if (vars.stmap32) {
      stmap32 = malloc(vars.outwidth*vars.outheight*sizeof(COLOUR32));
      for (i=0;i<vars.outwidth*vars.outheight;i++)
         stmap32[i] = black32;
   }
#endif

   // Set unspecified fisheye values
   if (vars.fishcenterx < 0 || vars.fishcentery < 0) {
      vars.fishcenterx = vars.fishwidth / 2; // Assumes circle is centered in sensor rectangle
      vars.fishcentery = vars.fishheight / 2;
   }
   //vars.fishcentery = vars.fishheight - vars.fishcentery;
   if (vars.fishradius < 0) 
      vars.fishradius = vars.fishwidth / 2; // Assumes fills sensor

   // Form the spherical map 
   starttime = GetTime(); 
   for (j=0;j<vars.outheight;j++) {
      for (i=0;i<vars.outwidth;i++) {
         rsum = 0;
         gsum = 0;
         bsum = 0;

         // Antialiasing loops 
         for (ai=0;ai<vars.antialias;ai++) {
            for (aj=0;aj<vars.antialias;aj++) {

               // Normalised coordinates
               x = 2 * (i + ai/(double)vars.antialias) / (double)vars.outwidth - 1; // -1 to 1
               y = 2 * (j + aj/(double)vars.antialias) / (double)vars.outheight - 1; // -1 to 1

               if (vars.hammer) {
                  x *= SQRT2;
                  y *= SQRT2;
                  z = sqrt(1-x*x/4-y*y/4);
                  longitude = 2*atan(x*z/(2*z*z-1));
                  latitude = asin(y*z);
               } else {
                  longitude = x * PI;     // -pi <= x < pi
                  latitude = y * PI/2;     // -pi/2 <= y < pi/2
               }
               if (longitude < -vars.longmax || longitude > vars.longmax)
                  continue;
               if (latitude < -vars.latmax || latitude > vars.latmax)
                  continue;
   
               if (vars.longblend1 >= 0 && vars.longblend2 >= 0) {
                  longblend = (vars.longblend2 - fabs(longitude)) / (vars.longblend2 - vars.longblend1);
                  if (longblend < 0)
                     longblend = 0;
                  if (longblend > 1)
                     longblend = 1;
               }

               if (vars.latblend1 >= 0 && vars.latblend2 >= 0) {
                  latblend = (vars.latblend2 - latitude) / (vars.latblend2 - vars.latblend1);
                  if (latblend < 0)
                     latblend = 0;
                  if (latblend > 1)
                     latblend = 1;
               }

               // Clip
               if (vars.hammer) {
                  if (x*x + y*y > 2)
                     continue;
               } 

               // Find the corresponding pixel in the fisheye image
               // Sum over the supersampling set
               if (FindFishPixel2(latitude,longitude,&u,&v,&fu,&fv,cata_camera)) {
                  uv_valide = true;
                  index = v * vars.fishwidth + u;
                  rsum += latblend * longblend * fisheye[index].r;
                  gsum += latblend * longblend * fisheye[index].g;
                  bsum += latblend * longblend * fisheye[index].b;
#ifdef ADDTIFF
                  if (ai == 0 && aj == 0 && vars.stmap16) {
                     index = j*vars.outwidth+i;
                     stmap16[index].r = (unsigned short)(fu * 65535);
                     stmap16[index].g = (unsigned short)(fv * 65535);
                     stmap16[index].b = 0;
                  }
                  if (ai == 0 && aj == 0 && vars.stmap32) {
                     index = j*vars.outwidth+i;
                     stmap32[index].r = (unsigned int)(fu * UINT_MAX);
                     stmap32[index].g = (unsigned int)(fv * UINT_MAX);
                     stmap32[index].b = 0;
                  }
#endif
               }
            }
         }
          
         index = j * vars.outwidth + i;
         spherical[index].r = rsum / vars.antialias2;
         spherical[index].g = gsum / vars.antialias2;
         spherical[index].b = bsum / vars.antialias2;
         if (uv_valide == true) 
         {
               spherical[index].a = 255;
               mask.at<uchar>(j,i) = 255;
         }
         else
         {
               spherical[index].a = 0;
               mask.at<uchar>(j,i) = 0;
         }
         
         
        
         
         
      }
   }
   stoptime = GetTime();
   cv::imwrite("mask.png",mask);
   //save spherical as tif 
   TIFF *tifFile = TIFFOpen("spherical_new.tif", "w");
   uint32 width, height;
    width  = vars.outwidth;
    height = vars.outheight;
     if ( tifFile ) 
     { 
         TIFFSetField( tifFile, TIFFTAG_IMAGEWIDTH, width ); 
         TIFFSetField( tifFile, TIFFTAG_IMAGELENGTH, height ); 
         TIFFSetField( tifFile, TIFFTAG_COMPRESSION, COMPRESSION_PACKBITS); 
         TIFFSetField( tifFile, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG); 
         TIFFSetField( tifFile, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB); 
         TIFFSetField( tifFile, TIFFTAG_BITSPERSAMPLE, 8); 
         TIFFSetField( tifFile, TIFFTAG_SAMPLESPERPIXEL, 4); 
         
        int cols = vars.outwidth;
        uchar* pdst = new uchar[cols*4];
         for(uint32 i = 0; i < vars.outheight; i++)
         {              
               int curidx_bit = i * cols * 4;
               for ( int idx = 0; idx < cols; idx ++ ) 
               { 
                  index = i * vars.outwidth + idx;
                  int curidx_dst  = idx * 4; 
                  pdst[curidx_dst]   = spherical[index].r;//r
                  pdst[curidx_dst+1] = spherical[index].g;//g
                  pdst[curidx_dst+2] = spherical[index].b;//b
                  pdst[curidx_dst+3] = spherical[index].a;//a
               } 

               TIFFWriteScanline( tifFile, pdst, i, 0 );
         }
        TIFFClose( tifFile ); 
     }
     else
     {
           std::cout << "Open file error!" << std::endl;
           exit(1);
     }

   if (debug) {
      fprintf(stderr,"Time for sampling: %g sec ",stoptime-starttime);
      fprintf(stderr,"(%g fps)\n",1.0/(stoptime-starttime));
   }

   // Write out the spherical map 
   if (vars.inputformat == JPG)
      sprintf(fname,"%s_sph.jpg",basename);
   else
      sprintf(fname,"%s_sph.tga",basename);
   if ((fptr = fopen(fname,"wb")) == NULL) {
      fprintf(stderr,"Failed to open output file\n");
      exit(-1);
   }
#ifdef ADDJPEG
   if (vars.inputformat == JPG)
      JPEG_Write(fptr,spherical,vars.outwidth,vars.outheight,100);
   else
#endif
      Write_Bitmap(fptr,spherical,vars.outwidth,vars.outheight,12);
   fclose(fptr);

   // Optionally create textured mesh as OBJ file
   if (vars.makeobj >= SPHERE && vars.makeobj <= APERTURE)
      MakeObj(basename,vars.makeobj);

   // Optionally create ffmpeg remap filter PGM files
   if (vars.makeremap) 
      MakeRemap(basename);

   // Save stmap as 16 bit tiff
#ifdef ADDTIFF
   if (vars.stmap16) {
      sprintf(fname,"%s_stmap16.tif",basename);
      TIFF_Write16(fname,stmap16,vars.outwidth,vars.outheight);
   }
   if (vars.stmap32) {
      sprintf(fname,"%s_stmap32.tif",basename);
      TIFF_Write32(fname,stmap32,vars.outwidth,vars.outheight);
   }
#endif

   exit(0);
}

void jpg2tiff_alfa()
{
    TIFF *tifFile = TIFFOpen("spherical.tif", "w");
    cv::Mat imageAlpha = cv::imread( "/home/dorothy/Tools/camodocal/1/masks3.jpg",CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat imagejpg = cv::imread("/home/dorothy/Tools/camodocal/1/tempImage3.jpg",CV_LOAD_IMAGE_UNCHANGED);

    std::vector<cv::Mat> channels;
    cv::split(imagejpg,channels);

    uint32 width, height;
    width  = vars.outwidth;
    height = vars.outheight;
     if ( tifFile ) 
     { 
         TIFFSetField( tifFile, TIFFTAG_IMAGEWIDTH, width ); 
         TIFFSetField( tifFile, TIFFTAG_IMAGELENGTH, height ); 
         TIFFSetField( tifFile, TIFFTAG_COMPRESSION, COMPRESSION_PACKBITS); 
         TIFFSetField( tifFile, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG); 
         TIFFSetField( tifFile, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB); 
         TIFFSetField( tifFile, TIFFTAG_BITSPERSAMPLE, 8); 
         TIFFSetField( tifFile, TIFFTAG_SAMPLESPERPIXEL, 4); 
         
        int cols = vars.outwidth;
        uchar* pdst = new uchar[cols*4];
         for(uint32 i = 0; i < vars.outheight; i++)
         {              
               int curidx_bit = i * cols * 4;
               for ( int idx = 0; idx < cols; idx ++ ) 
               { 
                  int curidx_dst  = idx * 4; 
                  pdst[curidx_dst]   = (uchar)channels[2].at<uchar>(i,idx);//r
                  pdst[curidx_dst+1] = (uchar)channels[1].at<uchar>(i,idx);//g
                  pdst[curidx_dst+2] = (uchar)channels[0].at<uchar>(i,idx);//b
                  pdst[curidx_dst+3] = (uchar)imageAlpha.at<uchar>(i,idx);//a
               } 

               TIFFWriteScanline( tifFile, pdst, i, 0 );
         }
        TIFFClose( tifFile ); 
     }
     else
     {
           std::cout << "Open file error!" << std::endl;
           exit(1);
     }
}
void GiveUsage(char *s)
{
   fprintf(stderr,"Usage: %s [options] imagefile\n",s);
   fprintf(stderr,"Options\n");
   fprintf(stderr,"   -w n        sets the output image size, default: 4 fisheye image width\n");
   fprintf(stderr,"   -a n        sets antialiasing level, default: %d\n",vars.antialias);
   fprintf(stderr,"   -s n        fisheye fov (degrees), default: %g\n",2*RTOD*vars.fishfov);
   fprintf(stderr,"   -c x y      fisheye center, default: center of image\n");
   fprintf(stderr,"   -r n        fisheye radius, default: half the fisheye image width\n");
   fprintf(stderr,"   -x n        tilt camera, default: 0\n");
   fprintf(stderr,"   -y n        roll camera, default: 0\n");
   fprintf(stderr,"   -z n        pan camera, default: 0\n");
   fprintf(stderr,"   -blo n n    longitude range for blending, default: no blending\n");
   fprintf(stderr,"   -bla n n    latitude range for blending, default: no blending\n");
   fprintf(stderr,"   -lom n      +- maximum longitude range clipping, default: no clipping\n");
   fprintf(stderr,"   -lam n      +- maximum latitude range clipping, default: no clipping\n");
   fprintf(stderr,"   -bg r g b   set background colour, default: grey or black for blending\n");
   fprintf(stderr,"   -p n n n n  4th order lens correction, default: off\n");
   fprintf(stderr,"   -o n        create a textured mesh as OBJ model, three types: 0,1,2\n");
   fprintf(stderr,"   -h          create Hammer projection instead, default: off\n");
#ifdef ADDTIFF
   fprintf(stderr,"   -t16        save 16 bit tiff file for STMap, default: off\n");
   fprintf(stderr,"   -t32        save 32 bit tiff file for STMap, default: off\n");
#endif
   fprintf(stderr,"   -f          create PGM files for ffmpeg remap filter, default: off\n");
   fprintf(stderr,"   -d          debug mode\n");
   exit(-1);
}

/*
   Given a longitude and latitude calculate the (u,v) pixel coordinates in the fisheye
   Return FALSE if the pixel is outside the fisheye image
*/
int FindFishPixel2(double latitude,double longitude,int *u,int *v,double *fu,double *fv,camodocal::CataCamera &cata_camera)
{
   int k;
   XYZ p,q;
   double theta,phi,r;

   // p is the ray from the camera position into the scene
   p.x = cos(latitude) * sin(longitude);
   p.y = cos(latitude) * cos(longitude);
   p.z = sin(latitude);
  
   // Apply transformation
   for (k=0;k<vars.ntransform;k++) {
      switch(vars.transform[k].axis) {
      case XTILT:
         q.x =  p.x;
         q.y =  p.y * vars.transform[k].cvalue + p.z * vars.transform[k].svalue;
         q.z = -p.y * vars.transform[k].svalue + p.z * vars.transform[k].cvalue;
         break;
      case YROLL:
         q.x =  p.x * vars.transform[k].cvalue + p.z * vars.transform[k].svalue;
         q.y =  p.y;
         q.z = -p.x * vars.transform[k].svalue + p.z * vars.transform[k].cvalue;
         break;
      case ZPAN:
         q.x =  p.x * vars.transform[k].cvalue + p.y * vars.transform[k].svalue;
         q.y = -p.x * vars.transform[k].svalue + p.y * vars.transform[k].cvalue;
         q.z =  p.z;
         break;
      }
      p = q;
   }

   // Calculate fisheye polar coordinates
   theta = atan2(p.z,p.x); // -pi ... pi
   phi = atan2(sqrt(p.x*p.x+p.z*p.z),p.y); // 0 ... fov
   if (!vars.rcorrection) {
      r = phi / vars.fishfov; // 0 .. 1
   } else {
      r = phi * (vars.a1 + phi * (vars.a2 + phi * (vars.a3 + phi * vars.a4)));
      if (phi > vars.fishfov)
         r *= 10;
   }
   if (r > 1)
      return(FALSE);

   // Determine the u,v coordinate
   Eigen::Vector2d p_est;
   Eigen::Vector3d P(p.x,p.z,-p.y);
   //cata_camera.spaceToPlane(P, p_est);
   
   *fu = vars.fishcenterx + r * vars.fishradius * cos(theta);
   *fv = vars.fishcentery + r * vars.fishradius * sin(theta);

   //std::cout << "-----" <<r * vars.fishradius * cos(theta) << '\n';
   //*fu = p_est(0);
   double xx_c = r * vars.fishradius * cos(theta);
   double xx = r * cos(theta);

   double yy_c = r * vars.fishradius * sin(theta);
   double yy = r * sin(theta);
   
   Eigen::Vector2d p_src(*fu,*fv);
   Eigen::Vector2d p_u;
   Eigen::Vector2d p_u_final;
   Eigen::Vector3d p_3d;

   cata_camera.liftProjective(p_src,p_3d);//将求出的单位圆上的点默认为畸变点，那么需要先投影到空间单位球上（对应的是不带畸变的空间点），然后再进行畸变矫正
   
   cata_camera.spaceToPlane(p_3d,p_u_final);//p_d为单位圆上的畸变点，从而求出映射矩阵（逆向映射）

   //*u = *fu;
   *u = p_u_final(0);
   *fu /= vars.fishwidth;
   if (*u < 0 || *u >= vars.fishwidth || *fu >= 1 || fv <= 0)
      return(FALSE);

   //*v = *fv;
   *v = p_u_final(1);
   *fv /= vars.fishheight;
   if (*v < 0 || *v >= vars.fishheight || *fv >= 1 || *fv < 0)
       return(FALSE);

      
   return(TRUE);
}

int FindFishPixel(double latitude,double longitude,int *u,int *v,double *fu,double *fv)
{
   int k;
   XYZ p,q;
   double theta,phi,r;

   // p is the ray from the camera position into the scene
   p.x = cos(latitude) * sin(longitude);
   p.y = cos(latitude) * cos(longitude);
   p.z = sin(latitude);
  
   // Apply transformation
   for (k=0;k<vars.ntransform;k++) {
      switch(vars.transform[k].axis) {
      case XTILT:
         q.x =  p.x;
         q.y =  p.y * vars.transform[k].cvalue + p.z * vars.transform[k].svalue;
         q.z = -p.y * vars.transform[k].svalue + p.z * vars.transform[k].cvalue;
         break;
      case YROLL:
         q.x =  p.x * vars.transform[k].cvalue + p.z * vars.transform[k].svalue;
         q.y =  p.y;
         q.z = -p.x * vars.transform[k].svalue + p.z * vars.transform[k].cvalue;
         break;
      case ZPAN:
         q.x =  p.x * vars.transform[k].cvalue + p.y * vars.transform[k].svalue;
         q.y = -p.x * vars.transform[k].svalue + p.y * vars.transform[k].cvalue;
         q.z =  p.z;
         break;
      }
      p = q;
   }

   // Calculate fisheye polar coordinates
   theta = atan2(p.z,p.x); // -pi ... pi
   phi = atan2(sqrt(p.x*p.x+p.z*p.z),p.y); // 0 ... fov
   if (!vars.rcorrection) {
      r = phi / vars.fishfov; // 0 .. 1
   } else {
      r = phi * (vars.a1 + phi * (vars.a2 + phi * (vars.a3 + phi * vars.a4)));
      if (phi > vars.fishfov)
         r *= 10;
   }
   if (r > 1)
      return(FALSE);

   // Determine the u,v coordinate
   *fu = vars.fishcenterx + r * vars.fishradius * cos(theta);
   *fu = 
   *u = *fu;
   *fu /= vars.fishwidth;
   if (*u < 0 || *u >= vars.fishwidth || *fu >= 1 || fv <= 0)
      return(FALSE);
   *fv = vars.fishcentery + r * vars.fishradius * sin(theta);
   *v = *fv;
   *fv /= vars.fishheight;
   if (*v < 0 || *v >= vars.fishheight || *fv >= 1 || *fv < 0)
       return(FALSE);

   return(TRUE);
}

double GetTime(void)
{
   double sec = 0;
   struct timeval tp;

   gettimeofday(&tp,NULL);
   sec = tp.tv_sec + tp.tv_usec / 1000000.0;

   return(sec);
}

/*
   Create ffmpeg remap filter PGM file
   Two files, one for x coordinate and one for y coordinate
   https://trac.ffmpeg.org/wiki/RemapFilter
*/
void MakeRemap(char *bn)
{
   int i,j,ix,iy,u,v;
   double x,y,longitude,latitude,fu,fv;
   char fname[256];
   FILE *fptrx = NULL,*fptry = NULL;

   sprintf(fname,"%s_x.pgm",bn);
   fptrx = fopen(fname,"w");
   fprintf(fptrx,"P2\n%d %d\n65535\n",vars.outwidth,vars.outheight);

   sprintf(fname,"%s_y.pgm",bn);
   fptry = fopen(fname,"w");
   fprintf(fptry,"P2\n%d %d\n65535\n",vars.outwidth,vars.outheight);

   for (j=vars.outheight-1;j>=0;j--) {
      for (i=0;i<vars.outwidth;i++) {
         ix = -1;
         iy = -1;

         // Normalised coordinates
         x = 2 * i / (double)vars.outwidth - 1; // -1 to 1
         y = 2 * j / (double)vars.outheight - 1; // -1 to 1

         // Calculate longitude and latitude
         longitude = x * PI;     // -pi <= x < pi
           latitude = y * PI/2;     // -pi/2 <= y < pi/2

         // Find the corresponding pixel in the fisheye image
         if (FindFishPixel(latitude,longitude,&u,&v,&fu,&fv)) {
            ix = u;
            iy = vars.fishheight-1-v;
         }
         fprintf(fptrx,"%d ",ix);
         fprintf(fptry,"%d ",iy);
      }
      fprintf(fptrx,"\n");
      fprintf(fptry,"\n");
   }
   fclose(fptrx);
   fclose(fptry);
}

/*
   Quick and nasty implementation of object file creation
   A mesh textured with the original fisheye image
   The rotations should be such as to align true "up" on the spherical image
*/
void MakeObj(char *bn,int thetype)
{
   int i,j,i1,i2,i3,i4;
   double fu,fv,latitude,longitude;
   XYZ p;
   int u,v;
   FILE *fptr;
   char fname[128];

   // mtl file
   sprintf(fname,"%s.mtl",bn);
   fptr = fopen(fname,"w");
   fprintf(fptr,"newmtl spheremtl\n");
   fprintf(fptr,"Ka 0 0 0\n");
   fprintf(fptr,"Kd 1 1 1\n");
   fprintf(fptr,"Ks 0 0 0\n");
   fprintf(fptr,"Ns 100\n");
   fprintf(fptr,"map_Kd %s.jpg\n",bn); // Assumes the original fisheye is a jpg
   fclose(fptr);

   // obj file
   sprintf(fname,"%s.obj",bn);
   fptr = fopen(fname,"w");
   fprintf(fptr,"mtllib %s.mtl\n",bn);
   fprintf(fptr,"g default\n");

   // Vertices
   for (j=0;j<=NOBJ/2;j++) {
      if (thetype == HEMISPHERE)
         latitude = j * PI / NOBJ;     // 0 ... pi/2
      else if (thetype == APERTURE)
         latitude = -(vars.fishfov-PID2) + 2 * j * (PID2+vars.fishfov-PID2) / NOBJ;
      else
         latitude = -PID2 + 2 * j * PI / NOBJ;     // -pi/2 ... pi/2
      if (thetype == SPHERE && j == 0) // Hack
         latitude += 0.001;
      if (j == NOBJ/2) // Hack
         latitude -= 0.001;
      for (i=0;i<=NOBJ;i++) {
         longitude = -PI + i * TWOPI / NOBJ;     // -pi ... pi
         p.x = cos(latitude)*sin(longitude);
         p.y = cos(latitude)*cos(longitude);
         p.z = sin(latitude);
         fprintf(fptr,"v %lf %lf %lf\n",p.x,p.y,p.z);
      }
   }

   // Texture coordinates
   for (j=0;j<=NOBJ/2;j++) {
      if (thetype == HEMISPHERE)
         latitude = j * PI / NOBJ;     // 0 ... pi/2
      else if (thetype == APERTURE)
         latitude = -(vars.fishfov-PID2) + 2 * j * (PID2+vars.fishfov-PID2) / NOBJ;
      else
         latitude = -PID2 + 2 * j * PI / NOBJ;     // -pi/2 ... pi/2
      for (i=0;i<=NOBJ;i++) {
         longitude = -PI + i * TWOPI / NOBJ;     // -pi ... pi
         FindFishPixel(latitude,longitude,&u,&v,&fu,&fv);
         if (u < 0)
            u = 0;
         if (v < 0)
            v = 0;
         if (u > vars.fishwidth)
            u = vars.fishwidth;
         if (v > vars.fishheight)
            v = vars.fishheight;
         fprintf(fptr,"vt %lf %lf\n",fu,fv);
      }
   }

   // Normals, same as vertices
   for (j=0;j<=NOBJ/2;j++) {
      if (thetype == HEMISPHERE)
         latitude = j * PI / NOBJ;     // 0 ... pi/2
      else if (thetype == APERTURE)
         latitude = -(vars.fishfov-PID2) + 2 * j * (PID2+vars.fishfov-PID2) / NOBJ;
      else
         latitude = -PID2 + 2 * j * PI / NOBJ;     // -pi/2 ... pi/2
      for (i=0;i<=NOBJ;i++) {
         longitude = -PI + i * TWOPI / NOBJ;     // -pi ... pi
         p.x = cos(latitude)*sin(longitude);
         p.y = cos(latitude)*cos(longitude);
         p.z = sin(latitude);
           fprintf(fptr,"vn %lf %lf %lf\n",p.x,p.y,p.z);
      }
   }

   // Faces
   fprintf(fptr,"usemtl spheremtl\n");
   for (j=0;j<NOBJ/2;j++) {
      for (i=0;i<NOBJ;i++) {
         i1 = 1 + j * (NOBJ+1) + i;
         i4 = 1 + j * (NOBJ+1) + i + 1;
         i3 = 1 + (j+1) * (NOBJ+1) + i + 1;
         i2 = 1 + (j+1) * (NOBJ+1) + i;
           fprintf(fptr,"f %d/%d/%d %d/%d/%d %d/%d/%d\n",i1,i1,i1, i2,i2,i2, i3,i3,i3);
           fprintf(fptr,"f %d/%d/%d %d/%d/%d %d/%d/%d\n",i1,i1,i1, i3,i3,i3, i4,i4,i4);
      }
   }

   fclose(fptr);
}

/*
   Initial values of all variables
*/
void InitVars(void)
{
   vars.fishwidth = 0;
   vars.fishheight = 0;
   vars.outwidth = 2048;
   vars.outheight = 1024;
   vars.antialias = 2;
   vars.antialias2 = 4;
   vars.fishcenterx = -1;
   vars.fishcentery = -1;
   vars.fishradius = -1;
   vars.fishfov = DTOR*180/2.0;
   vars.transform = NULL;
   vars.ntransform = 0;
   vars.longblend1 = -1;
   vars.longblend2 = -1;
   vars.latblend1 = -1;
   vars.latblend2 = -1;
   vars.latmax = 1e32;
   vars.longmax = 1e32;
   vars.rcorrection = FALSE;
   vars.stmap16 = FALSE;
   vars.stmap32 = FALSE;
   vars.a1 = 1;
   vars.a2 = 0;
   vars.a3 = 0;
   vars.a4 = 0;
   vars.background.r = 128;
   vars.background.g = 128;
   vars.background.b = 128;
   vars.background.a = 255;
   vars.inputformat = TGA;
   vars.makeobj = -1;
   vars.makeremap = FALSE;
}


