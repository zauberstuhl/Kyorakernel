/*****************touchscreen resolution setting*************/

#define R8C_3GA_2TG
//#define R8C_AUO_I2C

#ifdef R8C_AUO_I2C
  #ifndef R8C_3GA_2TG
  #define R8C_3GA_2TG
  #endif
#endif


#define TOUCHSCREEN_MINX 0
#define TOUCHSCREEN_MAXX 1280 /*(30*512) */
#define TOUCHSCREEN_MINY 0
#define TOUCHSCREEN_MAXY 768 /*(18*512)*/
