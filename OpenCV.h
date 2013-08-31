#include <opencv/cv.h>
#include <opencv/highgui.h>

//pragma
#ifdef _DEBUG
//Debug mode
#pragma comment(lib, "opencv_imgproc246d.lib")
#pragma comment(lib, "opencv_core246d.lib")
#pragma comment(lib, "opencv_highgui246d.lib")
#else
//Release mode
#pragma comment(lib, "opencv_imgproc246.lib")
#pragma comment(lib, "opencv_core246.lib")
#pragma comment(lib, "opencv_highgui246.lib")
#endif
