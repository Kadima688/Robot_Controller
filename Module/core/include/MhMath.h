#ifndef MhMath_H
#define MhMath_H

#include<math.h>
#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"eigen3/eigen3/Eigen/Dense"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include"eigen3/Eigen/Dense"
#endif


#ifndef PI
#define PI 3.14159265358979
#endif // !PI

#ifndef ANG2RAD
#define ANG2RAD 0.01745329251994
#endif // !ANG2RAD

#ifndef  RAD2ANG
#define RAD2ANG 57.29577951308232
#endif // ! RAD2ANG

namespace Mh {
	class MhMath {

	public:
		static inline double cosd(double angle) { return cos(angle * ANG2RAD); }
		static inline double sind(double angle) { return sin(angle * ANG2RAD); }
		static inline double atan2d(double value1, double value2) { return atan2(value1, value2) * RAD2ANG; }
		static inline double acosd(double angle) { return acos(angle) * RAD2ANG; }
	private:

	};
}
#endif