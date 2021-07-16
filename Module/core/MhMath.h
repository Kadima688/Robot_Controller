#ifndef MhMath_H
#define MhMath_H

#include<math.h>

#ifndef PI
#define PI 3.14159265358979//圆周率的宏定义
#endif // !PI

#ifndef ANG2RAD
#define ANG2RAD 0.01745329251994//角度转换为弧度的计算单位的宏定义
#endif // !ANG2RAD

#ifndef  RAD2ANG
#define RAD2ANG 57.29577951308232//弧度转换为角度的计算单位的宏定义
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