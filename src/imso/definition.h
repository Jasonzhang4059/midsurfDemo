#if !defined(__DEFINITION_H_INCLUDED)
#define __DEFINITION_H_INCLUDED 


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <iterator>
#include <vector>
#include <list>
#include <map>
#include <algorithm>

# define _WINDOWS 1
#if defined(_WINDOWS)
#include <windows.h>
#endif


#define SINGLE
#ifdef SINGLE

typedef float dFloat;
#define PI 3.14159265358979323846
#define CAMERADIST		500 
#define  ERR            1.0e-4
#define  ERRF           1.0e-5
#define CVLEN			0.05f //0.02
#define DELTA1           0.1       
#define DELTA2           0.01 
#define  PerturbTimes    1
enum{
	POINT_MODE = 0,
	TRANSPARENT_MODE = 1,
	SHADE_MODE = 2,
	WIREFRAME_MODE = 3,
	FIELDVALUESHOW_MODE
};
enum{
	HEIGHT_FIELD = 0,
	CURVATURE_FIELD = 1,
	DISTANCE_FIELD = 2,
	GEODESTIC_FIELD = 3,
	HARMONIC_FIELD = 4,
	SPECTRIM_FIELD = 5,
	GIVEN_FIELD = 6
};

namespace base {
	//Constants
	const int    MAX_TEXTURE_STAGE = 2;
	typedef std::vector<int>   ivec;
	typedef std::vector<unsigned int>   uvec;
	typedef std::vector<float> fvec;


	//Templates

	template<class _T>
	_T MIN(_T a, _T b) {return ((a<b)?a:b);}

	template<class _T>
	_T MAX(_T a, _T b) {return ((a>b)?a:b);}

	template<class _T>
	void SafeDelete(_T *x) {if (x) delete [] x; x = NULL;}

	template<class _T>
	void ClearPointerVector(_T& x) {for(int __i=0; __i<x.size();__i++)delete x[__i]; x.clear();}

	template<class A, class _T>
	void CopyPointerVector(_T& a, const _T& b) {
		ClearPointerVector(a);
		a.resize(b.size());
		for(int i=0; i<b.size();i++) {
			a[i] = new A;
			*a[i] = *b[i];
		}
	}

	template<class _T >
	void DeletePointerVectorElement(_T& a, int i) {
		_T::iterator iter=a.begin();
		iter+=i;
		delete *iter;
		a.erase(iter);
	}

	template<class _T >
	void DeleteVectorElement(_T& a, int i) {
		_T::iterator iter=a.begin();
		iter+=i;
		a.erase(iter);
	}

}




#endif











#endif