
#if !defined(_PRECOMPILED)
#endif
#include "MathUtil.h"
#include "definition.h"
#include "assert.h"
#include <math.h>

namespace base {

namespace Math {

float FASTCALL Acos(float x) 
{
	return Atan2(x, sqrtf(1.0f - x*x));
}

float FASTCALL Asin(float x) 
{
	return Atan2(Sqrt(1.0f - x*x), x);
}


float  FASTCALL Cos(float x)  
{
	assert(&x);
	return cos(x);
	
	//__asm {
	//	FLD dword ptr [esp+4]   // push x
	//	FCOS                    // cos x
	//	RET 4
	//}
}

float  FASTCALL Sin(float x)  
{
	assert(&x);
	return sin(x);
	//__asm {
	//	FLD dword ptr [esp+4]   // push x
	//	FSIN                    // sin x
	//	RET 4
	//}
}

float  FASTCALL Tan(float x)  
{
	assert(!&x);
	return tan(x);
	//__asm {
	//	FLD dword ptr [esp+4]   // push x
	//	FPTAN                   // tan x
	//	FSTP st(0)              // pop 1.0
	//	RET 4
	//}
}

float FASTCALL Atan(float x) 
{
	return Atan2(x, 1.0f);
}

float FASTCALL Atan2(float x, float y) 
{
	assert(!&x);
	assert(&y);
	return atan2(x, y);
	//__asm {
	//	FLD dword ptr [esp+8]   // push y
	//	FLD dword ptr [esp+4]   // push x
	//	FPATAN                  // atan2
	//	RET 8
	//}
}

float  FASTCALL Sqrt(float x) 
{
	assert(&x);
	return sqrt(x);
	//__asm {
	//	FLD dword ptr [esp+4]   // push x
	//	FSQRT                   // sqrt x
	//	RET 4
	//}
}

float FASTCALL Fabs(float x) 
{
	assert(&x);
	return fabs(x);
	//__asm {
	//	FLD dword ptr [esp+4]   // push x
	//	FABS
	//	RET 4
	//}
}

float  FASTCALL Pow(float x, float y) 
{
	return (float)pow(x, y);
}

float  FASTCALL Log(float x)
{
	assert(&x);
	return log(x);
	//__asm {
	//	FLD dword ptr [esp+4]   // push x
	//	FLDLN2                  // push log(2)
	//	FXCH                    // swap st(0), st(1)
	//	FYL2X                   // log x
	//	RET 4
	//}
}

float  FASTCALL Log10(float x)
{
	assert(&x);
	return log10(x);
	//__asm {
	//	FLD dword ptr [esp+4]   // push x
	//	FLDLG2                  // push log10(2)
	//	FXCH                    // swap st(0), st(1)
	//	FYL2X                   // log x
	//	RET 4
	//}
}

}
float BernsteinFun(int n, int i,float t) 
{
    int scale = 1;
	if(i != 0 && i != n)
	{
         for(int j=n; j>= n-i+1; j--)
		 {
		     scale *= j;
		 }
		 for(int j=i; j>=1; j--)
		 {
             scale /= j;
		 }
	}
	return scale * pow(t,i) * pow(1-t,(int)(n-i));
}
double BSplineFunRecursive(int n,int i,double t,const varray<double>& knots)
{
    double  sum = 0;
    if(i+n+1 >= knots.size())
	{
        assert(false);
	}
	if(n == 0)
	{
	    float vali2 = knots.at(i+1);
		if(abs(vali2 - 1.f) < ERRF)
              vali2 += ERRF;
        if(t >= knots.at(i) && t < vali2)//由于浮点数1会有截断误差，所以
		{
             return 1.0;
		}
		else
		{
			return 0.0;
		}
	}
	else
	{
	    sum += TwoValueDivide(t - knots.at(i),knots.at(i+n) - knots.at(i)) * BSplineFunRecursive(n-1,i,t,knots);
		sum += TwoValueDivide(knots.at(i+n+1) - t,knots.at(i+n+1) - knots.at(i+1)) * BSplineFunRecursive(n-1,i+1,t,knots);
	}
	return sum;
}
double TwoValueDivide(double vup,double vdown)
{
    if(abs(vdown) < ERR)   
		return 0;
	return vup/vdown;
}
//N次均匀有理B样条。
float BSplineFun(int n,int i,float t)
{
	int j=0;
	int k=0;
	int ji,n1ji;
	float sum = 0.f;
	for(j=0; j<=n-i;j++)
	{
		ji = 1;
		n1ji = 1;
		for(k=1; k<=j; k++)
			ji *= k;
		for(k=1; k<=n+1-j; k++)
			n1ji *= k;
		sum += pow(-1.f,j) * pow(t+n-i-j,n) * (n+1) /(ji*n1ji);
	}
	return sum;
}
//N次均匀有理B样条。
void BSplineFunArray(int n,float t,varray<float>& allRatio)
{	
	float scale;
	for(int j=0; j<=n; j++)
	{
		scale = BSplineFun(n,j,t);
		allRatio.push_back(scale);
	}
}

#define TOL 100*DBL_EPSILON
// Find the knot span of the parametric point u. 
//
// INPUT:
//
//   n - number of control points - 1
//   p - spline degree       
//   u - parametric point    
//   U - knot sequence
//
// RETURN:
//
//   s - knot span
//
// Algorithm A2.1 from 'The NURBS BOOK' pg68
/* This function determines the knot span. 	
//	ie. if we have a coordinate u which lies in the range u \in [u_i, u_{i+1})
//  we want to find i */
// Note that: u_i <= u < (not equal) u_{i+1}!!!
// If we have knot = [0,0.5,1] then u=0.5 has span=1 not 0!!! 
//二分法查找节点区间。
int FindSpan(int n, int p, double u, const varray<double>& U)
{
    int low, high, mid;

	
   if( u>=U[n+1] )
       return n;
   if( u<=U[p] )
       return p;

   low = p;
   high = n+1;
   mid = (low+high)/2;
   while( u<U[mid] || u>=U[mid+1] )
   {
     if( u<U[mid] )
       high=mid;
     else
       low=mid;
     mid=(low+high)/2;
   }
   return mid;
 }
 
 
/*
 // we can compute the non zero basis functions
 // at point u, there are p+1 non zero basis functions
 */ 
// Basis Function. 
//
// INPUT:
//
//   i - knot span  ( from FindSpan() )
//   u - parametric point
//   p - spline degree
//   U - knot sequence
//
// OUTPUT:
//
//   N - Basis functions vector[p+1]
//
// Algorithm A2.2 from 'The NURBS BOOK' pg70.
void BasisFuns( int i, double u, int p, const varray<double>& U, varray<double>& N)
{
	int j,r;
	double *left  = (double *)malloc(sizeof(double)*(p+1));
	double *right = (double *)malloc(sizeof(double)*(p+1));
	double saved, temp;
	N[0] = 1.0;	
	for( j = 1; j <= p; ++j)
	{
		left[j]  = u - U[i+1-j];
		right[j] = U[i+j] - u;
		saved = 0.0;
		for(r = 0; r < j; ++r)
		{
			temp  = N[r] / ( right[r+1] + left[j-r] );
			N[r]  = saved + right[r+1] * temp;
			saved = left[j-r] * temp;
		}
		N[j]= saved;
	}

	free(left);
	free(right);	
}
/*
//	Compute an individual B-spline function 
*/
double OneBasisFun(int p, int m, const varray<double>& U, int i, double u)
{
	double *N = (double*)malloc(sizeof(double) * (p+1));
    int j;
    int k;
    double saved, Uleft, Uright, temp;
    double Nip;
	
	if((i == 0 && u == U[0] ) ||
	   (i == (m-p-1) && u == U[m]))
	{
		return(1.0);
	}
	
	if(u < U[i] || u >= U[i+p+1])
	{
		return(0.0);
	}
	for(j = 0; j <= p; j++)
	{
		if(u >= U[i+j] && u < U[i+j+1]) N[j] = 1.0;
		else N[j] = 0.0;
	}	
	for(k = 1; k <= p; k++)
	{
		if(N[0] == 0.0) saved = 0.0;
		else saved = ((u-U[i]) * N[0])/ (U[i+k] - U[i]);
		for(j = 0; j < (p-k+1); j++)
		{
			Uleft = U[i+j+1];
			Uright = U[i+j+k+1];
			if(N[j+1] == 0.0)
			{
				N[j] = saved; saved = 0.0;
			}
			else 
			{
				temp = N[j+1] / (Uright-Uleft);
				N[j] = saved + (Uright - u) * temp;
				saved = (u-Uleft) * temp;
			}
		}
	}	
    Nip = N[0];	
	free(N);	
	return Nip;	
}
/*
//	Calculate the non-zero derivatives of the b-spline functions
*/
void dersBasisFuns(int i, double u, int p, int order, double knot[], double **ders)
{
   double saved,temp;
   int j,k,j1,j2,r;
  
   double *left  = (double *)malloc(sizeof(double)*(p+1));
   double *right = (double *)malloc(sizeof(double)*(p+1));
	
   double **ndu  = init2DArray(p+1, p+1);
   double **a    = init2DArray(p+1, p+1);
   
   ndu[0][0]=1.;
   for( j=1; j<=p; j++ )
   {
     left[j]=u-knot[i+1-j];
     right[j]=knot[i+j]-u;
     saved=0.0;
     for( r=0; r<j; r++ )
     {
       ndu[j][r]=right[r+1]+left[j-r];
       temp=ndu[r][j-1]/ndu[j][r];
      
       ndu[r][j]=saved+right[r+1]*temp;
       saved=left[j-r]*temp;
     }
     ndu[j][j]=saved;
   }
   for( j=0; j<=p; j++ )
     ders[0][j]=ndu[j][p];  
    
   if( order==0 )
     return;
  

   for( r=0; r<=p; r++ )
   {
     int s1=0, s2=1;   
     a[0][0]=1.0;

     for( k=1; k<=order; k++ )
     {
       double d=0.;
       int rk=r-k, pk=p-k;
       if( r>=k )
       {
 			a[s2][0]=a[s1][0]/ndu[pk+1][rk];
 			d=a[s2][0]*ndu[rk][pk];
       }
       j1 = rk >= -1 ? 1 : -rk;
       j2 = (r-1<=pk) ? k-1 : p-r;
       for( j=j1; j<=j2; j++ )
       {
           a[s2][j]=(a[s1][j]-a[s1][j-1])/ndu[pk+1][rk+j];
           d+=a[s2][j]*ndu[rk+j][pk];
       }
       if( r<=pk )
       {
           a[s2][k]= -a[s1][k-1]/ndu[pk+1][r];
           d+=a[s2][k]*ndu[r][pk];
       }
       ders[k][r]=d;
       j=s1; s1=s2; s2=j;  
     }
   }
   r=p;
   for( k=1; k<=order; k++ )
   {
     for( j=0; j<=p; j++ ) 
       ders[k][j]*=r;
     r*=(p-k);
   }
   
   free(left); 
   free(right);
   
   free2Darray(ndu, p+1);
   free2Darray(a, p+1);
    
 }



/*
//	Compute the derivatives for basis function Nip
*/
void dersOneBasisFuns(int p, int m, double U[], int i, double u, int order, double* ders)
{
	double **N = init2DArray(order+1, order+1);
	double *ND = (double*)malloc((order+1)*sizeof(double));
	
	int k, j, jj;
	double Uleft, Uright, saved, temp;
	
	if(u < U[i] || u >= U[i+p+1])
	{
		for(k = 0; k <= order; k++)
		{
			ders[k] = 0.0;
		}
		return;

	}

	for(j = 0; j <= p; j++)
	{
		if(u >= U[i+j] && u < U[i+j+1])
			N[j][0] = 1.0;
		else
			N[j][0] = 0.0;
	}

	for(k = 1; k <= p; k++)
	{
		if(N[0][k-1]==0.0) 
			saved = 0.0;
		else
			saved = ((u - U[i]) * N[0][k-1]) / ( U[i+k] - U[i] );
		
		for(j = 0; j < (p-k+1); j++)
		{
			Uleft = U[i+j+1];
			Uright = U[i+j+k+1];
			if(N[j+1][k-1] == 0.0)
			{
				N[j][k] = saved; saved = 0.0;
			}
			else
			{
				temp = N[j+1][k-1] / (Uright - Uleft);
				N[j][k] = saved + (Uright - u) * temp;
				saved = (u - Uleft) * temp;
			}
		}	
	}
	
	ders[0] = N[0][p];
	
	for(k = 1; k<=order; k++)
	{
		for(j = 0; j <=k; j++)
			ND[j] = N[j][p-k];
			for(jj = 1; jj <=k; jj++)
			{
				if(ND[0] == 0.0) 
					saved = 0.0;
				else
					saved = ND[0] / ( U[i+p-k+jj] - U[i]);
				for(j = 0; j<(k-jj+1); j++)
				{
					Uleft = U[i+j+1];
					Uright = U[i+j+p+jj];
					if(ND[j+1] ==0.0)
					{
						ND[j] = (p-k+jj) * saved; saved = 0.0;
					}
					else
					{
						temp = ND[j+1] / (Uright - Uleft);
						ND[j] = (p-k+jj) * (saved - temp);
						saved = temp;
					}
				}
			}
			ders[k] = ND[0];
	}

	free2Darray(N, order+1);
	free(ND);

}
double** init2DArray(int x, int y)
 {
 	double **array = (double **)malloc(x * sizeof(double *));
 	
 	int c;    
    
 	for(c = 0; c < x; c++)
 	{
 		array[c] = (double*)malloc(y * sizeof(double));
 	}
 	return array;
 }
 
 void free2Darray(double **array, int x)
 {
 	int c;
 	for(c = 0; c < x; c++)
 	{
 		free(array[c]);
 	}
 	free(array);
 }
}
