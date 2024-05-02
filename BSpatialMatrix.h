/* BSpatialMatrix 20/02/2024

 $$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialMatrix.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:


 TODO: Intrinisics
 
 On my intel machine this does not speed things up much.
 Maybe should try implementing BSpatialMatrix and BSpatialVector directly using __m256d types
 
 compile with -mavx for AVX

  
 #include <immintrin.h>
  
 const BSpatialMatrix 
 operator*( const BSpatialMatrix &m ) const
 {    
     BSpatialMatrix retVal;
     for ( int i = 0; i < 6; ++i )
     {
         const std::array<BScalar, 6>& d = m_data[i];
         for ( int j = 0; j < 6; ++j )
         {
             __m256d __res = _mm256_set1_pd(0.0);
             for ( int k = 0; k < 6; k+=3 )
             { 
                 __res = _mm256_add_pd(__res, _mm256_mul_pd(_mm256_set_pd(d[k], d[k+1], d[k+2], 0.0),
                                                            _mm256_set_pd(m[k][j], m[k+1][j], m[k+2][j], 0.0)));
             }
             retVal[i][j] = __res[0] + __res[1] + __res[2] + __res[3]; // take care reverse load
         }
     }
     return retVal;
 }
  
*/


#ifndef __BSPATIALMATRIX_H__
#define __BSPATIALMATRIX_H__

#ifndef __BMATRIX63_H__
#include "BMatrix63.h"
#endif



class BSpatialMatrix
{

public:

    BSpatialMatrix( void )=default;
    BSpatialMatrix( const std::array<std::array<BScalar, 6>, 6>& d ): m_data(d) {}
    
    explicit BSpatialMatrix( BScalar s ){ set(s); }
    explicit BSpatialMatrix( const std::vector<std::vector<BScalar>> &d ) 
    {
        assert(m_data.size() == 6 && m_data[0].size() == 6);
        set(d);
    }
    BSpatialMatrix( BScalar m00, BScalar m01, BScalar m02, BScalar m03, BScalar m04, BScalar m05,
                    BScalar m10, BScalar m11, BScalar m12, BScalar m13, BScalar m14, BScalar m15,
                    BScalar m20, BScalar m21, BScalar m22, BScalar m23, BScalar m24, BScalar m25,
                    BScalar m30, BScalar m31, BScalar m32, BScalar m33, BScalar m34, BScalar m35,
                    BScalar m40, BScalar m41, BScalar m42, BScalar m43, BScalar m44, BScalar m45,
                    BScalar m50, BScalar m51, BScalar m52, BScalar m53, BScalar m54, BScalar m55 ) 
    {
        set(m00, m01, m02, m03, m04, m05,
            m10, m11, m12, m13, m14, m15,
            m20, m21, m22, m23, m24, m25,
            m30, m31, m32, m33, m34, m35,
            m40, m41, m42, m43, m44, m45,
            m50, m51, m52, m53, m54, m55);
    }
    
    explicit BSpatialMatrix( const BMatrix3 &m0, const BMatrix3 &m1, 
                             const BMatrix3 &m2, const BMatrix3 &m3 )
    {
        set(m0, m1, m2, m3);
    }
    
    ~BSpatialMatrix( void )=default;


    void
    set( BScalar s ) 
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] = s;
    }
    
    void 
    set( const std::vector<std::vector<BScalar>> &d )  
    {  
        assert(m_data.size() == 6 && m_data[0].size() == 6);  
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] = d[i][j];
    }
    
    void 
    set( BScalar m00, BScalar m01, BScalar m02, BScalar m03, BScalar m04, BScalar m05,
         BScalar m10, BScalar m11, BScalar m12, BScalar m13, BScalar m14, BScalar m15,
         BScalar m20, BScalar m21, BScalar m22, BScalar m23, BScalar m24, BScalar m25,
         BScalar m30, BScalar m31, BScalar m32, BScalar m33, BScalar m34, BScalar m35,
         BScalar m40, BScalar m41, BScalar m42, BScalar m43, BScalar m44, BScalar m45,
         BScalar m50, BScalar m51, BScalar m52, BScalar m53, BScalar m54, BScalar m55 )
    {
        m_data[0][0] = m00; m_data[0][1] = m01;  m_data[0][2] = m02; m_data[0][3] = m03;  m_data[0][4] = m04; m_data[0][5] = m05;
        m_data[1][0] = m10; m_data[1][1] = m11;  m_data[1][2] = m12; m_data[1][3] = m13;  m_data[1][4] = m14; m_data[1][5] = m15;
        m_data[2][0] = m20; m_data[2][1] = m21;  m_data[2][2] = m22; m_data[2][3] = m23;  m_data[2][4] = m24; m_data[2][5] = m25;
        m_data[3][0] = m30; m_data[3][1] = m31;  m_data[3][2] = m32; m_data[3][3] = m33;  m_data[3][4] = m34; m_data[3][5] = m35;
        m_data[4][0] = m40; m_data[4][1] = m41;  m_data[4][2] = m42; m_data[4][3] = m43;  m_data[4][4] = m44; m_data[4][5] = m45;
        m_data[5][0] = m50; m_data[5][1] = m51;  m_data[5][2] = m52; m_data[5][3] = m53;  m_data[5][4] = m54; m_data[5][5] = m55;
    }

    void 
    set( const BMatrix3 &m0, const BMatrix3 &m1, const BMatrix3 &m2, const BMatrix3 &m3 )
    {
        m_data[0][0] = m0[0][0]; m_data[0][1] = m0[0][1]; m_data[0][2] = m0[0][2];  m_data[0][3] = m1[0][0]; m_data[0][4] = m1[0][1]; m_data[0][5] = m1[0][2];
        m_data[1][0] = m0[1][0]; m_data[1][1] = m0[1][1]; m_data[1][2] = m0[1][2];  m_data[1][3] = m1[1][0]; m_data[1][4] = m1[1][1]; m_data[1][5] = m1[1][2];
        m_data[2][0] = m0[2][0]; m_data[2][1] = m0[2][1]; m_data[2][2] = m0[2][2];  m_data[2][3] = m1[2][0]; m_data[2][4] = m1[2][1]; m_data[2][5] = m1[2][2];
        
        m_data[3][0] = m2[0][0]; m_data[3][1] = m2[0][1]; m_data[3][2] = m2[0][2];  m_data[3][3] = m3[0][0]; m_data[3][4] = m3[0][1]; m_data[3][5] = m3[0][2];
        m_data[4][0] = m2[1][0]; m_data[4][1] = m2[1][1]; m_data[4][2] = m2[1][2];  m_data[4][3] = m3[1][0]; m_data[4][4] = m3[1][1]; m_data[4][5] = m3[1][2];
        m_data[5][0] = m2[2][0]; m_data[5][1] = m2[2][1]; m_data[5][2] = m2[2][2];  m_data[5][3] = m3[2][0]; m_data[5][4] = m3[2][1]; m_data[5][5] = m3[2][2];
    }
    
    std::array<BScalar, 6>&
    operator[]( int i ) { return m_data[i]; }
    
    const std::array<BScalar, 6>&
    operator[]( int i ) const { return m_data[i]; }
    
    size_t 
    rows( void ) const { return 6; }
    
    size_t 
    cols( void ) const { return 6; }
    
    std::array<std::array<BScalar, 6>, 6>&
    data( void ) { return m_data; }
    
    const std::array<std::array<BScalar, 6>, 6>&
    data( void ) const { return m_data; }
    
    const BSpatialMatrix
    operator+( const BSpatialMatrix &m ) const
    {
        BSpatialMatrix retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = m_data[i][j] + m[i][j];
        return retVal;
    }
    
    const BSpatialMatrix
    operator-( const BSpatialMatrix &m ) const
    {
        BSpatialMatrix retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = m_data[i][j] - m[i][j];
        return retVal;
    }
    
    const BSpatialMatrix
    operator-( void ) const
    {
        BSpatialMatrix retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = -m_data[i][j];
        return retVal;
    }
    
    const BSpatialMatrix&
    operator-=( const BSpatialMatrix &m )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] -= m[i][j];
        return *this;
    }
    
    const BSpatialMatrix&
    operator+=( const BSpatialMatrix &m )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] += m[i][j];
        return *this;    
    }
    
    const BSpatialMatrix&
    operator/=( BScalar s )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] /= s;
        return *this;    
    }
    
    const BSpatialMatrix&
    operator*=( BScalar s )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] *= s;
        return *this;    
    }

    const BSpatialVector 
    operator*( const BSpatialVector &v ) const 
    {    
        BSpatialVector retVal(0.0);
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i] += m_data[i][j] * v[j];
        return retVal;
    }
        
    const BSpatialMatrix 
    operator*( const BSpatialMatrix &m ) const
    {   
        BSpatialMatrix retVal(0.0);
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                for ( int k = 0; k < 6; ++k )
                    retVal[i][j] += m_data[i][k] * m[k][j];
        return retVal;
    }

    const BMatrix63 
    operator*( const BMatrix63 &m ) const
    {    
        BMatrix63 retVal(0.0);
        for ( int i = 0; i < 6; ++i ) 
            for ( int j = 0; j < 3; ++j )
                for ( int k = 0; k < 6; ++k ) 
                    retVal[i][j] += m_data[i][k] * m[k][j];
        return retVal;
    }

    const BSpatialMatrix
    operator*( BScalar s ) const
    { 
        BSpatialMatrix retVal;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                retVal[i][j] = m_data[i][j] * s;
        return retVal; 
    }

    const BMatrix3
    topLeft( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i][j] = m_data[i][j];
        return retVal; 
    }
    
    const BMatrix3
    topRight( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 3; j < 6; ++j )
                retVal[i][j] = m_data[i][j];
        return retVal; 
    }
    
    const BMatrix3
    botLeft( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 3; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i][j] = m_data[i][j];
        return retVal; 
    }
    
    const BMatrix3
    botRight( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 3; i < 6; ++i )
            for ( int j = 3; j < 6; ++j )
                retVal[i][j] = m_data[i][j];
        return retVal; 
    }

    
    bool 
    operator==( const BSpatialMatrix& m ) const { return (m_data == m.m_data); }
    
    bool 
    operator!=( const BSpatialMatrix& m ) const { return (m_data != m.m_data); }
    
private:

    std::array<std::array<BScalar, 6>, 6> m_data;
};


inline const BSpatialMatrix 
operator*( BScalar s, const BSpatialMatrix &m ) 
// scalar multiplication
{ 
    return m * s; 
} 

inline const BSpatialMatrix 
operator*( const BMatrix63 &m1, const BMatrix36 &m2 )  
{
    BSpatialMatrix retVal(0.0);
    for ( int i = 0; i < 6; ++i )
        for ( int j = 0; j < 6; ++j )
            for ( int k = 0; k < 3; ++k )
                retVal[i][j] += m1[i][k] * m2[k][j];
    return retVal;
}

//
// Articulated Rigid Body
//

namespace arb
{

    inline const BSpatialMatrix
    transpose( const BSpatialMatrix &m )
    {
        BSpatialMatrix retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = m[j][i];
        return retVal;    
    }

    // was called Math::VectorCrossMatrix 
    inline const BMatrix3 
    cross_matrix( const BVector3 &v ) 
    {
        return BMatrix3(  0.0, -v[2],  v[1],
                         v[2],   0.0, -v[0],
                        -v[1],  v[0],   0.0 );
    }
};


inline std::ostream&
operator<<( std::ostream& ostr, const BSpatialMatrix& m )
{
    ostr << m[0][0] << ' ' << m[0][1] << ' ' << m[0][2] << ' ' << m[0][3] << ' ' << m[0][4] << ' ' << m[0][5] << '\n'
         << m[1][0] << ' ' << m[1][1] << ' ' << m[1][2] << ' ' << m[1][3] << ' ' << m[1][4] << ' ' << m[1][5] << '\n'
         << m[2][0] << ' ' << m[2][1] << ' ' << m[2][2] << ' ' << m[2][3] << ' ' << m[2][4] << ' ' << m[2][5] << '\n'
         << m[3][0] << ' ' << m[3][1] << ' ' << m[3][2] << ' ' << m[3][3] << ' ' << m[3][4] << ' ' << m[3][5] << '\n'
         << m[4][0] << ' ' << m[4][1] << ' ' << m[4][2] << ' ' << m[4][3] << ' ' << m[4][4] << ' ' << m[4][5] << '\n'
         << m[5][0] << ' ' << m[5][1] << ' ' << m[5][2] << ' ' << m[5][3] << ' ' << m[5][4] << ' ' << m[5][5] << ' ';
    return ostr;
}


inline std::istream& 
operator>>( std::istream& istr, BSpatialMatrix& m )
{
    istr >> m[0][0] >> m[0][1] >> m[0][2] >> m[0][3] >> m[0][4] >> m[0][5]
         >> m[1][0] >> m[1][1] >> m[1][2] >> m[1][3] >> m[1][4] >> m[1][5]
         >> m[2][0] >> m[2][1] >> m[2][2] >> m[2][3] >> m[2][4] >> m[2][5]
         >> m[3][0] >> m[3][1] >> m[3][2] >> m[3][3] >> m[3][4] >> m[3][5]
         >> m[4][0] >> m[4][1] >> m[4][2] >> m[4][3] >> m[4][4] >> m[4][5]
         >> m[5][0] >> m[5][1] >> m[5][2] >> m[5][3] >> m[5][4] >> m[5][5];
    return istr;
}



#endif


