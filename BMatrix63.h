/* BMatrix63 20/02/2024

 $$$$$$$$$$$$$$$$$$$
 $   BMatrix63.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
*/


#ifndef __BMATRIX63_H__
#define __BMATRIX63_H__



#ifndef __BSPATIALMATRIX_H__
#include "BSpatialMatrix.h"
#endif


class BMatrix63
{
    
public:
    
    BMatrix63( void )=default;
    explicit BMatrix63( BScalar s ) { set(s); }
    
    BMatrix63( const std::array<std::array<BScalar, 3>, 6> &d): m_data(d) {}
    
    explicit BMatrix63( const std::vector<std::vector<BScalar>> &d ) 
    {
        assert(d.size() == 6 && d[0].size() == 3);
        set(d);
    }
    BMatrix63( BScalar m00, BScalar m01, BScalar m02, 
               BScalar m10, BScalar m11, BScalar m12, 
               BScalar m20, BScalar m21, BScalar m22, 
               BScalar m30, BScalar m31, BScalar m32, 
               BScalar m40, BScalar m41, BScalar m42, 
               BScalar m50, BScalar m51, BScalar m52 ) 
    { 
        set(m00, m01, m02,
            m10, m11, m12,
            m20, m21, m22, 
            m30, m31, m32,
            m40, m41, m42, 
            m50, m51, m52);
    }
    
    ~BMatrix63( void )=default;
    
    void
    set( BScalar s ) 
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j] = s;
    }
    
    void
    set( const std::vector<std::vector<BScalar>> &d ) 
    {
        assert(d.size() == 6 && d[0].size() == 3);
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j] = d[i][j];
    }
    
    void 
    set( BScalar m00, BScalar m01, BScalar m02, 
         BScalar m10, BScalar m11, BScalar m12, 
         BScalar m20, BScalar m21, BScalar m22, 
         BScalar m30, BScalar m31, BScalar m32, 
         BScalar m40, BScalar m41, BScalar m42, 
         BScalar m50, BScalar m51, BScalar m52 )
    {
        m_data[0][0] = m00; m_data[0][1] = m01; m_data[0][2] = m02;
        m_data[1][0] = m10; m_data[1][1] = m11; m_data[1][2] = m12;
        m_data[2][0] = m20; m_data[2][1] = m21; m_data[2][2] = m22;
        m_data[3][0] = m30; m_data[3][1] = m31; m_data[3][2] = m32;
        m_data[4][0] = m40; m_data[4][1] = m41; m_data[4][2] = m42;
        m_data[5][0] = m50; m_data[5][1] = m51; m_data[5][2] = m52;
    }
    
    std::array<BScalar, 3>&
    operator[]( int i ) { return m_data[i]; }
    
    const std::array<BScalar, 3>&
    operator[]( int i ) const { return m_data[i]; }
    
    static size_t 
    rows( void ) { return 6; }
    
    static size_t 
    cols( void ) { return 3; }
    
    std::array<std::array<BScalar, 3>, 6>&
    data( void ) { return m_data; }
    
    const std::array<std::array<BScalar, 3>, 6>&
    data( void ) const { return m_data; }
    
    const BSpatialVector 
    operator*( const BVector3 &v ) const 
    {
        BSpatialVector retVal;
        for ( int i = 0; i < 6; ++i )
            retVal[i] = glm::dot(BVector3(m_data[i][0], m_data[i][1], m_data[i][2]), v);
        return retVal; 
    }

    const BMatrix63 
    operator*( const BMatrix3 &m ) const
    {
        BMatrix63 retVal(B_ZERO_6x3);
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j ) 
                for ( int k = 0; k < 3; ++k )
                    retVal[i][j] += m_data[i][k] * m[k][j];
        return retVal;
    }
    
    const BMatrix63
    operator*( BScalar s ) const
    { 
        BMatrix63 retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )  
                retVal[i][j] = m_data[i][j] * s;
        return retVal; 
    }

    const BMatrix63&
    operator*=( BScalar s )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j] *= s;
        return *this;    
    }

    bool 
    operator==( const BMatrix63 &v ) const { return (m_data == v.m_data); }
    
    bool 
    operator!=( const BMatrix63 &v ) const { return (m_data != v.m_data); }

    
private:
    
    std::array<std::array<BScalar, 3>, 6> m_data;

};

// scalar multiplication
inline const BMatrix63 
operator*( BScalar s, const BMatrix63 &v ) { return v * s; }


inline std::ostream&
operator<<( std::ostream &ostr, const BMatrix63 &m )
{
    ostr << m[0][0] << ' ' << m[0][1] << ' ' << m[0][2] << '\n'
         << m[1][0] << ' ' << m[1][1] << ' ' << m[1][2] << '\n'
         << m[2][0] << ' ' << m[2][1] << ' ' << m[2][2] << '\n'
         << m[3][0] << ' ' << m[3][1] << ' ' << m[3][2] << '\n' 
         << m[4][0] << ' ' << m[4][1] << ' ' << m[4][2] << '\n' 
         << m[5][0] << ' ' << m[5][1] << ' ' << m[5][2] << ' ';
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BMatrix63 &m )
{
    istr >> m[0][0] >> m[0][1] >> m[0][2] 
         >> m[1][0] >> m[1][1] >> m[1][2] 
         >> m[2][0] >> m[2][1] >> m[2][2] 
         >> m[3][0] >> m[3][1] >> m[3][2] 
         >> m[4][0] >> m[4][1] >> m[4][2] 
         >> m[5][0] >> m[5][1] >> m[5][2];
    return istr;
}


//
//
//


class BMatrix36
{

public:

    BMatrix36( void )=default;
    explicit BMatrix36( BScalar s ) { set(s); }
    BMatrix36( const std::array<std::array<BScalar, 6>, 3> &d ): m_data(d) {}
    
    explicit BMatrix36( const std::vector<std::vector<BScalar>> &d )
    {
        assert(d.size() == 3 && d[0].size() == 6);
        set(d);
    }
    BMatrix36( BScalar m00, BScalar m01, BScalar m02, BScalar m03, BScalar m04, BScalar m05,
               BScalar m10, BScalar m11, BScalar m12, BScalar m13, BScalar m14, BScalar m15,
               BScalar m20, BScalar m21, BScalar m22, BScalar m23, BScalar m24, BScalar m25 ) 
    {
        set( m00, m01, m02, m03, m04, m05,
             m10, m11, m12, m13, m14, m15,
             m20, m21, m22, m23, m24, m25 );
    }
    ~BMatrix36( void )=default;

    void
    set( BScalar s ) 
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] = s;
    }
    
    void 
    set( const std::vector<std::vector<BScalar>> &d )  
    {  
        assert(d.size() == 3 && d[0].size() == 6);  
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] = d[i][j];
    }
    
    void 
    set( BScalar m00, BScalar m01, BScalar m02, BScalar m03, BScalar m04, BScalar m05,
         BScalar m10, BScalar m11, BScalar m12, BScalar m13, BScalar m14, BScalar m15,
         BScalar m20, BScalar m21, BScalar m22, BScalar m23, BScalar m24, BScalar m25 )
    {
        m_data[0][0] = m00; m_data[0][1] = m01; m_data[0][2] = m02; m_data[0][3] = m03; m_data[0][4] = m04; m_data[0][5] = m05;
        m_data[1][0] = m10; m_data[1][1] = m11; m_data[1][2] = m12; m_data[1][3] = m13; m_data[1][4] = m14; m_data[1][5] = m15;
        m_data[2][0] = m20; m_data[2][1] = m21; m_data[2][2] = m22; m_data[2][3] = m23; m_data[2][4] = m24; m_data[2][5] = m25;
    }
    
    std::array<BScalar, 6>&
    operator[]( int i ) { return m_data[i]; }
    
    const std::array<BScalar, 6>&
    operator[]( int i ) const { return m_data[i]; }
    
    static size_t 
    rows( void ) { return 3; }
    
    static size_t 
    cols( void ) { return 6; }

    std::array<std::array<BScalar, 6>, 3>&
    data( void ) { return m_data; }
    
    const std::array<std::array<BScalar, 6>, 3>&
    data( void ) const { return m_data; }
    
    const BVector3
    operator*( const BSpatialVector &v ) const 
    {
        BVector3 retVal(B_ZERO_3);
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i] += m_data[i][j] * v[j];
        return retVal; 
    }

    const BMatrix3 
    operator*( const BMatrix63 &m ) const  
    {
        BMatrix3 retVal(B_ZERO_3x3);
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                for ( int k = 0; k < 6; ++k )
                    retVal[i][j] += m_data[i][k] * m[k][j];
        return retVal;
    }

    const BMatrix36
    operator*( BScalar s ) const
    { 
        BMatrix36 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )  
                retVal[i][j] = m_data[i][j] * s;
        return retVal; 
    }

    const BMatrix36&
    operator*=( BScalar s )
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] *= s;
        return *this;    
    }
    
    bool 
    operator==( const BMatrix36 &v ) const { return (m_data == v.m_data); }
    
    bool 
    operator!=( const BMatrix36 &v ) const { return (m_data != v.m_data); }
    
private:
    
    std::array<std::array<BScalar, 6>, 3> m_data;
};

// scalar multiplication
inline const BMatrix36 
operator*( BScalar s, const BMatrix36 &m ) { return m * s; }

inline const BSpatialMatrix 
operator*( const BMatrix63 &m1, const BMatrix36 &m2 )  
{
    BSpatialMatrix retVal(B_ZERO_6x6);
    for ( int i = 0; i < 6; ++i )
        for ( int j = 0; j < 6; ++j )
            for ( int k = 0; k < 3; ++k )
                retVal[i][j] += m1[i][k] * m2[k][j];
    return retVal;
}

inline const BMatrix63 //  move this out of this class
operator*( const BSpatialMatrix &m1, const BMatrix63 &m2 ) 
{    
    BMatrix63 retVal(B_ZERO_6x3);
    for ( int i = 0; i < 6; ++i ) 
        for ( int j = 0; j < 3; ++j )
            for ( int k = 0; k < 6; ++k ) 
                retVal[i][j] += m1[i][k] * m2[k][j];
    return retVal;
}

inline std::ostream&
operator<<( std::ostream &ostr, const BMatrix36 &m )
{
    ostr << m[0][0] << ' ' << m[0][1] << ' ' << m[0][2] << ' ' << m[0][3] << ' ' << m[0][4] << ' ' << m[0][5] << '\n'
         << m[1][0] << ' ' << m[1][1] << ' ' << m[1][2] << ' ' << m[1][3] << ' ' << m[1][4] << ' ' << m[1][5] << '\n'
         << m[2][0] << ' ' << m[2][1] << ' ' << m[2][2] << ' ' << m[2][3] << ' ' << m[2][4] << ' ' << m[2][5] << ' ';
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BMatrix36 &m )
{
    istr >> m[0][0] >> m[0][1] >> m[0][2] >> m[0][3] >> m[0][4] >> m[0][5]
         >> m[1][0] >> m[1][1] >> m[1][2] >> m[1][3] >> m[1][4] >> m[1][5]
         >> m[2][0] >> m[2][1] >> m[2][2] >> m[2][3] >> m[2][4] >> m[2][5];
    return istr;
}

//
// Articulated Rigid Body
//

namespace arb
{

    inline const BMatrix63 
    transpose( const BMatrix36 &m ) 
    { 
        BMatrix63 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[j][i] = m[i][j];
        return retVal;
    }


    inline const BMatrix36 
    transpose( const BMatrix63 &m ) 
    {
        BMatrix36 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = m[j][i];
        return retVal;
    }

}; 

#endif


