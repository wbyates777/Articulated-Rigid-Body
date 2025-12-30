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



#ifndef __BMATRIX6_H__
#include "BMatrix6.h"
#endif


class BMatrix63
{
    
public:
    
    BMatrix63( void )=default;
    constexpr BMatrix63( const std::array<std::array<BScalar, 3>, 6> &d): m_data(d) {}
    explicit BMatrix63( const std::vector<std::vector<BScalar>> &d ) { set(d); }
    
    constexpr explicit BMatrix63( BScalar s ) : m_data 
                                                {   
                                                    s,  s,  s, 
                                                    s,  s,  s, 
                                                    s,  s,  s, 
                                                    s,  s,  s, 
                                                    s,  s,  s, 
                                                    s,  s,  s 
                                                } {}
    
    BMatrix63( const BMatrix3 &t, const BMatrix3 &b ) : m_data
                                                        {
                                                            t[0][0], t[0][1], t[0][2],
                                                            t[1][0], t[1][1], t[1][2],
                                                            t[2][0], t[2][1], t[2][2],
                                                            
                                                            b[0][0], b[0][1], b[0][2],
                                                            b[1][0], b[1][1], b[1][2],
                                                            b[2][0], b[2][1], b[2][2]
                                                        } {}
    
    constexpr BMatrix63( BScalar m00, BScalar m01, BScalar m02, 
                         BScalar m10, BScalar m11, BScalar m12, 
                         BScalar m20, BScalar m21, BScalar m22, 
                         BScalar m30, BScalar m31, BScalar m32, 
                         BScalar m40, BScalar m41, BScalar m42, 
                         BScalar m50, BScalar m51, BScalar m52 )  : m_data  
                                                                    {   
                                                                        m00,  m01,  m02, 
                                                                        m10,  m11,  m12, 
                                                                        m20,  m21,  m22, 
                                                                        m30,  m31,  m32, 
                                                                        m40,  m41,  m42, 
                                                                        m50,  m51,  m52 
                                                                    } {}
    
    ~BMatrix63( void )=default;
    
    void
    set( BScalar s ) 
    {
        m_data = 
        {   
            s,  s,  s, 
            s,  s,  s, 
            s,  s,  s, 
            s,  s,  s, 
            s,  s,  s, 
            s,  s,  s 
        };
    }
    
    void
    set( const std::vector<std::vector<BScalar>> &d ) 
    {
        assert(d.size() == 6 && d[0].size() == 3);
        m_data = 
        {   
            d[0][0],  d[0][1],  d[0][2], 
            d[1][0],  d[1][1],  d[1][2], 
            d[2][0],  d[2][1],  d[2][2], 
            d[3][0],  d[3][1],  d[3][2], 
            d[4][0],  d[4][1],  d[4][2], 
            d[5][0],  d[5][1],  d[5][2] 
        };
    }
    
    void 
    set( BScalar m00, BScalar m01, BScalar m02, 
         BScalar m10, BScalar m11, BScalar m12, 
         BScalar m20, BScalar m21, BScalar m22, 
         BScalar m30, BScalar m31, BScalar m32, 
         BScalar m40, BScalar m41, BScalar m42, 
         BScalar m50, BScalar m51, BScalar m52 )
    {
        m_data = 
        {   
            m00,  m01,  m02, 
            m10,  m11,  m12, 
            m20,  m21,  m22, 
            m30,  m31,  m32, 
            m40,  m41,  m42, 
            m50,  m51,  m52 
        };
    }
    
    void 
    set( const BMatrix3 &top, const BMatrix3 &bot ) 
    {
        m_data = 
        {
            top[0][0], top[0][1], top[0][2],
            top[1][0], top[1][1], top[1][2],
            top[2][0], top[2][1], top[2][2],
            
            bot[0][0], bot[0][1], bot[0][2],
            bot[1][0], bot[1][1], bot[1][2],
            bot[2][0], bot[2][1], bot[2][2]
        };
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
    
    
    const BMatrix3
    top( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i][j] = m_data[i][j];
        return retVal; 
    }
    
    void
    top( const BMatrix3 &m )  
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j] = m[i][j];
    }
    
    const BMatrix3
    bot( void ) const 
    {
        BMatrix3 retVal; 
        for ( int i = 3; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i-3][j] = m_data[i][j];
        return retVal; 
    }
    
    void
    bot( const BMatrix3 &m )  
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i+3][j] = m[i][j];
    }

    const BVector6 
    operator*( const BVector3 &v ) const 
    {
        BVector6 retVal;
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

    const BMatrix63
    operator/( BScalar s ) const
    { 
        BMatrix63 retVal;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j)
                retVal[i][j] = m_data[i][j] / s;
        return retVal; 
    }

    const BMatrix63&
    operator/=( BScalar s )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j] /= s;
        return *this;    
    }
    
    const BMatrix63
    operator-( const BMatrix63 &m ) const
    {
        BMatrix63 retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i][j] = m_data[i][j] - m[i][j];
        return retVal;
    }
 
    const BMatrix63&
    operator-=( const BMatrix63 &m )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j] -= m[i][j];
        return *this;
    }
    
    const BMatrix63
    operator-( void ) const
    {
        BMatrix63 retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i][j] = -m_data[i][j];
        return retVal;
    }
    
    const BMatrix63
    operator+( const BMatrix63 &m ) const
    {
        BMatrix63 retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i][j] = m_data[i][j] + m[i][j];
        return retVal;
    }
    
    const BMatrix63&
    operator+=( const BMatrix63 &m )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j] += m[i][j];
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
         << m[5][0] << ' ' << m[5][1] << ' ' << m[5][2] << '\n';
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
    constexpr BMatrix36( const std::array<std::array<BScalar, 6>, 3> &d ): m_data(d) {}
    explicit BMatrix36( const std::vector<std::vector<BScalar>> &d ) { set(d); }
    
    constexpr explicit BMatrix36( BScalar s ) : m_data 
                                                {
                                                    s, s, s, s, s, s,
                                                    s, s, s, s, s, s,
                                                    s, s, s, s, s, s, 
                                                } {}
    
    BMatrix36( const BMatrix3 &l, const BMatrix3 &r ) : m_data
                                                        {
                                                            l[0][0], l[0][1], l[0][2],  r[0][0], r[0][1], r[0][2],
                                                            l[1][0], l[1][1], l[1][2],  r[1][0], r[1][1], r[1][2],
                                                            l[2][0], l[2][1], l[2][2],  r[2][0], r[2][1], r[2][2]
                                                        } {}

    
    constexpr BMatrix36( BScalar m00, BScalar m01, BScalar m02, BScalar m03, BScalar m04, BScalar m05,
                         BScalar m10, BScalar m11, BScalar m12, BScalar m13, BScalar m14, BScalar m15,
                         BScalar m20, BScalar m21, BScalar m22, BScalar m23, BScalar m24, BScalar m25 ) 
                         : m_data
                         {
                             m00, m01, m02, m03, m04, m05,
                             m10, m11, m12, m13, m14, m15,
                             m20, m21, m22, m23, m24, m25, 
                         } {}
    
    ~BMatrix36( void )=default;

    void
    set( BScalar s ) 
    {
        m_data = 
        {
            s, s, s, s, s, s,
            s, s, s, s, s, s,
            s, s, s, s, s, s, 
        };
    }
    
    void 
    set( const std::vector<std::vector<BScalar>> &d )  
    {  
        assert(d.size() == 3 && d[0].size() == 6);  
        m_data = 
        {
            d[0][0], d[0][1], d[0][2], d[0][3], d[0][4], d[0][5],
            d[1][0], d[1][1], d[1][2], d[1][3], d[1][4], d[1][5],
            d[2][0], d[2][1], d[2][2], d[2][3], d[2][4], d[2][5] 
        };
    }
    
    void 
    set( BScalar m00, BScalar m01, BScalar m02, BScalar m03, BScalar m04, BScalar m05,
         BScalar m10, BScalar m11, BScalar m12, BScalar m13, BScalar m14, BScalar m15,
         BScalar m20, BScalar m21, BScalar m22, BScalar m23, BScalar m24, BScalar m25 )
    {
        m_data = 
        {
            m00, m01, m02, m03, m04, m05,
            m10, m11, m12, m13, m14, m15,
            m20, m21, m22, m23, m24, m25, 
        };
    }
    
    void 
    set( const BMatrix3 &left, const BMatrix3 &right ) 
    {
        m_data = 
        {
            left[0][0], left[0][1], left[0][2],  right[0][0], right[0][1], right[0][2],
            left[1][0], left[1][1], left[1][2],  right[1][0], right[1][1], right[1][2],
            left[2][0], left[2][1], left[2][2],  right[2][0], right[2][1], right[2][2]
        };
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
    
    
    const BMatrix3
    left( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i][j] = m_data[i][j];
        return retVal; 
    }
    
    void
    left( const BMatrix3 &m )  
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j] = m[i][j];
    }
    
    const BMatrix3
    right( void ) const 
    {
        BMatrix3 retVal; 
        for ( int i = 0; i < 3; ++i )
            for ( int j = 3; j < 6; ++j )
                retVal[i][j-3] = m_data[i][j];
        return retVal; 
    }
    
    void
    right( const BMatrix3 &m )  
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j+3] = m[i][j];
    }
    
    const BVector3
    operator*( const BVector6 &v ) const 
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
    
    const BMatrix36
    operator/( BScalar s ) const
    { 
        BMatrix36 retVal;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 6; ++j)
                retVal[i][j] = m_data[i][j] / s;
        return retVal; 
    }

    const BMatrix36&
    operator/=( BScalar s )
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] /= s;
        return *this;    
    }
    
    const BMatrix36
    operator-( const BMatrix36 &m ) const
    {
        BMatrix36 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = m_data[i][j] - m[i][j];
        return retVal;
    }
 
    const BMatrix36&
    operator-=( const BMatrix36 &m )
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] -= m[i][j];
        return *this;
    }
    
    const BMatrix36
    operator-( void ) const
    {
        BMatrix36 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = -m_data[i][j];
        return retVal;
    }
    
    const BMatrix36
    operator+( const BMatrix36 &m ) const
    {
        BMatrix36 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = m_data[i][j] + m[i][j];
        return retVal;
    }
    
    const BMatrix36&
    operator+=( const BMatrix36 &m )
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] += m[i][j];
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

inline const BMatrix6 
operator*( const BMatrix63 &m1, const BMatrix36 &m2 )  
{
    BMatrix6 retVal(B_ZERO_6x6);
    for ( int i = 0; i < 6; ++i )
        for ( int j = 0; j < 6; ++j )
            for ( int k = 0; k < 3; ++k )
                retVal[i][j] += m1[i][k] * m2[k][j];
    return retVal;
}

inline const BMatrix63 
operator*( const BMatrix6 &m1, const BMatrix63 &m2 ) 
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
         << m[2][0] << ' ' << m[2][1] << ' ' << m[2][2] << ' ' << m[2][3] << ' ' << m[2][4] << ' ' << m[2][5] << '\n';
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
    inline bool 
    nearZero( const BMatrix63 &m )  { return (nearZero(m.top()) && nearZero(m.bot())); }

    inline bool 
    isnan( const BMatrix63 &m ) { return (isnan(m.top()) || isnan(m.bot())); }


    inline bool 
    nearZero( const BMatrix36 &m ) { return (nearZero(m.left()) && nearZero(m.right())); }

    inline bool 
    isnan( const BMatrix36 &m ) { return (isnan(m.left()) || isnan(m.right())); }


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
} 

#endif


