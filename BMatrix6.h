/* Spatial Matrix  20/02/2024

 $$$$$$$$$$$$$$$$$$
 $   BMatrix6.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Spatial Matrix (see Featherstone, RBDA).
 
 Represents 6D spatial matrix. 
 Can be used to represent spatial transform or spatial inertia.


    M = | A B |
        | C D |

    N = | E F |
        | G H |

    M * N = | (AE + BG)  (AF + BH) |
            | (CE + DG)  (CF + DH) |
     
     
    V = | X |
        | Y |
      
    M * V =  | AX + BY |
             | CX + DY |
  

*/


#ifndef __BMATRIX6_H__
#define __BMATRIX6_H__

#ifndef __BVECTOR6_H__
#include "BVector6.h"
#endif


class BMatrix6
{

public:

    BMatrix6( void )=default;
    
    constexpr BMatrix6( const std::array<std::array<BScalar, 6>, 6>& d ): m_data(d) {}
    explicit BMatrix6( const std::vector<std::vector<BScalar>> &d )  { set(d); }
    explicit BMatrix6( const std::vector<BScalar> &d ) { set(d); }
    
    constexpr explicit BMatrix6( BScalar s ) :  m_data 
    {
        s,  s,  s,  s,  s,  s,
        s,  s,  s,  s,  s,  s,
        s,  s,  s,  s,  s,  s,
        s,  s,  s,  s,  s,  s,
        s,  s,  s,  s,  s,  s,
        s,  s,  s,  s,  s,  s
    } {}
    
    constexpr BMatrix6( BScalar m00, BScalar m01, BScalar m02, BScalar m03, BScalar m04, BScalar m05,
                        BScalar m10, BScalar m11, BScalar m12, BScalar m13, BScalar m14, BScalar m15,
                        BScalar m20, BScalar m21, BScalar m22, BScalar m23, BScalar m24, BScalar m25,
                        BScalar m30, BScalar m31, BScalar m32, BScalar m33, BScalar m34, BScalar m35,
                        BScalar m40, BScalar m41, BScalar m42, BScalar m43, BScalar m44, BScalar m45,
                        BScalar m50, BScalar m51, BScalar m52, BScalar m53, BScalar m54, BScalar m55 ) 
    : m_data 
    {
        m00,  m01,  m02,  m03,  m04,  m05,
        m10,  m11,  m12,  m13,  m14,  m15,
        m20,  m21,  m22,  m23,  m24,  m25,
        m30,  m31,  m32,  m33,  m34,  m35,
        m40,  m41,  m42,  m43,  m44,  m45,
        m50,  m51,  m52,  m53,  m54,  m55
    } {}
    
    constexpr BMatrix6( const BMatrix3 &m0, const BMatrix3 &m1, 
                        const BMatrix3 &m2, const BMatrix3 &m3 ) :  m_data 
    {
        m0[0][0], m0[0][1], m0[0][2], m1[0][0], m1[0][1], m1[0][2],
        m0[1][0], m0[1][1], m0[1][2], m1[1][0], m1[1][1], m1[1][2],
        m0[2][0], m0[2][1], m0[2][2], m1[2][0], m1[2][1], m1[2][2],
        
        m2[0][0], m2[0][1], m2[0][2], m3[0][0], m3[0][1], m3[0][2],
        m2[1][0], m2[1][1], m2[1][2], m3[1][0], m3[1][1], m3[1][2],
        m2[2][0], m2[2][1], m2[2][2], m3[2][0], m3[2][1], m3[2][2],
    } {}
    
    ~BMatrix6( void )=default;
    
    
    void
    set( BScalar s ) 
    {
        m_data = 
        {
            s,  s,  s,  s,  s,  s,
            s,  s,  s,  s,  s,  s,
            s,  s,  s,  s,  s,  s,
            s,  s,  s,  s,  s,  s,
            s,  s,  s,  s,  s,  s,
            s,  s,  s,  s,  s,  s
        };
    }
    
    void 
    set( const std::vector<std::vector<BScalar>> &d )  
    {  
        assert(m_data.size() == 6 && m_data[0].size() == 6);  
        m_data = 
        {
            d[0][0],  d[0][1],  d[0][2],  d[0][3],  d[0][4],  d[0][5],
            d[1][0],  d[1][1],  d[1][2],  d[1][3],  d[1][4],  d[1][5],
            d[2][0],  d[2][1],  d[2][2],  d[2][3],  d[2][4],  d[2][5],
            d[3][0],  d[3][1],  d[3][2],  d[3][3],  d[3][4],  d[3][5],
            d[4][0],  d[4][1],  d[4][2],  d[4][3],  d[4][4],  d[4][5],
            d[5][0],  d[5][1],  d[5][2],  d[5][3],  d[5][4],  d[5][5]
        };
    }
    
    void
    set( const std::vector<BScalar> &d )
    {
        assert(m_data.size() == 36);  
        m_data = 
        {
             d[0],   d[1],   d[2],   d[3],   d[4],   d[5],
             d[6],   d[7],   d[8],   d[9],  d[10],  d[11],
            d[12],  d[13],  d[14],  d[15],  d[16],  d[17],
            d[18],  d[19],  d[20],  d[21],  d[22],  d[23],
            d[24],  d[25],  d[26],  d[27],  d[28],  d[29],
            d[30],  d[31],  d[32],  d[33],  d[34],  d[35]
        }; 
    }
    
    void 
    set( BScalar m00, BScalar m01, BScalar m02, BScalar m03, BScalar m04, BScalar m05,
         BScalar m10, BScalar m11, BScalar m12, BScalar m13, BScalar m14, BScalar m15,
         BScalar m20, BScalar m21, BScalar m22, BScalar m23, BScalar m24, BScalar m25,
         BScalar m30, BScalar m31, BScalar m32, BScalar m33, BScalar m34, BScalar m35,
         BScalar m40, BScalar m41, BScalar m42, BScalar m43, BScalar m44, BScalar m45,
         BScalar m50, BScalar m51, BScalar m52, BScalar m53, BScalar m54, BScalar m55 )
    {
        m_data = 
        {
            m00,  m01,  m02,  m03,  m04,  m05,
            m10,  m11,  m12,  m13,  m14,  m15,
            m20,  m21,  m22,  m23,  m24,  m25,
            m30,  m31,  m32,  m33,  m34,  m35,
            m40,  m41,  m42,  m43,  m44,  m45,
            m50,  m51,  m52,  m53,  m54,  m55
        };
    }
    
    
    void 
    set( const BMatrix3 &m0, const BMatrix3 &m1, const BMatrix3 &m2, const BMatrix3 &m3 )
    {
        m_data = 
        {
            m0[0][0], m0[0][1], m0[0][2], m1[0][0], m1[0][1], m1[0][2],
            m0[1][0], m0[1][1], m0[1][2], m1[1][0], m1[1][1], m1[1][2],
            m0[2][0], m0[2][1], m0[2][2], m1[2][0], m1[2][1], m1[2][2],
            
            m2[0][0], m2[0][1], m2[0][2], m3[0][0], m3[0][1], m3[0][2],
            m2[1][0], m2[1][1], m2[1][2], m3[1][0], m3[1][1], m3[1][2],
            m2[2][0], m2[2][1], m2[2][2], m3[2][0], m3[2][1], m3[2][2],
        };
    }
     
    std::array<BScalar, 6>&
    operator[]( int i ) { return m_data[i]; }
    
    const std::array<BScalar, 6>&
    operator[]( int i ) const { return m_data[i]; }
    
    static size_t 
    rows( void ) { return 6; }
    
    static size_t 
    cols( void ) { return 6; }
    
    std::array<std::array<BScalar, 6>, 6>&
    data( void ) { return m_data; }
    
    const std::array<std::array<BScalar, 6>, 6>&
    data( void ) const { return m_data; }
    
    
    BMatrix3
    topLeft( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i][j] = m_data[i][j];
        return retVal; 
    }
    
    void
    topLeft( const BMatrix3 &m )  
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j] = m[i][j];
    }
    
    BMatrix3
    topRight( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 0; i < 3; ++i )
            for ( int j = 3; j < 6; ++j )
                retVal[i][j-3] = m_data[i][j];
        return retVal; 
    }
    
    void
    topRight( const BMatrix3 &m )  
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i][j+3] = m[i][j];
    }
    
    BMatrix3
    botLeft( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 3; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                retVal[i-3][j] = m_data[i][j];
        return retVal; 
    }
    
    void
    botLeft( const BMatrix3 &m )  
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i+3][j] = m[i][j];
    }
    
    BMatrix3
    botRight( void ) const 
    {
        BMatrix3 retVal;
        for ( int i = 3; i < 6; ++i )
            for ( int j = 3; j < 6; ++j )
                retVal[i-3][j-3] = m_data[i][j];
        return retVal; 
    }

    void
    botRight( const BMatrix3 &m )  
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                m_data[i+3][j+3] = m[i][j];
    }
    

    BMatrix6
    operator*( BScalar s ) const
    { 
        BMatrix6 retVal;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                retVal[i][j] = m_data[i][j] * s;
        return retVal; 
    }
    
    BMatrix6&
    operator*=( BScalar s )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] *= s;
        return *this;    
    }

    BMatrix6
    operator/( BScalar s ) const
    { 
        BMatrix6 retVal;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                retVal[i][j] = m_data[i][j] / s;
        return retVal; 
    }

    BMatrix6&
    operator/=( BScalar s )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] /= s;
        return *this;    
    }

    BMatrix6
    operator-( void ) const
    {
        BMatrix6 retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = -m_data[i][j];
        return retVal;
    }


    BMatrix6
    operator-( const BMatrix6 &m ) const
    {
        BMatrix6 retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = m_data[i][j] - m[i][j];
        return retVal;
    }
 
    BMatrix6&
    operator-=( const BMatrix6 &m )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] -= m[i][j];
        return *this;
    }
    
    BMatrix6
    operator+( const BMatrix6 &m ) const
    {
        BMatrix6 retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = m_data[i][j] + m[i][j];
        return retVal;
    }
    
    BMatrix6&
    operator+=( const BMatrix6 &m )
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                m_data[i][j] += m[i][j];
        return *this;    
    }

    
    BVector6  
    operator*( const BVector6 &v ) const 
    {    
       // BVector6 retVal(B_ZERO_6);
       // for ( int i = 0; i < 6; ++i )
       //     for ( int j = 0; j < 6; ++j )
       //         retVal[i] += m_data[i][j] * v[j];
       // return retVal;
        
        return BVector6( (arb::transpose(topLeft()) * v.ang()) + (arb::transpose(topRight()) * v.lin()),
                         (arb::transpose(botLeft()) * v.ang()) + (arb::transpose(botRight()) * v.lin()) );
    }
 
    
    BMatrix6 
    operator*( const BMatrix6 &m ) const
    {   
        /*BMatrix6 retVal(B_ZERO_6x6);
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
               for ( int k = 0; k < 6; ++k )
                    retVal[i][j] += m_data[i][k] * m[k][j];
        return retVal;*/
        
        const BMatrix3 tl1 = m.topLeft();
        const BMatrix3 tr1 = m.topRight();
        const BMatrix3 bl1 = m.botLeft();
        const BMatrix3 br1 = m.botRight();
        
        const BMatrix3 tl2 = topLeft();
        const BMatrix3 tr2 = topRight();
        const BMatrix3 bl2 = botLeft();
        const BMatrix3 br2 = botRight();
        
        return BMatrix6( (tl1 * tl2) + (bl1 * tr2), (tr1 * tl2) + (br1 * tr2),
                         (tl1 * bl2) + (bl1 * br2), (tr1 * bl2) + (br1 * br2) );
    }
    
    BMatrix6& 
    operator*=(const BMatrix6 &rhs)
    {
        return (*this = *this * rhs);
    }

    bool 
    operator==( const BMatrix6 &m ) const { return (m_data == m.m_data); }
    
    bool 
    operator!=( const BMatrix6 &m ) const { return (m_data != m.m_data); }
    
private:

    std::array<std::array<BScalar, 6>, 6> m_data;
};

// scalar multiplication
inline BMatrix6 
operator*( BScalar s, const BMatrix6 &m ) { return m * s; } 

namespace arb
{

    inline bool 
    nearZero( const BMatrix6 &m )  
    {  
        return    (nearZero(m.topLeft()) && nearZero(m.topRight()) 
                && nearZero(m.botLeft()) && nearZero(m.botRight()));
    }

    inline bool 
    isnan(const BMatrix6 &m) 
    { 
        return     (isnan(m.topLeft()) || isnan(m.topRight()) 
                 || isnan(m.botLeft()) || isnan(m.botRight())); 
    }

    inline constexpr BMatrix6
    transpose( const BMatrix6 &m ) 
    { 
        BMatrix6 retVal;
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 6; ++j )
                retVal[i][j] = m[j][i];
        return retVal;  
    }
}

inline std::ostream&
operator<<( std::ostream &ostr, const BMatrix6 &m )
{
    ostr << m[0][0] << ' ' << m[0][1] << ' ' << m[0][2] << ' ' << m[0][3] << ' ' << m[0][4] << ' ' << m[0][5] << '\n'
         << m[1][0] << ' ' << m[1][1] << ' ' << m[1][2] << ' ' << m[1][3] << ' ' << m[1][4] << ' ' << m[1][5] << '\n'
         << m[2][0] << ' ' << m[2][1] << ' ' << m[2][2] << ' ' << m[2][3] << ' ' << m[2][4] << ' ' << m[2][5] << '\n'
         << m[3][0] << ' ' << m[3][1] << ' ' << m[3][2] << ' ' << m[3][3] << ' ' << m[3][4] << ' ' << m[3][5] << '\n'
         << m[4][0] << ' ' << m[4][1] << ' ' << m[4][2] << ' ' << m[4][3] << ' ' << m[4][4] << ' ' << m[4][5] << '\n'
         << m[5][0] << ' ' << m[5][1] << ' ' << m[5][2] << ' ' << m[5][3] << ' ' << m[5][4] << ' ' << m[5][5] << '\n';
    return ostr;
}


inline std::istream& 
operator>>( std::istream &istr, BMatrix6 &m )
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



