/* BSpatialVector 31/01/2024

 $$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialVector.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Spatial Vector (see Featherstone, RBDA).
 
 Represents a 6D spatial vector, Could be a motion vector $m \in M^6$ 
 or a force vector $f \in F^6$.
 
 Colloquially, a spatial velocity vector is called a 'twist', 
 while a spatial force vector is called a 'wrench'.
 
*/


#ifndef __BSPATIALVECTOR_H__
#define __BSPATIALVECTOR_H__


#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif


class BSpatialVector
{

public:
    
    BSpatialVector( void )=default;
    constexpr BSpatialVector( const std::array<BScalar, 6> &d ): m_data(d) {}
    constexpr explicit BSpatialVector( BScalar s ): m_data{s, s, s, s, s, s} {}
    explicit BSpatialVector( const std::vector<BScalar> &d ) { assert(d.size() == 6); set(d); }
    explicit BSpatialVector( const std::vector<std::vector<BScalar>> &d ) { assert(d.size() == 1 && d[0].size() == 6); set(d[0]); }
    constexpr BSpatialVector( BScalar s0, BScalar s1, BScalar s2, BScalar s3, BScalar s4, BScalar s5 ): m_data{s0, s1, s2, s3, s4, s5} {}
    BSpatialVector( const BVector3 &h, const BVector3 &t ): m_data{h[0], h[1], h[2], t[0], t[1], t[2]} {}
    ~BSpatialVector( void )=default;
    
    void
    set( BScalar s ) { m_data = { s, s, s, s, s, s }; }
    
    void 
    set( const std::vector<BScalar> &d ) 
    { 
        assert(d.size() == 6);
        m_data = { d[0], d[1], d[2], d[3], d[4], d[5] };
    }
    
    void 
    set( BScalar s0, BScalar s1, BScalar s2, BScalar s3, BScalar s4, BScalar s5 ) 
    { 
        m_data = { s0, s1, s2, s3, s4, s5 };
    }
    
    void 
    set( const BVector3 &h, const BVector3 &t ) 
    { 
        m_data = { h[0], h[1], h[2], t[0], t[1], t[2] };
    }
    
    // angular
    void
    ang( BScalar s ) { m_data[0] = s; m_data[1] = s; m_data[2] = s; }

    void
    ang( const BVector3 &v ) { m_data[0] = v[0]; m_data[1] = v[1]; m_data[2] = v[2]; }

    const BVector3
    ang( void ) const { return BVector3(m_data[0], m_data[1], m_data[2]); }

    // linear
    void
    lin( BScalar s ) { m_data[3] = s; m_data[4] = s; m_data[5] = s; }

    void
    lin( const BVector3 &v ) { m_data[3] = v[0]; m_data[4] = v[1]; m_data[5] = v[2]; }
    
    const BVector3
    lin( void ) const { return BVector3(m_data[3], m_data[4], m_data[5]); }
    
    BScalar&
    operator[]( int i ) { return m_data[i]; }
    
    const BScalar
    operator[]( int i ) const { return m_data[i]; }

    static size_t 
    size( void ) { return 6; }
    
    std::array<BScalar, 6>&
    data( void ) { return m_data; }
    
    const std::array<BScalar, 6>&
    data( void ) const { return m_data; }
    
    operator const std::array<BScalar, 6>&( void ) const { return  m_data; }
    
    const BSpatialVector
    operator-( void ) const
    {
        return BSpatialVector(-m_data[0], -m_data[1], -m_data[2], 
                              -m_data[3], -m_data[4], -m_data[5]);
    }
  
    const BSpatialVector
    operator/( BScalar s ) const
    {
        return BSpatialVector(m_data[0] / s, m_data[1] / s, m_data[2] / s, 
                              m_data[3] / s, m_data[4] / s, m_data[5] / s);
    }
    
    const BSpatialVector&
    operator/=( BScalar s )
    {
        m_data[0] /= s; m_data[1] /= s; m_data[2] /= s;
        m_data[3] /= s; m_data[4] /= s; m_data[5] /= s;
        return *this;
    }
    
    const BSpatialVector
    operator*( BScalar s ) const
    { 
        return BSpatialVector(m_data[0] * s, m_data[1] * s, m_data[2] * s, 
                              m_data[3] * s, m_data[4] * s, m_data[5] * s); 
    }
    
    const BSpatialVector&
    operator*=( BScalar s )
    {
        m_data[0] *= s; m_data[1] *= s; m_data[2] *= s;
        m_data[3] *= s; m_data[4] *= s; m_data[5] *= s;
        return *this;
    }
    
    const BSpatialVector
    operator+( const BSpatialVector &v ) const
    {
        return BSpatialVector(m_data[0] + v[0], m_data[1] + v[1], m_data[2] + v[2], 
                              m_data[3] + v[3], m_data[4] + v[4], m_data[5] + v[5]);
    }
    
    const BSpatialVector&
    operator+=( const BSpatialVector &v )
    {
        m_data[0] += v[0]; m_data[1] += v[1]; m_data[2] += v[2];
        m_data[3] += v[3]; m_data[4] += v[4]; m_data[5] += v[5];
        return *this;
    }
    
    const BSpatialVector
    operator-( const BSpatialVector &v ) const
    {
        return BSpatialVector(m_data[0] - v[0], m_data[1] - v[1], m_data[2] - v[2], 
                              m_data[3] - v[3], m_data[4] - v[4], m_data[5] - v[5]);
    }
    
    const BSpatialVector&
    operator-=( const BSpatialVector &v )
    {
        m_data[0] -= v[0]; m_data[1] -= v[1]; m_data[2] -= v[2];
        m_data[3] -= v[3]; m_data[4] -= v[4]; m_data[5] -= v[5];
        return *this;
    }

    bool 
    operator==( const BSpatialVector &v ) const { return (m_data == v.m_data); }
    
    bool 
    operator!=( const BSpatialVector &v ) const { return (m_data != v.m_data); }

private:

    std::array<BScalar, 6> m_data;
};

// scalar multiplication
inline const BSpatialVector 
operator*( BScalar s, const BSpatialVector &v ) { return v * s; } 

inline std::ostream&
operator<<( std::ostream &ostr, const BSpatialVector &v )
{
    ostr << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v[3] << ' ' << v[4] << ' ' << v[5] << ' ';
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BSpatialVector &v )
{
    istr >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5];
    return istr;
}



#endif


