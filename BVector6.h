/* BVector6 31/01/2024

 $$$$$$$$$$$$$$$$$$
 $   BVector6.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Spatial Vector (see Featherstone, RBDA).
 
 Represents a 6D spatial vector, Could be a motion vector $m \in M^6$ 
 or a force vector $f \in F^6$.
 
 By convention, spatial velocity vectors are called twists, 
 and spatial force vectors are called wrenches.
 
 It should be noted that we employ an (ang, lin) represenation, following Featherstone and RBDL.
 
*/


#ifndef __BVECTOR6_H__
#define __BVECTOR6_H__


#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif


class BVector6
{

public:
    
    BVector6( void )=default;
    
    constexpr BVector6( const std::array<BScalar, 6> &d ): m_data(d) {}
    constexpr explicit BVector6( BScalar s ): m_data{s, s, s, s, s, s} {}
    constexpr explicit BVector6( BScalar s0, BScalar s1 ): m_data{s0, s0, s0, s1, s1, s1} {}
    explicit BVector6( const std::vector<BScalar> &d ) { assert(d.size() == 6); set(d); }
    explicit BVector6( const std::vector<std::vector<BScalar>> &d ) { assert(d.size() == 1 && d[0].size() == 6); set(d[0]); }
    constexpr BVector6( BScalar s0, BScalar s1, BScalar s2, BScalar s3, BScalar s4, BScalar s5 ): m_data{s0, s1, s2, s3, s4, s5} {}
    BVector6( const BVector3 &h, const BVector3 &t ): m_data{h[0], h[1], h[2], t[0], t[1], t[2]} {}
    BVector6( const BVector3 &h, BScalar s3, BScalar s4, BScalar s5 ): m_data{h[0], h[1], h[2], s3, s4, s5} {}
    BVector6( const BScalar s0, BScalar s1, BScalar s2, const BVector3 &t ): m_data{s0, s1, s2, t[0], t[1], t[2]} {}
    
    ~BVector6( void )=default;
    
    void
    set( BScalar s ) { m_data = { s, s, s, s, s, s }; }
    
    void
    set( BScalar s0, BScalar s1 ) { m_data = { s0, s0, s0, s1, s1, s1 }; }
    
    void 
    set( BScalar s0, BScalar s1, BScalar s2, BScalar s3, BScalar s4, BScalar s5 ) 
    { 
        m_data = { s0, s1, s2, s3, s4, s5 };
    }
    
    void 
    set( const std::vector<BScalar> &d ) 
    { 
        assert(d.size() == 6);
        m_data = { d[0], d[1], d[2], d[3], d[4], d[5] };
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
    ang( BScalar s0, BScalar s1, BScalar s2 ) { m_data[0] = s0; m_data[1] = s1; m_data[2] = s2; }
    
    void
    ang( const BVector3 &v ) { m_data[0] = v[0]; m_data[1] = v[1]; m_data[2] = v[2]; }

    BVector3
    ang( void ) const { return BVector3(m_data[0], m_data[1], m_data[2]); }

    // linear
    void
    lin( BScalar s ) { m_data[3] = s; m_data[4] = s; m_data[5] = s; }

    void
    lin( BScalar s3, BScalar s4, BScalar s5 ) { m_data[3] = s3; m_data[4] = s4; m_data[5] = s5; }
    
    void
    lin( const BVector3 &v ) { m_data[3] = v[0]; m_data[4] = v[1]; m_data[5] = v[2]; }
    
    BVector3
    lin( void ) const { return BVector3(m_data[3], m_data[4], m_data[5]); }
    
    BScalar&
    operator[]( int i ) { return m_data[i]; }
    
    BScalar
    operator[]( int i ) const { return m_data[i]; }

    static size_t 
    size( void ) { return 6; }
    
    std::array<BScalar, 6>&
    data( void ) { return m_data; }
    
    const std::array<BScalar, 6>&
    data( void ) const { return m_data; }
    
    operator const std::array<BScalar, 6>&( void ) const { return  m_data; }
    
    BVector6
    operator-( void ) const
    {
        return BVector6(-m_data[0], -m_data[1], -m_data[2], 
                        -m_data[3], -m_data[4], -m_data[5]);
    }
  
    BVector6
    operator/( BScalar s ) const
    {
        return BVector6(m_data[0] / s, m_data[1] / s, m_data[2] / s, 
                        m_data[3] / s, m_data[4] / s, m_data[5] / s);
    }
    
    BVector6&
    operator/=( BScalar s )
    {
        m_data[0] /= s; m_data[1] /= s; m_data[2] /= s;
        m_data[3] /= s; m_data[4] /= s; m_data[5] /= s;
        return *this;
    }
    
    BVector6
    operator*( BScalar s ) const
    { 
        return BVector6(m_data[0] * s, m_data[1] * s, m_data[2] * s, 
                        m_data[3] * s, m_data[4] * s, m_data[5] * s); 
    }
    
    BVector6&
    operator*=( BScalar s )
    {
        m_data[0] *= s; m_data[1] *= s; m_data[2] *= s;
        m_data[3] *= s; m_data[4] *= s; m_data[5] *= s;
        return *this;
    }
    
    BVector6
    operator+( const BVector6 &v ) const
    {
        return BVector6(m_data[0] + v[0], m_data[1] + v[1], m_data[2] + v[2], 
                        m_data[3] + v[3], m_data[4] + v[4], m_data[5] + v[5]);
    }
    
    BVector6&
    operator+=( const BVector6 &v )
    {
        m_data[0] += v[0]; m_data[1] += v[1]; m_data[2] += v[2];
        m_data[3] += v[3]; m_data[4] += v[4]; m_data[5] += v[5];
        return *this;
    }
    
    BVector6
    operator-( const BVector6 &v ) const
    {
        return BVector6(m_data[0] - v[0], m_data[1] - v[1], m_data[2] - v[2], 
                        m_data[3] - v[3], m_data[4] - v[4], m_data[5] - v[5]);
    }
    
    BVector6&
    operator-=( const BVector6 &v )
    {
        m_data[0] -= v[0]; m_data[1] -= v[1]; m_data[2] -= v[2];
        m_data[3] -= v[3]; m_data[4] -= v[4]; m_data[5] -= v[5];
        return *this;
    }

    bool 
    operator==( const BVector6 &v ) const { return (m_data == v.m_data); }
    
    bool 
    operator!=( const BVector6 &v ) const { return (m_data != v.m_data); }

private:

    std::array<BScalar, 6> m_data;
};

// scalar multiplication
inline BVector6 
operator*( BScalar s, const BVector6 &v ) { return v * s; } 


namespace arb 
{

    inline bool 
    nearZero( const BVector6 &v )  {  return (nearZero(v.ang()) && nearZero(v.lin())); }

    inline bool 
    isnan( const BVector6 &v ) { return (isnan(v.lin()) || isnan(v.ang())); }
    
    inline constexpr BVector6 
    min(const BVector6 &v1, const BVector6 &v2) 
    { 
        return BVector6(std::min(v1[0],v2[0]), std::min(v1[1],v2[1]), std::min(v1[2],v2[2]), 
                        std::min(v1[3],v2[3]), std::min(v1[4],v2[4]), std::min(v1[5],v2[5]));
    }
    
    inline constexpr BVector6 
    max(const BVector6 &v1, const BVector6 &v2)
    { 
        return BVector6(std::max(v1[0],v2[0]), std::max(v1[1],v2[1]), std::max(v1[2],v2[2]), 
                        std::max(v1[3],v2[3]), std::max(v1[4],v2[4]), std::max(v1[5],v2[5]));
    }
    
    inline constexpr BVector6 
    clamp(const BVector6 &v, const BVector6 &min, const BVector6 &max) 
    { 
        return BVector6(std::clamp(v[0], min[0], max[0]), std::clamp(v[1], min[1], max[1]), std::clamp(v[2], min[2], max[2]), 
                        std::clamp(v[3], min[3], max[3]), std::clamp(v[4], min[4], max[4]), std::clamp(v[5], min[5], max[5]));
    }
}

inline std::ostream&
operator<<( std::ostream &ostr, const BVector6 &v )
{
    ostr << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v[3] << ' ' << v[4] << ' ' << v[5] << ' ';
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BVector6 &v )
{
    istr >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5];
    return istr;
}



#endif


