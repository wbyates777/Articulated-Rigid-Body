/* Articulated Body Spatial Inertia 09/08/2025

 $$$$$$$$$$$$$$$$$$$$
 $   BABInertia.h   $
 $$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Spatial Articulated Body Inertia - compact representation.
 See Featherstone, RBDA, page 245, 247.
 
 Articulated body inertia is a generalisation of spatial inertia 
 and is defined as a 6x6 matrix 
 
 | I_o  H |
 | H^T  M |
 
 where 
     I_o is the rotational inertia at body frame origin
     H is the generalised inertia matrix, initially H = |h|\times, 
     M is generalised mass, initially M = m * B_IDENTITY_3x3 
 and
     m is mass
     com is centre of mass
     h = m * com is linear momentum

 ABInertia is only used (at the moment) inside BDynamics::forward() for holding 
 interim 'articulated inertias' that are linked together in a kinematic chain/tree of bodies. 
 In order to represent the (spatial) inertia of a single body use BRBInertia 

 https://github.com/jrl-umi3218/SpaceVecAlg/blob/master/src/SpaceVecAlg/ABInertia.h
 https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_articulated_body_inertia.html

 
*/


#ifndef __BABINERTIA_H__
#define __BABINERTIA_H__


#ifndef __BMATRIX6_H__
#include "BMatrix6.h"
#endif

#ifndef __BPRODUCTS_H__
#include "BProducts.h"
#endif


#ifndef __BMATRIX63_H__
#include "BMatrix63.h"
#endif

#ifndef __BRBINERTIA_H__
#include "BRBInertia.h"
#endif

class BABInertia
{
    
public:
    
    BABInertia( void )=default;
    
    // inertia I_o at body frame origin
    constexpr BABInertia( const BMatrix3 &M, const BMatrix3 &H, const BMatrix3 &I_o ): m_M(M), m_H(H), m_I(I_o) {}
    
    BABInertia( BScalar m, const BVector3 &com, const BMatrix3 &I_com ): m_M(m), m_H(m * arb::cross(com)), m_I(I_com) 
    {
        m_I += m * arb::crosst(com); // transform I_com to body coordinate frame origin  if com != zero
    }
    
    explicit BABInertia( const BMatrix6 &I ) { setInertia(I); }    
    
    // called from SDynamics
    explicit BABInertia( const BVector6 &a, const BVector6 &b ): m_M(arb::outer(a.lin(), b.lin())), 
                                                                             m_H(arb::outer(a.ang(), b.lin())),
                                                                             m_I(arb::outer(a.ang(), b.ang()))  {}
    
    BABInertia( const BRBInertia &rbi ):  m_M(BMatrix3(rbi.mass())), m_H(arb::cross(rbi.h())), m_I(rbi.inertia()) {}
    
    ~BABInertia( void )=default;

  
    void
    clear( void ) { m_M = m_H = m_I = B_ZERO_3x3; }
    
    void 
    setInertia( const BMatrix6 &I ) 
    { 
        m_I = I.topLeft(); m_H = I.topRight(); m_M = I.botRight(); 
    }

    // generalized mass 
    const BMatrix3&
    M( void ) const { return m_M; } 
    
    // generalised inertia
    const BMatrix3&
    H( void ) const { return m_H; } 
    
    // rotational inertia at body origin
    const BMatrix3&
    I( void ) const { return m_I; } 
    
    operator BMatrix6( void ) const 
    { 
        return BMatrix6( m_I, m_H, arb::transpose(m_H), m_M );
    }
    
    const BABInertia
    operator-( void ) const { return BABInertia(-m_M, -m_H, -m_I); }
    
    const BABInertia 
    operator-( const BABInertia &rhs ) const
    {
        return BABInertia( m_M - rhs.m_M, m_H - rhs.m_H, m_I - rhs.m_I );
    }  
    
    const BABInertia& 
    operator-=( const BABInertia &rhs )
    {
        m_M -= rhs.m_M; m_H -= rhs.m_H; m_I -= rhs.m_I;
        return *this; 
    }
    
    
    const BABInertia 
    operator+( const BABInertia &rhs ) const
    {
        return BABInertia( m_M + rhs.m_M, m_H + rhs.m_H, m_I + rhs.m_I );
    }
    
    const BABInertia&
    operator+=( const BABInertia &rhs )
    {
        m_M += rhs.m_M; m_H += rhs.m_H; m_I += rhs.m_I;
        return *this; 
    }
    
    const BABInertia
    operator*( BScalar s ) const { return BABInertia( s * m_M, s * m_H, s * m_I ); }
    
    const BABInertia&
    operator*=( BScalar s )
    {
        m_M *= s; m_H *= s; m_I *= s;
        return *this; 
    }
    
    // SpaceAlgVec::Operators.h; pass a motion vector returns a force vector
    const BVector6 
    operator*( const BVector6 &v ) const
    {
        //const BVector3 ang((m_I * v.ang()) + (arb::transpose(m_H) * v.lin()));
        //const BVector3 lin((m_H * v.ang()) + (m_M * v.lin()) );
        //return BVector6( ang, lin );
        
        return BVector6((m_I[0][0] * v[0]) + (m_I[1][0] * v[1]) + (m_I[2][0] * v[2])  +  (m_H[0][0] * v[3]) + (m_H[0][1] * v[4]) + (m_H[0][2] * v[5]), 
                              (m_I[0][1] * v[0]) + (m_I[1][1] * v[1]) + (m_I[2][1] * v[2])  +  (m_H[1][0] * v[3]) + (m_H[1][1] * v[4]) + (m_H[1][2] * v[5]), 
                              (m_I[0][2] * v[0]) + (m_I[1][2] * v[1]) + (m_I[2][2] * v[2])  +  (m_H[2][0] * v[3]) + (m_H[2][1] * v[4]) + (m_H[2][2] * v[5]), 
                        
                              (m_H[0][0] * v[0]) + (m_H[1][0] * v[1]) + (m_H[2][0] * v[2])  +  (m_M[0][0] * v[3]) + (m_M[1][0] * v[4]) + (m_M[2][0] * v[5]),
                              (m_H[0][1] * v[0]) + (m_H[1][1] * v[1]) + (m_H[2][1] * v[2])  +  (m_M[0][1] * v[3]) + (m_M[1][1] * v[4]) + (m_M[2][1] * v[5]),
                              (m_H[0][2] * v[0]) + (m_H[1][2] * v[1]) + (m_H[2][2] * v[2])  +  (m_M[0][2] * v[3]) + (m_M[1][2] * v[4]) + (m_M[2][2] * v[5]) );
    }
    
    
    const BMatrix63 
    operator*( const BMatrix63 &m ) const  
    {
        return BMatrix63((m_I[0][0] * m[0][0]) + (m_I[1][0] * m[1][0]) + (m_I[2][0] * m[2][0])  +  (m_H[0][0] * m[3][0]) + (m_H[0][1] * m[4][0]) + (m_H[0][2] * m[5][0]), 
                         (m_I[0][0] * m[0][1]) + (m_I[1][0] * m[1][1]) + (m_I[2][0] * m[2][1])  +  (m_H[0][0] * m[3][1]) + (m_H[0][1] * m[4][1]) + (m_H[0][2] * m[5][1]), 
                         (m_I[0][0] * m[0][2]) + (m_I[1][0] * m[1][2]) + (m_I[2][0] * m[2][2])  +  (m_H[0][0] * m[3][2]) + (m_H[0][1] * m[4][2]) + (m_H[0][2] * m[5][2]), 
    
                         (m_I[0][1] * m[0][0]) + (m_I[1][1] * m[1][0]) + (m_I[2][1] * m[2][0])  +  (m_H[1][0] * m[3][0]) + (m_H[1][1] * m[4][0]) + (m_H[1][2] * m[5][0]), 
                         (m_I[0][1] * m[0][1]) + (m_I[1][1] * m[1][1]) + (m_I[2][1] * m[2][1])  +  (m_H[1][0] * m[3][1]) + (m_H[1][1] * m[4][1]) + (m_H[1][2] * m[5][1]),
                         (m_I[0][1] * m[0][2]) + (m_I[1][1] * m[1][2]) + (m_I[2][1] * m[2][2])  +  (m_H[1][0] * m[3][2]) + (m_H[1][1] * m[4][2]) + (m_H[1][2] * m[5][2]), 

                         (m_I[0][2] * m[0][0]) + (m_I[1][2] * m[1][0]) + (m_I[2][2] * m[2][0])  +  (m_H[2][0] * m[3][0]) + (m_H[2][1] * m[4][0]) + (m_H[2][2] * m[5][0]), 
                         (m_I[0][2] * m[0][1]) + (m_I[1][2] * m[1][1]) + (m_I[2][2] * m[2][1])  +  (m_H[2][0] * m[3][1]) + (m_H[2][1] * m[4][1]) + (m_H[2][2] * m[5][1]),
                         (m_I[0][2] * m[0][2]) + (m_I[1][2] * m[1][2]) + (m_I[2][2] * m[2][2])  +  (m_H[2][0] * m[3][2]) + (m_H[2][1] * m[4][2]) + (m_H[2][2] * m[5][2]), 
                         
                         (m_H[0][0] * m[0][0]) + (m_H[1][0] * m[1][0]) + (m_H[2][0] * m[2][0])  +  (m_M[0][0] * m[3][0]) + (m_M[1][0] * m[4][0]) + (m_M[2][0] * m[5][0]),
                         (m_H[0][0] * m[0][1]) + (m_H[1][0] * m[1][1]) + (m_H[2][0] * m[2][1])  +  (m_M[0][0] * m[3][1]) + (m_M[1][0] * m[4][1]) + (m_M[2][0] * m[5][1]),
                         (m_H[0][0] * m[0][2]) + (m_H[1][0] * m[1][2]) + (m_H[2][0] * m[2][2])  +  (m_M[0][0] * m[3][2]) + (m_M[1][0] * m[4][2]) + (m_M[2][0] * m[5][2]),
                         
                         (m_H[0][1] * m[0][0]) + (m_H[1][1] * m[1][0]) + (m_H[2][1] * m[2][0])  +  (m_M[0][1] * m[3][0]) + (m_M[1][1] * m[4][0]) + (m_M[2][1] * m[5][0]),
                         (m_H[0][1] * m[0][1]) + (m_H[1][1] * m[1][1]) + (m_H[2][1] * m[2][1])  +  (m_M[0][1] * m[3][1]) + (m_M[1][1] * m[4][1]) + (m_M[2][1] * m[5][1]),
                         (m_H[0][1] * m[0][2]) + (m_H[1][1] * m[1][2]) + (m_H[2][1] * m[2][2])  +  (m_M[0][1] * m[3][2]) + (m_M[1][1] * m[4][2]) + (m_M[2][1] * m[5][2]),
                        
                         (m_H[0][2] * m[0][0]) + (m_H[1][2] * m[1][0]) + (m_H[2][2] * m[2][0])  +  (m_M[0][2] * m[3][0]) + (m_M[1][2] * m[4][0]) + (m_M[2][2] * m[5][0]),
                         (m_H[0][2] * m[0][1]) + (m_H[1][2] * m[1][1]) + (m_H[2][2] * m[2][1])  +  (m_M[0][2] * m[3][1]) + (m_M[1][2] * m[4][1]) + (m_M[2][2] * m[5][1]),
                         (m_H[0][2] * m[0][2]) + (m_H[1][2] * m[1][2]) + (m_H[2][2] * m[2][2])  +  (m_M[0][2] * m[3][2]) + (m_M[1][2] * m[4][2]) + (m_M[2][2] * m[5][2]));
    
        /* BMatrix63 retVal(B_ZERO_6x3);
        for (int i = 0; i < 3; ++i)   
        {         
            for (int j = 0; j < 3; ++j)   
            {     
                for (int k = 0; k < 3; ++k)
                {
                    retVal[i][j]   += m_I[k][i] * m[k][j] + m_H[i][k] * m[k+3][j];       
                    retVal[i+3][j] += m_H[k][i] * m[k][j] + m_M[k][i] * m[k+3][j];      
                }
            }
        }
        return retVal; */
    }
    
    const BABInertia  
    operator+(const BRBInertia &rbi) const
    // returns Ia + I
    {
        const BMatrix3 M = m_M + BMatrix3(rbi.mass());
        const BMatrix3 H = m_H + arb::cross(rbi.h());
        const BMatrix3 I = m_I + rbi.inertia(); 
        return BABInertia(M, H, I);
    }

    const BABInertia  
    operator-(const BRBInertia &rbi) const
    // returns Ia - I
    {
        const BMatrix3 M = m_M - BMatrix3(rbi.mass());
        const BMatrix3 H = m_H - arb::cross(rbi.h());
        const BMatrix3 I = m_I - rbi.inertia(); 
        return BABInertia(M, H, I);
    }

    const BABInertia& 
    operator+=(const BRBInertia &rbi)
    // returns Ia += I
    {
        m_M += BMatrix3(rbi.mass());
        m_H += arb::cross(rbi.h());
        m_I += rbi.inertia(); 
        return *this;
    }

    const BABInertia& 
    operator-=(const BRBInertia &rbi)
    // returns Ia -= I
    {
        m_M -= BMatrix3(rbi.mass());
        m_H -= arb::cross(rbi.h());
        m_I -= rbi.inertia(); 
        return *this;
    }

    bool 
    operator==( const BABInertia &v ) const 
    { 
        return (m_M == v.m_M) && (m_H == v.m_H) && (m_I == v.m_I);
    }
    
    bool 
    operator!=( const BABInertia &v ) const 
    { 
        return (m_M != v.m_M) || (m_H != v.m_H) || (m_I != v.m_I);
    }
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BABInertia &m );
    
    friend std::istream& 
    operator>>( std::istream &istr, BABInertia &m );
    
private:
    
    BMatrix3 m_M; // mass matrix, initially (m * B_IDENTITY_3x3)
    BMatrix3 m_H; // generalised inertia coupling 
    BMatrix3 m_I; // rotational inertia at zero (see RBInertia)
    
};

#ifndef GLM_FORCE_INTRINSICS
constexpr BABInertia B_ZERO_ABI(B_ZERO_3x3, B_ZERO_3x3, B_ZERO_3x3);
#else
const BABInertia B_ZERO_ABI(B_ZERO_3x3, B_ZERO_3x3, B_ZERO_3x3);
#endif

// scalar multiplication
inline const BABInertia 
operator*( BScalar s, const BABInertia &m ) { return m * s; }


namespace arb
{

    // you should generally try to avoid taking the inverse of a spatial matrix
    inline const BMatrix6 
    inverse( const BABInertia &abi ) 
    // Schur complement - analytical inverse - https://en.wikipedia.org/wiki/Schur_complement
    {  
        const BMatrix3 invM = arb::inverse(abi.M());
        const BMatrix3 T = abi.I() - arb::transpose(abi.H()) * invM * abi.H();
        const BMatrix3 invT = arb::inverse(T);
        
        const BMatrix3 topLeft  = invM + invM * abi.H() * invT * arb::transpose(abi.H()) * invM;
        const BMatrix3 topRight = -invM * abi.H() * invT;
        const BMatrix3 botLeft  = -invT * arb::transpose(abi.H()) * invM;
        const BMatrix3 botRight = invT;
     
        return BMatrix6(topLeft, topRight, botLeft, botRight);
    } 

}


inline std::ostream&
operator<<( std::ostream &ostr, const BABInertia &m )
{
    ostr << m.m_M << '\n' << m.m_H << '\n'  << m.m_I << '\n';
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BABInertia &m )
{
    istr >> m.m_M >> m.m_H >> m.m_I;
    return istr;
}


#endif


