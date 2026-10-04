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


#ifndef BABINERTIA_H
#define BABINERTIA_H


#ifndef BMATRIX6_H
#include "BMatrix6.h"
#endif

#ifndef BPRODUCTS_H
#include "BProducts.h"
#endif

#ifndef BMATRIX63_H
#include "BMatrix63.h"
#endif

#ifndef BRBINERTIA_H
#include "BRBInertia.h"
#endif


class BABInertia
{
    
public:
    
    BABInertia( void )=default;
    
    // inertia I_o at body frame origin
    constexpr BABInertia( const BMatrix3 &M, const BMatrix3 &H, const BMatrix3 &I_o ): m_M(M), m_H(H), m_I(I_o) {}
    
    constexpr BABInertia( const BInertia &I ): m_M(BMatrix3(I.mass())), m_H(arb::cross(I.h())), m_I(I.I()) {} 
    
    explicit BABInertia( const BMatrix6 &I ) { set(I); }  
    
    // called from BDynamics  (U * Dinv * U^T) for 1-DoF
    explicit BABInertia( const BVector6 &a, const BVector6 &b ): m_M(arb::outer(a.lin(), b.lin())), 
                                                                 m_H(arb::outer(a.ang(), b.lin())),
                                                                 m_I(arb::outer(a.ang(), b.ang()))  {}
    // called from BDynamics
    BABInertia( const BMatrix63 &U, const BMatrix3 &Dinv ) 
    // (U * Dinv * U^T) for 3-DoF
    {
        const BMatrix3 U_top  = U.top();   
        const BMatrix3 U_topT = arb::transpose(U_top);
        const BMatrix3 U_bot  = U.bot(); 
        const BMatrix3 U_botT = arb::transpose(U_bot);
        const BMatrix3 UD_top = Dinv * U_top;
        const BMatrix3 UD_bot = Dinv * U_bot;

        m_M = U_botT * UD_bot;
        m_H = U_botT * UD_top;
        m_I = U_topT * UD_top;
    }
    
    BABInertia( const BRBInertia &I ):  m_M(BMatrix3(I.mass())), m_H(arb::cross(I.h())), m_I(I.I()) {}
    

    ~BABInertia( void )=default;

  
    void
    clear( void ) { m_M = m_H = m_I = B_ZERO_3x3; }
    
    void 
    set( const BInertia &I ) 
    {
        m_M = I.mass(); m_H = arb::cross(I.h()); m_I = I.I();
    }
    
    void 
    set( const BRBInertia &I ) 
    {
        m_M = I.mass(); m_H = arb::cross(I.h()); m_I = I.I();
    }
    
    void 
    set( const BMatrix6 &I ) 
    { 
        m_I = I.topLeft(); m_H = I.topRight(); m_M = I.botRight(); 
    }

    operator BMatrix6( void ) const 
    { 
        return BMatrix6( m_I, m_H, arb::transpose(m_H), m_M );
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
    

    BABInertia
    operator-( void ) const { return BABInertia(-m_M, -m_H, -m_I); }
    
    BABInertia 
    operator-( const BABInertia &rhs ) const
    {
        return BABInertia( m_M - rhs.m_M, m_H - rhs.m_H, m_I - rhs.m_I );
    }  
    
    BABInertia& 
    operator-=( const BABInertia &rhs )
    {
        m_M -= rhs.m_M; m_H -= rhs.m_H; m_I -= rhs.m_I;
        return *this; 
    }
       
    BABInertia 
    operator+( const BABInertia &rhs ) const
    {
        return BABInertia( m_M + rhs.m_M, m_H + rhs.m_H, m_I + rhs.m_I );
    }
    
    BABInertia&
    operator+=( const BABInertia &rhs )
    {
        m_M += rhs.m_M; m_H += rhs.m_H; m_I += rhs.m_I;
        return *this; 
    }
    
    BABInertia
    operator*( BScalar s ) const { return BABInertia( s * m_M, s * m_H, s * m_I ); }
    
    BABInertia&
    operator*=( BScalar s )
    {
        m_M *= s; m_H *= s; m_I *= s;
        return *this; 
    }
    
    // SpaceAlgVec::Operators.h; pass a motion vector returns a force vector
    BVector6 
    operator*( const BVector6 &v ) const
    // return Ia * v
    {
        const BVector3 v_lin = v.lin();
        const BVector3 v_ang = v.ang();
        const BVector3 ang = (m_I * v_ang) + (arb::transpose(m_H) * v_lin);
        const BVector3 lin = (m_H * v_ang) + (m_M * v_lin);
        return BVector6(ang, lin);
    }
    
    BMatrix63 
    operator*( const BMatrix63 &m ) const
    // return Ia * m
    {
        const BMatrix3 top = m.top();
        const BMatrix3 bot = m.bot();
        const BMatrix3 ang = (top * glm::transpose(m_I)) + (bot * m_H);
        const BMatrix3 lin = (top * glm::transpose(m_H)) + (bot * glm::transpose(m_M));
        return BMatrix63(ang, lin);
    }
    
    BABInertia  
    operator+(const BRBInertia &rbi) const
    // return Ia + I
    {
        const BMatrix3 M_ = m_M + BMatrix3(rbi.mass());
        const BMatrix3 H_ = m_H + arb::cross(rbi.h());
        const BMatrix3 I_ = m_I + rbi.I(); 
        return BABInertia(M_, H_, I_);
    }

    BABInertia  
    operator-(const BRBInertia &rbi) const
    // return Ia - I
    {
        const BMatrix3 M_ = m_M - BMatrix3(rbi.mass());
        const BMatrix3 H_ = m_H - arb::cross(rbi.h());
        const BMatrix3 I_ = m_I - rbi.I(); 
        return BABInertia(M_, H_, I_);
    }

    BABInertia& 
    operator+=(const BRBInertia &rbi)
    // return Ia += I
    {
        m_M[0][0] += rbi.mass();
        m_M[1][1] += rbi.mass();
        m_M[2][2] += rbi.mass();
        m_H += arb::cross(rbi.h());
        m_I += rbi.I(); 
        return *this;
    }

    BABInertia& 
    operator-=(const BRBInertia &rbi)
    // return Ia -= I
    {
        m_M[0][0] -= rbi.mass();
        m_M[1][1] -= rbi.mass();
        m_M[2][2] -= rbi.mass();
        m_H -= arb::cross(rbi.h());
        m_I -= rbi.I(); 
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


// scalar multiplication
inline BABInertia 
operator*( BScalar s, const BABInertia &m ) { return m * s; }


#ifndef GLM_FORCE_INTRINSICS
constexpr BABInertia B_ZERO_ABI(B_ZERO_3x3, B_ZERO_3x3, B_ZERO_3x3);
#else
const BABInertia B_ZERO_ABI(B_ZERO_3x3, B_ZERO_3x3, B_ZERO_3x3);
#endif


namespace arb
{
    inline constexpr BMatrix6 
    inverse( const BABInertia &abi ) 
    // Schur complement - analytical inverse - https://en.wikipedia.org/wiki/Schur_complement
    // WARNING: ensure arb::inverses(m) exist i.e.  i.e. arb::isinvertible(m)
    {  
        const BMatrix3 invM = arb::inverse(abi.M());
        const BMatrix3 T = abi.I() - arb::transpose(abi.H()) * invM * abi.H();
        const BMatrix3 invT = arb::inverse(T);
        
        const BMatrix3 topLeft  = invT;
        const BMatrix3 topRight = -invM * abi.H() * invT;
        const BMatrix3 botLeft  = -invT * arb::transpose(abi.H()) * invM;
        const BMatrix3 botRight = invM + invM * abi.H() * invT * arb::transpose(abi.H()) * invM;
 
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


