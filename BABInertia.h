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
 
 | I   H |
 | H^T M |
 
 where 
     I is the rotational inertia at body frame origin
     H = |h|\times is the generalised momentum matrix
     M is generalised mass i.e m * S_IDENTITY_3x3 
 and
     m is mass
     c is centre of mass
     h = m * c is linear momentum

 ABInertia is only used (at the moment) inside BDynamics::forward(.) for holding 
 interim 'articulated inertias' that are linked together (in kinematic chain/tree of objects). 
 In order to represent the (spatial) inertia of a single object use BRBInertia 

 https://github.com/jrl-umi3218/SpaceVecAlg/blob/master/src/SpaceVecAlg/ABInertia.h
 https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_articulated_body_inertia.html

 
*/


#ifndef __BABINERTIA_H__
#define __BABINERTIA_H__


#ifndef __BSPATIALMATRIX_H__
#include "BSpatialMatrix.h"
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
    
    // inertia I at body frame origin
    constexpr BABInertia( const BMatrix3 &M, const BMatrix3 &H, const BMatrix3 &I ): m_M(M), m_H(H), m_I(I) {}
    
    BABInertia( BScalar m, const BVector3 &com, const BMatrix3 &I_com ): m_M(m), m_H(m * arb::cross(com)), m_I(I_com) 
    {
        m_I += m * arb::crosst(com); // transform I_com to body coordinate frame origin  if com != zero
    }
    
    explicit BABInertia( const BSpatialMatrix &I ) { setInertia(I); }    
    
    // called from SDynamics
    explicit BABInertia( const BSpatialVector &a, const BSpatialVector &b ): m_M(arb::outer(a.lin(), b.lin())), 
                                                                             m_I(arb::outer(a.ang(), b.ang())),
                                                                             m_H(arb::outer(a.ang(), b.lin()))  {}
    
    BABInertia( const BRBInertia &rbi ):  m_M(BMatrix3(rbi.mass())), m_H(arb::cross(rbi.h())), m_I(rbi.inertia()) {}
    
    ~BABInertia( void )=default;
    
    void 
    setInertia( const BSpatialMatrix &I )    
    {
        m_I = I.topLeft(); m_H = I.topRight(); m_M = I.botRight(); 
    }
    
    void
    clear( void ) { m_M = m_H = m_I = B_ZERO_3x3; }
    
    //
    
    // generalized mass 
    const BMatrix3&
    M( void ) const { return m_M; } 
    
    // generalised inertia
    const BMatrix3&
    H( void ) const { return m_H; } 
    
    // rotational inertia at body origin
    const BMatrix3&
    I( void ) const { return m_I; } 
    
    operator BSpatialMatrix( void ) const 
    { 
        return BSpatialMatrix( m_I, m_H, glm::transpose(m_H), m_M );
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
    const BSpatialVector 
    operator*( const BSpatialVector &v ) const
    {
        //const BVector3 ang((m_I * v.ang()) + (glm::transpose(m_H) * v.lin()));
        //const BVector3 lin((m_H * v.ang()) + (m_M * v.lin()) );
        //return BSpatialVector( ang, lin );
        
        return BSpatialVector((m_I[0][0] * v[0]) + (m_I[1][0] * v[1]) + (m_I[2][0] * v[2])  +  (m_H[0][0] * v[3]) + (m_H[0][1] * v[4]) + (m_H[0][2] * v[5]), 
                              (m_I[0][1] * v[0]) + (m_I[1][1] * v[1]) + (m_I[2][1] * v[2])  +  (m_H[1][0] * v[3]) + (m_H[1][1] * v[4]) + (m_H[1][2] * v[5]), 
                              (m_I[0][2] * v[0]) + (m_I[1][2] * v[1]) + (m_I[2][2] * v[2])  +  (m_H[2][0] * v[3]) + (m_H[2][1] * v[4]) + (m_H[2][2] * v[5]), 
                        
                              (m_H[0][0] * v[0]) + (m_H[1][0] * v[1]) + (m_H[2][0] * v[2])  +  (m_M[0][0] * v[3]) + (m_M[1][0] * v[4]) + (m_M[2][0] * v[5]),
                              (m_H[0][1] * v[0]) + (m_H[1][1] * v[1]) + (m_H[2][1] * v[2])  +  (m_M[0][1] * v[3]) + (m_M[1][1] * v[4]) + (m_M[2][1] * v[5]),
                              (m_H[0][2] * v[0]) + (m_H[1][2] * v[1]) + (m_H[2][2] * v[2])  +  (m_M[0][2] * v[3]) + (m_M[1][2] * v[4]) + (m_M[2][2] * v[5]) );
    }
    
    
    const BMatrix63 
    operator*( const BMatrix63 &m ) const  
    {
        BMatrix63 retVal(B_ZERO_6x3);
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
        return retVal;
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

inline std::ostream&
operator<<( std::ostream &ostr, const BABInertia &m )
{
    ostr << m.m_M << '\n' << m.m_H << '\n'  << m.m_I << ' ';
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BABInertia &m )
{
    istr >> m.m_M >> m.m_H >> m.m_I;
    return istr;
}


namespace arb
{

    // you should generally try to avoid taking the inverse of a spatial matrix
    inline const BSpatialMatrix 
    inverse( const BABInertia &abI ) 
    // Schur complement - analytical inverse - https://en.wikipedia.org/wiki/Schur_complement
    {  
        const BMatrix3 Minv = glm::inverse(abI.M());
        const BMatrix3 T = abI.I() - glm::transpose(abI.H()) * Minv * abI.H();
        const BMatrix3 Tinv = glm::inverse(T);
        
        const BMatrix3 topLeft  = Minv + Minv * abI.H() * Tinv * glm::transpose(abI.H()) * Minv;
        const BMatrix3 topRight = -Minv * abI.H() * Tinv;
        const BMatrix3 botLeft  = -Tinv * glm::transpose(abI.H()) * Minv;
        const BMatrix3 botRight = Tinv;
        
        // remove small asymmetries in H and H^T
        // topLeft = 0.5 * (topLeft + glm::transpose(topLeft));
        // botRight = 0.5 * (botRight + glm::transpose(botRight));
        
        return BSpatialMatrix(topLeft, topRight, botLeft, botRight);
    } 

};

#endif


