/*! 

         $$$$$$$$$$$$$$$$$$$$
         $   BTransform.h   $
         $$$$$$$$$$$$$$$$$$$$

 @file BTransform.h
 @ingroup SpatialAlgebra
 date 20/02/2024
 
 by W.B. Yates
 
 ---

 @brief A compact representation of spatial (Plücker) Transforms (see Featherstone, RBDA, Section 2.8, page 20, and Appendix A3, page 245).
 
 Convention: A spatial transform $X(E,r)$ maps base-frame coordinates to body-frame coordinates.
    - $E$ is the body-to-base rotation (its columns are the body axes in base coordinates).
    - $r$ is the position of the body-frame origin, expressed in base coordinates.
    - The inverse is $(E^T, -E^T r)$.

 Points (consistent with spatial methods and BHomogeneous.h):
 
        X.apply(p)           =  E^T (p - r)    base coordinates -> body coordinates
        X.applyTranspose(p)  =  r + E p        body coordinates -> base coordinates
 

 Motion vectors are transformed by $X$ while force vectors are transformed by the dual $X^{*} = X^{-T}$ (see Featherstone, RBDA, Section 2.6, eqn 2.13, page 18).
 
 
 This class employs a compact representation of spatial transformations.
 Instead of using a full 6x6 matrix, the class only stores a 3x3  rotation matrix $E$
 and a 3D translation vector $r$ to represent a spatial transformation. 
 It also provides some efficient matrix operations such as composition
 and transformation of spatial vectors.
 
 The spatial methods of BTransform have been tested against and match numerically the Eigen3 RBDL function:
 
 @code{.cpp}
 SpatialMatrix toMatrix () const {
   Matrix3d _Erx =
     E * Matrix3d (
         0., -r[2], r[1],
         r[2], 0., -r[0],
         -r[1], r[0], 0.
         );
   SpatialMatrix result;
   result.block<3,3>(0,0) = E;
   result.block<3,3>(0,3) = Matrix3d::Zero();
   result.block<3,3>(3,0) = -_Erx;
   result.block<3,3>(3,3) = E;

   return result;
 }
 @endcode

 Relationship to RBDL:
 
 ARB matrices are glm matrices. glm constructors fill column by column, whereas
 Eigen/RBDL initialisers fill row by row. The RBDL code for Xrotz(t) therefore becomes,
 when the argument list is transcribed unchanged, the transposed matrix: arb::rotz(t) is the active rotation,
 while the $E$ stored by RBDL's Xrotz(t) is rotz(-t). Hence $E_ARB = (E_RBDL)^T$, and the
 spatial methods apply $E^T$ where RBDL applies $E$.
 Both ARB and RBDL give the same result when a transform is applied to a spatial vector.
 
 A spatial transform $X$ as a block 6x6 matrix in this implementation has the form:

         |     E      0  |
         | -rx * E    E  |

 where rx = arb::cross(r) (see BTransform::operator BMatrix6()).
 
 The remaining transforms have analytic block definitions (which differ from
 the usual matrix definitions) for inverse and transpose:

         X^{-1}  =  |    E^T      0  |
                    | E^T * rx   E^T |
         
         X^T     =  | E^T    E^T * rx |
                    |  0         E^T  |
         
         X^*  =     | E  -rx * E |
                    | 0      E   |
         
 where  $X^* = X^{-T}$ (see RBDA, page 22). 
 
 When transforming a spatial inertia $I$ by a spatial transform $X$ (see RBDA, eqn 2.66, page 34)
 
            X.apply(I)           = X^* I X^{-1} 
            X.applyTranspose(I)  = X^T I X  
 

 @note BMatrix3 $E$ is a rotation; the transpose of a rotation $E^T$ is also its inverse $E^{-1}$
 @note As $(A * B)^T == B^T * A^T$ and $rx^T == -rx$, then $(-rx * E)^T == E^T * rx$
       where the two negatives cancel out
 @see https://en.wikipedia.org/wiki/Rotation_matrix
 @see https://en.wikipedia.org/wiki/Frame_of_reference
 
*/



#ifndef BTRANSFORM_H
#define BTRANSFORM_H


#ifndef BRBINERTIA_H
#include "BRBInertia.h"
#endif

#ifndef BABINERTIA_H
#include "BABInertia.h"
#endif


class BTransform
{
    
public:
    
    BTransform( void )=default;
    /// performs a translate then rotate i.e. Xrot(glm::radians(theta), axis) * Xtrans(trans); 
    constexpr BTransform( const BMatrix3 &rot, const BVector3 &trans = B_ZERO_3 ): m_E(rot), m_r(trans) {}
    constexpr explicit BTransform( const BVector3 &trans ): m_E(B_IDENTITY_3x3), m_r(trans) {}
    ~BTransform( void )=default;
    
    void
    clear( void ) {  m_E = B_IDENTITY_3x3; m_r = B_ZERO_3; }

    /// convert to a 6x6 motion transform (equivalent to arb::toMotion() in BAdjoint.h)
    operator BMatrix6( void ) const 
    {
        return BMatrix6( m_E, B_ZERO_3x3, arb::cross(-m_r) * m_E, m_E );
    }
    
    void
    set( const BMatrix3 &E, const BVector3 &r = B_ZERO_3 ) 
    {
        m_E = E; m_r = r;
    }
    
    void
    set( const BVector3 &r ) 
    {
        m_E = B_IDENTITY_3x3; m_r = r;
    }
    

    /// rotation preserves coordinate frame origin
    const BMatrix3&
    E( void ) const { return m_E; }

    BMatrix3&
    E( void ) { return m_E; }

    void
    E( const BMatrix3 &m ) { m_E = m; }
    
    /// translation moves coordinate frame origin
    const BVector3&
    r( void ) const { return m_r; }

    BVector3&
    r( void ) { return m_r; }
    
    void
    r( const BVector3 &v ) { m_r = v; }
    
 
    BTransform 
    operator*( const BTransform &rhs ) const 
    {
        return BTransform(rhs.m_E * m_E, rhs.m_r + (rhs.m_E * m_r));
    }
    
    BMatrix6 
    operator*( const BMatrix6 &rhs ) const 
    {
        return BMatrix6(*this) * rhs;
    }
    
    BVector6 
    operator*( const BVector6 &v ) const 
    // return X * v
    {
        const BMatrix3 ET = arb::transpose(m_E);
        const BVector3 v_ang = v.ang();
        const BVector3 v_rxw = v.lin() - arb::cross(m_r, v_ang);
        return BVector6(ET * v_ang, ET * v_rxw);
    }

    BVector6 
    apply( const BVector6 &v ) const { return operator*(v); }

    BVector6 
    applyTranspose( const BVector6 &f ) const
    // return X^T * f 
    {
        const BVector3 ETf = m_E * f.lin();
        const BVector3 aux = m_E * f.ang() + arb::cross(m_r, ETf);
        return BVector6(aux, ETf);
    }


    BMatrix63 
    operator*( const BMatrix63 &u ) const 
    // return X * u
    {
        const BMatrix3 ang = u.top() * m_E;
        const BMatrix3 lin = (u.bot() - (u.top() * arb::cross(m_r))) * m_E;                 
        return BMatrix63(ang, lin);
    }
    
    BMatrix63 
    apply( const BMatrix63 &u ) const { return operator*(u); }
   
    BMatrix63 
    applyTranspose( const BMatrix63 &u ) const
    // return X^T * u
    {
        const BMatrix3 ET = arb::transpose(m_E);
        const BMatrix3 ETf = u.bot() * ET; 
        const BMatrix3 ang = u.top() * ET + ETf * arb::cross(m_r);  
        return BMatrix63(ang, ETf);
    }


    /// base coordinates -> body coordinates: E^T (p - r)
    BVector3
    apply( const BVector3 &p ) const { return arb::transpose(m_E) * (p - m_r); }
    
    /// body coordinates -> base coordinates: r + E p
    BVector3
    applyTranspose( const BVector3 &p ) const { return m_r + (m_E * p); }

 
    BRBInertia
    apply( const BRBInertia &rbi ) const
    // return X^* I X^{-1}
    {
        const BMatrix3 ET  = arb::transpose(m_E);
        const BVector3 h   = ET * (rbi.h() - (rbi.mass() * m_r)); 
        const BMatrix3 rx  = arb::cross(m_r);
        const BMatrix3 aux = (rx * arb::cross(rbi.h())) + ((arb::cross(rbi.h() - rbi.mass() * m_r) * rx));
        const BMatrix3 I   =  ET * (rbi.I() + aux) * m_E;
        return BRBInertia( rbi.mass(), h, I );
    }
    
     
    BRBInertia 
    applyTranspose( const BRBInertia &rbi ) const 
    // return X^T I X 
    {
        const BVector3 Eh = m_E * rbi.h();
        const BVector3 h  = Eh + (rbi.mass() * m_r); 
        BMatrix3 I = m_E * rbi.I() * arb::transpose(m_E);
        BScalar c = arb::dot(Eh + h, m_r);
        BVector3 col0 = m_r[0] * Eh + h[0] * m_r;
        BVector3 col1 = m_r[1] * Eh + h[1] * m_r;
        BVector3 col2 = m_r[2] * Eh + h[2] * m_r;
        col0[0] -= c; col1[1] -= c; col2[2] -= c;
        I -= BMatrix3(col0, col1, col2);
        return BRBInertia( rbi.mass(), h, I );
    }
    
    BABInertia 
    apply( const BABInertia &abi ) const  
    // return X^* I X^{-1} 
    {
        const BMatrix3 rx = arb::cross(m_r);
        const BMatrix3 H = abi.H() - (rx * abi.M());
        const BMatrix3 I = abi.I() - (rx * arb::transpose(abi.H()) ) + (H * rx);
        const BMatrix3 ET = arb::transpose(m_E);
        return BABInertia( ET * abi.M() * m_E,  ET * H * m_E,  ET * I * m_E );
    }
    
    BABInertia 
    applyTranspose( const BABInertia &abi ) const
    // return X^T I X 
    {
        const BMatrix3 ET = arb::transpose(m_E);
        const BMatrix3 M = m_E * abi.M() * ET;
        const BMatrix3 H = m_E * abi.H() * ET;
        const BMatrix3 I_orig = m_E * abi.I() * ET;
        const BMatrix3 rx = arb::cross(m_r);
        const BMatrix3 rxM = rx * M;
        const BMatrix3 rxH = rx * H; 
        const BMatrix3 I = I_orig - rxH + (arb::transpose(H) - rxM) * rx;
        return BABInertia( M, H - arb::transpose(rxM), I );
    }
    
    bool 
    operator==( const BTransform &v ) const { return (m_r == v.m_r) && (m_E == v.m_E); }
    
    bool 
    operator!=( const BTransform &v ) const { return (m_r != v.m_r) || (m_E != v.m_E); }
    
    friend std::istream& 
    operator>>( std::istream &istr, BTransform &m );
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BTransform &m );
    
private:
    
    BMatrix3 m_E; // rotation - note the transpose of a rotation is also its inverse
    BVector3 m_r; // translation - inverse translation is -m_r
};

 
#ifndef GLM_FORCE_INTRINSICS
constexpr BTransform B_IDENTITY_TRANS(B_IDENTITY_3x3, B_ZERO_3);
#else
const BTransform B_IDENTITY_TRANS(B_IDENTITY_3x3, B_ZERO_3);
#endif



namespace arb
{

    inline constexpr BTransform 
    inverse( const BTransform &X )  
    // return  X^{-1}
    { 
        const BMatrix3 ET = arb::transpose(X.E()); 
        return BTransform(ET, -ET * X.r()); 
    }
    
    //
    // rotations, see RBDA, Section 2.8, table, 2.2, page 23
    //
    
    /// @warning angle in radians, axis *must* be normalised
    inline constexpr BTransform 
    Xrot( BScalar theta, const BVector3 &axis ) { return BTransform(rot(theta, axis)); }
    
    /// @warning  angle in radians
    inline constexpr BTransform 
    Xrotx( BScalar theta ) { return BTransform(rotx(theta)); }

    /// @warning  angle in radians
    inline constexpr BTransform 
    Xroty( BScalar theta ) { return BTransform(roty(theta)); }

    /// @warning  angle in radians
    inline constexpr BTransform 
    Xrotz( BScalar theta ) { return BTransform(rotz(theta)); }

    inline constexpr BTransform 
    Xtrans( const BVector3 &r ) { return BTransform( B_IDENTITY_3x3, r ); }
    
}


inline std::ostream&
operator<<( std::ostream &ostr, const BTransform &m )
{
    ostr << m.m_E << ' ' << m.m_r  << ' ';
    return ostr;
}


inline std::istream& 
operator>>( std::istream &istr, BTransform &m )
{
    istr >> m.m_E >> m.m_r;
    return istr;
}


#endif


