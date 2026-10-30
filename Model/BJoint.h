/*!
 @file BJoint.h
 @date 31/01/2024

 $$$$$$$$$$$$$$$$
 $   BJoint.h   $
 $$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

  
 The ARB library (following RBDL) supports a multitude of joints such as
 revolute, planar, fixed, singularity-free spherical joints and joints
 with multiple degrees of freedom.
 
 Fixed1 joints do not cause any overhead as the bodies that are
 rigidly connected are merged into a moving parent body. By doing so Fixed1 joints 
 do not add computational costs when performing dynamics computations.
 
 Fixed2 joints are not merged into a single body and are added as distinct 
 bodies (with mass) that are fixed, that is, rigidly connected.
 
      
*/

#ifndef BJOINT_H
#define BJOINT_H

#ifndef BVECTOR6_H
#include "BVector6.h"
#endif

#ifndef BMATRIX63_H
#include "BMatrix63.h"
#endif

#ifndef BTRANSFORM_H
#include "BTransform.h"
#endif

/// auxiliary parameters not used by BModel or BDynamics
#ifndef URDFTYPES_H
#include "URDFTypes.h"  /// @see BJointParams
#endif


class BJoint
{

public:
      
    enum JType 
    {
        UNDEFINED = 0,
        
        // emulated 2-6 DoF joint.
        // values here are important - do not change
        J1DoF = 1, J2DoF = 2, J3DoF = 3, J4DoF = 4, J5DoF = 5, J6DoF = 6, 
        
        Prismatic, // 1-DoF - sliding joint

        Revolute, RevoluteX, RevoluteY, RevoluteZ,  // 1-DoF - hinge joint
        
        // 3-DoF joint that uses Euler conventions (faster than emulated multi DoF joints).
        EulerZYX,  EulerXYZ,  EulerYXZ,   EulerZXY,  
        
        // spherical or 'ball and socket' joint - allows arbitrary rotation about a specific point.
        // 3-DoF joint using quaternions for joint positional and angular velocity variables.
        Spherical, 
        
        TransXYZ, // TranslationXYZ - linear translation (XYZ) 3-DoF joint
        
        // 6-DoF joint for floating-base systems i.e a drone, or game vehicle  
        // modeled internally by a TransXYZ and a Spherical joint. 
        // use this joint for first body added to model (see RBDA, section 4.1, page 66).
        FloatBase, 
        
        Fixed1,        // Fixed1 joint which causes the inertial properties to be merged with the parent body.
        Fixed2,        // Fixed2 joint which adds a distinct child body (with inertia etc) to parent body.
        
        Helical,       // A 1-DoF 'screw' joint with both rotational and translational motion.
        
        Planar,        // Planar joint (3-DOF: x,y translation and z rotation)

        MAXJOINT    
    };
    
    BJoint( void )=default;

    BJoint( JType jtype );
    
    /// only joint_type BRevolute or BPrismatic; joint_axis is axis of rotation or translation
    BJoint( JType jtype, const BVector3 &jaxis);
    
    /// constructs a 1-6 DoF joint with the given motion subspaces.
    BJoint( const BVector6 &axis_0 );
    
    BJoint( const BVector6 &axis_0, 
            const BVector6 &axis_1 );
    
    BJoint( const BVector6 &axis_0,
            const BVector6 &axis_1,
            const BVector6 &axis_2 );
    
    BJoint( const BVector6 &axis_0,
            const BVector6 &axis_1,
            const BVector6 &axis_2,
            const BVector6 &axis_3 );
    
    BJoint( const BVector6 &axis_0,
            const BVector6 &axis_1,
            const BVector6 &axis_2,
            const BVector6 &axis_3,
            const BVector6 &axis_4 );
    
    BJoint( const BVector6 &axis_0,
            const BVector6 &axis_1,
            const BVector6 &axis_2,
            const BVector6 &axis_3,
            const BVector6 &axis_4,
            const BVector6 &axis_5 );
    
    ~BJoint( void )=default;
    
    void
    clear( void );
    
    void
    setId( BJointId jid ) { m_id = jid; }
    
    BJointId
    getId( void ) const { return m_id; }
    
    int
    DoFCount( void ) const { return (int) m_axis.size(); }
    
    JType 
    jtype( void ) const { return m_jtype; }
    
    /// @brief Computes all variables for a joint model
    ///
    ///  By appropriate modification of this function all types of joints can be
    ///  modeled. Calculate $\[X_lambda, X_J, S_i, v_J, c_J\]$. See RBDA Section 4.4 for details.
    ///
    /// @param[in] q        joint state (position) variables
    /// @param[in] qdot     joint velocity variables
    void
    jcalc( const std::vector<BScalar> &q, const std::vector<BScalar> &qdot );
    
    /// assume this joint is 'closed' i.e. position and velocity, q and qdot are zero 
    void
    jcalc( void )
    {
        const std::vector<BScalar> zero(std::max(m_qidx + 6, m_widx + 1), 0.0); 
        jcalc(zero, zero);
    }

    /// i-th spatial axis of the joint
    const BVector6&
    axis( int i ) const { return m_axis[i]; }

    /// spatial axes of the joint
    const std::vector<BVector6>&
    axis( void ) const { return m_axis; }
    
    /// transform from the parent body frame $\lambda(i)$ to joint $i$ 
    /// ${i}^X_{\lambda(i)} = X_J * X_T(i)$
    const BTransform& 
    X_lambda( void ) const { return m_X_lambda; }
   
    /// the action of the joint - typically a function of the joint's type m_jtype and state $q$  (see RBDA, Table 4.1) 
    const BTransform& 
    X_J( void ) const { return m_X_J; } 
    
    /// transform from the parent body $\lambda(i)$ to the origin 
    /// of the joint frame in body $i$ (see RBDA, Section 4.2, page 73). 
    /// $X_T$ locates the joint's coordinate frame origin in body $i$ 
    /// (position and orientation). Set by BModel::addBody(). 
    const BTransform& 
    X_T( void ) const { return m_X_T; }
    
    BTransform& 
    X_T( void ) { return m_X_T; }
    
    void 
    X_T( const BTransform &b )  { m_X_T = b; }
    
    
    /// a joint's motion subspace $S$ (see RBDA, Table 4.1) - depends on type of joint 
    /// when DoF=1 S=SpatialVector, DoF=3 S=Matrix63, DoF=6 S=SpatialMatrix
    const BMotionSpace& 
    S( void ) const { return m_S; }


    /// joint spatial velocity $v_J$ (see RBDA, Section 4.4)
    const BVector6&
    v_J( void ) const { return m_v_J; } 
    
    /// joint spatial acceleration $c_J$ (see RBDA, Section 4.4)
    const BVector6&
    c_J( void ) const { return m_c_J; } 
    

    void
    qindex( int q ) { m_qidx = q; }
    
    int
    qindex( void ) const { return m_qidx; }

    void
    windex( int w ) { m_widx = w; }
    
    int
    windex( void ) const { return m_widx; }
    

    BQuat
    getQuat( const std::vector<BScalar> &q ) const;
    
    void 
    setQuat( const BQuat &quat, std::vector<BScalar> &q ) const;

    
    BJointParams& 
    params( void ) { return m_params; }
    
    const BJointParams& 
    params( void ) const { return m_params; }
    
    
    bool 
    operator==( const BJoint &v ) const { return (m_id == v.m_id); }
    
    bool 
    operator!=( const BJoint &v ) const { return (m_id != v.m_id); }

    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BJoint &j );
    
    friend std::istream& 
    operator>>( std::istream &istr, BJoint &j );
    
    
    static std::string 
    toString( JType jt );
    
    
private:
    
    BJoint( const std::vector<BVector6> &axes );
    
    void
    setMotionSpace( const BMotionSpace &m ) { m_S = m; }
    
    void
    setMotionSpace( const BVector6 &v )  { m_S = {v[0], v[1], v[2], v[3], v[4], v[5]};  }
    
    void
    setMotionSpace( const BMatrix63 &m );
  
    void
    setMotionSpace( const BMatrix6 &m );

    static bool 
    validate_spatial_axis( const BVector6 &axis );
    
    BJointId     m_id;

    int          m_qidx;
    int          m_widx;    /// if joint is spherical - index of quaternion $w$ variable (at end of $q$-vector)
    JType        m_jtype; 

  
    BTransform m_X_lambda;  /// ${i}^X_{\lambda(i)} = X_J X_T(i)$ (see RBDA, example 4.3) 
    BTransform m_X_J;       /// action of the joint (see RBDA Section 4.4 and Table 4.1)
    BTransform m_X_T;       /// transform from parent to joint frame origin - the joint's position (see RBDA, Section 4.2)
    
    /// joint state variables - spatial velocity and spatial acceleration (see RBDA, Section 4.4)
    BVector6   m_v_J;     
    BVector6   m_c_J;       

    BJointParams m_params; /// auxiliary URDF parameters not used by BModel or BDynamics
   
    BMotionSpace m_S;             /// motion subspace of joint denoted $S$ (RBDA, and Table 4.1)    
    std::vector<BVector6> m_axis; /// spatial axes of the joint; 1 for each degree of freedom
};



inline std::ostream&
operator<<( std::ostream &ostr, const BJoint &j )
{
    ostr << j.m_id << ' ';
    ostr << j.m_qidx << ' ';
    ostr << j.m_widx << ' ';
    ostr << int(j.m_jtype) << '\n';
    
    ostr << j.m_X_lambda << '\n';
    ostr << j.m_X_J << '\n';
    ostr << j.m_X_T << '\n';
    
    ostr << j.m_v_J << '\n';
    ostr << j.m_c_J << '\n';
   
    ostr << j.m_S  << '\n';
    ostr << j.m_params  << '\n';
    

    ostr << j.m_axis << '\n';
    
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BJoint &j )
{
    istr >> j.m_id;
    istr >> j.m_qidx;
    istr >> j.m_widx;
    
    int jt;
    istr >> jt;
    j.m_jtype = BJoint::JType(jt);
    
    istr >> j.m_X_lambda;
    istr >> j.m_X_J;
    istr >> j.m_X_T;
    
    istr >> j.m_v_J;
    istr >> j.m_c_J;
    
    istr >> j.m_S;
    istr >> j.m_params;
    
    istr >> j.m_axis;
    
    return istr;
}


#endif




