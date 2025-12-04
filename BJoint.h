/* BJoint 31/01/2024

 $$$$$$$$$$$$$$$$
 $   BJoint.h   $
 $$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Note: The following comments has been copied from the RBDL code.
 
 \page joint_description Joint Modeling
 
 \section joint_overview Overview
 
 The Rigid Body Dynamics Library supports a multitude of joints:
 revolute, planar, fixed, singularity-free spherical joints and joints
 with multiple degrees of freedom oint Model Calculation Routinein any combinations.
 
 Fixed1 joints do not cause any overhead in RBDL as the bodies that are
 rigidly connected are merged into a single body. For details see \ref
 joint_models_fixed.
 
 Fixed2 joints are not merged into a single body and are added as the bodies that are
 rigidly connected.
 
 Joints with multiple degrees of freedom are emulated by default which
 means that they are split up into multiple single degree of freedom
 joints which results in equivalent models. This has the benefit that it
 simplifies the required algebra and also code branching in RBDL. A
 special case are joints with three degrees of freedom for which specific
 joints are available that should be used for performance reasons
 whenever possible. See \ref joint_three_dof for details.
 
 Joints are defined by their motion subspace. For each degree of freedom
 a one dimensional motion subspace is specified as a Math::SpatialVector.
 This vector follows the following convention: \f[ (r_x, r_y, r_z, t_x,
 t_y, t_z) \f]
 
 To specify a planar joint with three degrees of freedom for which the
 first two are translations in \f$x\f$ and \f$y\f$ direction and the last
 is a rotation around \f$z\f$, the following joint definition can be
 used:
 
 \code 
    Joint planar_joint = Joint (
        Math::SpatialVector (0., 0., 0., 1., 0., 0.),
        Math::SpatialVector (0., 0., 0., 0., 1., 0.),
        Math::SpatialVector (0., 0., 1., 0., 0., 0.)
    );
 \endcode
 
 \note Please note that in the Rigid %Body Dynamics Library all angles
 are specified in radians.
 
 \section joint_models_fixed Fixed1 Joints
 
 Fixed1 joints do not add an additional degree of freedom to the model.
 When adding a body that via a fixed joint (i.e. when the type is
 JointTypeFixed) then the dynamical parameters mass and inertia are
 merged onto its moving parent. By doing so fixed bodies do not add
 computational costs when performing dynamics computations.
 
 To ensure a consistent API for the Kinematics such fixed bodies have a
 different range of ids. Where as the ids start at 1 get incremented for
 each added body, fixed bodies start at Model::fixed_body_discriminator
 which has a default value of std::numeric_limits<unsigned int>::max() /
 2. This means theoretical a maximum of each 2147483646 movable and fixed
 bodies are possible.
 
 To check whether a body is connected by a fixed joint you can use the
 function Model::isFixedBodyId().
 
 \section joint_three_dof 3-DoF Joints
 
 RBDL has highly efficient implementations for the following three degree
 of freedom joints:
 <ul>
     <li>\ref JointTypeTranslationXYZ which first translates along X, then
     Y, and finally Z.</li>
     <li>\ref JointTypeEulerZYX which first rotates around Z, then Y, and
     then X.</li>
     <li>\ref JointTypeEulerXYZ which first rotates around X, then Y, and
     then Z.</li>
     <li>\ref JointTypeEulerYXZ which first rotates around Y, then X, and
     then Z.</li>
     <li>\ref JointTypeEulerZXY which first rotates around Z, then X, and
     then Y.</li>
     <li>\ref JointTypeSpherical which is a singularity free joint that
     uses a Quaternion and the bodies angular velocity (see \ref
     joint_singularities for details).</li>
 </ul>
 
 These joints can be created by providing the joint type as an argument
 to the Joint constructor, e.g.:
 
 \code Joint joint_rot_zyx = Joint ( JointTypeEulerZYX ); \endcode
 
 Using 3-Dof joints is always favourable over using their emulated
 counterparts as they are considerably faster and describe the same
 kinematics and dynamics.
 
 \section joint_floatingbase Floating-Base Joint (a.k.a. Freeflyer Joint)
 
 RBDL has a special joint type for floating-base systems that uses the
 enum JointTypeFloatingBase. The first three DoF are translations along
 X,Y, and Z. For the rotational part it uses a JointTypeSpherical joint.
 It is internally modeled by a JointTypeTranslationXYZ and a
 JointTypeSpherical joint. It is recommended to only use this joint for
 the very first body added to the model.
 
 Positional variables are translations along X, Y, and Z, and for
 rotations it uses Quaternions. To set/get the orientation use
 Model::SetQuaternion () / Model::GetQuaternion() with the body id
 returned when adding the floating base (i.e. the call to
 Model::AddBody() or Model::AppendBody()).
 
 \section joint_singularities Joint Singularities
 
 Singularities in the models arise when a joint has three rotational
 degrees of freedom and the rotations are described by Euler- or
 Cardan-angles. The singularities present in these rotation
 parametrizations (e.g. for ZYX Euler-angles for rotations where a
 +/- 90 degrees rotation around the Y-axis) may cause problems in
 dynamics calculations, such as a rank-deficit joint-space inertia matrix
 or exploding accelerations in the forward dynamics calculations.
 
 For this case RBDL has the special joint type
 RigidBodyDynamics::JointTypeSpherical. When using this joint type the
 model does not suffer from singularities, however this also results in
 a change of interpretation for the values \f$\mathbf{q}, \mathbf{\dot{q}}, \mathbf{\ddot{q}}\f$, and \f$\mathbf{\tau}\f$:
 
 <ul>
     <li> The values in \f$\mathbf{q}\f$ for the joint parameterizes the orientation of a joint using a
     Quaternion \f$q_i\f$ </li>
     <li> The values in \f$\mathbf{\dot{q}}\f$ for the joint describe the angular
     velocity \f$\omega\f$ of the joint in body coordinates</li>
     <li> The values in \f$\mathbf{\ddot{q}}\f$ for the joint describe the angular
     acceleration \f$\dot{\omega}\f$ of the joint in body coordinates</li>
     <li> The values in \f$\mathbf{\tau}\f$ for the joint describe the three couples
     acting on the body in body coordinates that are actuated by the joint.</li>
 </ul>
 
 As a result, the dimension of the vector \f$\mathbf{q}\f$ is higher than
 of the vector of the velocity variables. Additionally, the values in
 \f$\mathbf{\dot{q}}\f$ are \b not the derivative of \f$q\f$ and are therefore
 denoted by \f$\mathbf{\bar{q}}\f$ (and similarly for the joint
 accelerations).
 
 RBDL stores the Quaternions in \f$\mathbf{q}\f$ such that the 4th component of
 the joint is appended to \f$\mathbf{q}\f$. E.g. for a model with the joints:
 TX, Spherical, TY, Spherical, the values of \f$\mathbf{q},\mathbf{\bar{q}},\mathbf{\bar{\bar{q}}},\mathbf{\tau}\f$ are:
 
 
 \f{eqnarray*}
 \mathbf{q} &=& ( q_{tx}, q_{q1,x}, q_{q1,y}, q_{q1,z}, q_{ty}, q_{q2,x}, q_{q2,y}, q_{q2,z}, q_{q1,w}, q_{q2,w})^T \\
 \mathbf{\bar{q}} &=& ( \dot{q}_{tx}, \omega_{1,x}, \omega_{1,y}, \omega_{1,z}, \dot{q}_{ty}, \omega_{2,x}, \omega_{2,y}, \omega_{2,z} )^T \\
 \mathbf{\bar{\bar{q}}} &=& ( \ddot{q}_{tx}, \dot{\omega}_{1,x}, \dot{\omega}_{1,y}, \dot{\omega}_{1,z}, \ddot{q}_{ty}, \dot{\omega}_{2,x}, \dot{\omega}_{2,y}, \dot{\omega}_{2,z} )^T \\
 \mathbf{\tau} &=& ( \tau_{tx}, \tau_{1,x}, \tau_{1,y}, \tau_{1,z}, \tau_{ty}, \tau_{2,x}, \tau_{2,y}, \tau_{2,z} )^T
 \f}
 
 \subsection spherical_integration Numerical Integration of Quaternions
 
 An additional consequence of this is, that special treatment is required
 when numerically integrating the angular velocities. One possibility is
 to interpret the angular velocity as an axis-angle pair scaled by the
 timestep and use it create a quaternion that is applied to the previous
 Quaternion. Another is to compute the quaternion rates from the angular
 velocity. For details see James Diebel "Representing Attitude: Euler
 Angles, Unit Quaternions, and Rotation Vectors", 2006,
 http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.110.5134.
      
*/

#ifndef __BJOINT_H__
#define __BJOINT_H__


#ifndef __BSPATIALVECTOR_H__
#include "BSpatialVector.h"
#endif

#ifndef __BMATRIX63_H__
#include "BMatrix63.h"
#endif

#ifndef __BSPATIALTRANSFORM_H__
#include "BSpatialTransform.h"
#endif



class BJoint
{

public:
      
    enum BJointType 
    {
        UNDEFINED = 0,
        
        // Emulated 2-6 DoF joint.
        // values here are important - do not change
        J1DoF = 1, J2DoF = 2, J3DoF = 3, J4DoF = 4, J5DoF = 5, J6DoF = 6, 
        
        
        Prismatic, // 1 DoF - Sliding joint
        
        Revolute, RevoluteX, RevoluteY, RevoluteZ,  // 1 DoF - Hinge joint
        
        // 3 DoF joint that uses Euler conventions (faster than emulated multi DoF joints).
        EulerZYX,  EulerXYZ,  EulerYXZ,   EulerZXY,  
        
        // Spherical or 'ball and socket' joint - allows arbitrary rotation about a specific point.
        Spherical, // 3 DoF joint using quaternions for joint positional and angular velocity variables.
        
        TranslationXYZ,
        
        // A 6-DoF joint for floating-base systems.  
        // modeled internally by a TranslationXYZ and a Spherical joint. 
        // use this joint for first body added to model (see RBDA, section 4.1, page 66).
        FloatingBase, 
        
        Fixed1,        // Fixed1 joint which causes the inertial properties to be merged with the parent body.
        Fixed2,        // Fixed2 joint which adds a distinct child body to parent body.
        
        Helical,      // A 1-DoF 'screw' joint with both rotational and translational motion.
        
        // Custom,
        MAXJOINT    
    };
    
    BJoint( void )=default;

    BJoint( BJointType joint_type );
    
    // only joint_type BRevolute or BPrismatic; joint_axis is axis of rotation or translation
    BJoint( BJointType joint_type, const BVector3 &joint_axis);
    
    // constructs a 1-6 DoF joint with the given motion subspaces.
    BJoint( const BSpatialVector &axis_0 );
    
    BJoint( const BSpatialVector &axis_0, 
            const BSpatialVector &axis_1 );
    
    BJoint( const BSpatialVector &axis_0,
            const BSpatialVector &axis_1,
            const BSpatialVector &axis_2 );
    
    BJoint( const BSpatialVector &axis_0,
            const BSpatialVector &axis_1,
            const BSpatialVector &axis_2,
            const BSpatialVector &axis_3 );
    
    BJoint( const BSpatialVector &axis_0,
            const BSpatialVector &axis_1,
            const BSpatialVector &axis_2,
            const BSpatialVector &axis_3,
            const BSpatialVector &axis_4 );
    
    BJoint( const BSpatialVector &axis_0,
            const BSpatialVector &axis_1,
            const BSpatialVector &axis_2,
            const BSpatialVector &axis_3,
            const BSpatialVector &axis_4,
            const BSpatialVector &axis_5 );
    
    ~BJoint( void )=default;
    
    void
    clear( void );
    
    void
    setId( BJointId jid ) { m_id = jid; }
    
    BJointId
    getId( void ) const { return m_id; }
    
    int
    DoFCount( void ) const { return (int) m_jointAxes.size(); }
    
    BJointType 
    jtype( void ) const { return m_jtype; }
    
    // calculate [X_lambda, X_J, S_i, v_J, c_J] = jcalc(jtype(i), q, qdot, i) (see RBDA, Section 4.4) 
    void
    jcalc( const std::vector<BScalar> &q, const std::vector<BScalar> &qdot );
    
    // assume this joint is 'closed' i.e. position and velocity, q and qdot are zero 
    void
    jcalc( void )
    {
        const std::vector<BScalar> zero(std::max(m_qidx + 3, m_widx + 1), 0.0); 
        jcalc(zero, zero);
    }

    // the spatial axis i of the joint
    const BSpatialVector&
    axis( int i ) const { return m_jointAxes[i]; }

    // the spatial axes of the joint
    const std::vector<BSpatialVector>&
    axes( void ) const { return m_jointAxes; }
    
    // the transformation from the parent body frame $\lambda(i)$ to body $i$ 
    // ${i}^X_{\lambda(i)} = X_J * X_T(i)$
    const BSpatialTransform& 
    X_lambda( void ) const { return m_X_lambda; }
   
    // the action of the joint - typically a function of the joint's type m_jtype and state $q = (pos,vel,acc,tau)$     
    // $X_J = (E,r)$ (see RBDA, Table 4.1)
    const BSpatialTransform& 
    X_J( void ) const { return m_X_J; } 
    
    // transformation from the parent body $\lambda(i)$ to the origin 
    // of the joint frame in body $i$ (see RBDA, Section 4.2, page 73). 
    // $X_T$ locates the joint's coordinate frame origin in body $i$.
    // Set by BModel::addBody(). 
    const BSpatialTransform& 
    X_T( void ) const { return m_X_T; }
    
    void 
    X_T( const BSpatialTransform &b )  { m_X_T = b; }
    
    
    // a joint's motion subspace $S$ (see RBDA, Table 4.1) - depends on type of joint 
    // when DoF=1 $S$=SpatialVector, DoF=3 $S$=Matrix63, DoF=6 $S$=SpatialMatrix
    const BJointSpace& 
    S( void ) const { return m_S; }


    // joint spatial velocity $v_J$ (see RBDA, Section 4.4)
    const BSpatialVector&
    v_J( void ) const { return m_v_J; } 
    
    // joint spatial acceleration $c_J$ (see RBDA, Section 4.4)
    const BSpatialVector&
    c_J( void ) const { return m_c_J; } 
    
    
    int
    qindex( void ) const { return m_qidx; }
    
    void
    qindex( int q ) { m_qidx = q; }

    void
    windex( int w ) { m_widx = w; }

    BQuat
    getQuat( const std::vector<BScalar> &q ) const;
    
    void 
    setQuat( const BQuat &quat, std::vector<BScalar> &q ) const;

    
    bool 
    operator==( const BJoint &v ) const { return (m_id == v.m_id); }
    
    bool 
    operator!=( const BJoint &v ) const { return (m_id != v.m_id); }
    
    static std::string 
    toString( BJointType jt );
    
private:
    
    void
    setJointSpace( const BJointSpace &m ) { m_S = m; }
    
    inline void
    setJointSpace( const BSpatialVector &v );
    
    void
    setJointSpace( const BMatrix63 &m );
  
    void
    setJointSpace( const BSpatialMatrix &m );
    
    static bool 
    validate_spatial_axis( const BSpatialVector &axis );
    
    void
    setJoint( const std::vector<BSpatialVector> &axes );
    
    BJointId     m_id;

    int          m_qidx;
    int          m_widx; // if joint is spherical - index of quaternion $w$ variable (at end of $q$-vector)
    BJointType   m_jtype; 

  
    // $s^X_p$ = rot(E) xlt(r) - rotation(E) * translate(r) - translate br $r$ then rotate by $E$
    BSpatialTransform m_X_lambda;  // ${i}^X_{\lambda(i)} = X_J X_T(i)$ (see RBDA, example 4.3) 
    BSpatialTransform m_X_J;       // see RBDA Section 4.4 and Table 4.1
    BSpatialTransform m_X_T;       // $X_T$ transform from parent frame to joint position (see RBDA, Section 4.2)
    
    // joint state variables - joint spatial velocity and spatial acceleration (see RBDA, Section 4.4)
    BSpatialVector    m_v_J;     
    BSpatialVector    m_c_J;       
   
    // motion subspace of joint denoted $S$ (RBDA, and Table 4.1)
    BJointSpace       m_S; 

    // spatial axes of the joint; 1 for each degree of freedom
    std::vector<BSpatialVector> m_jointAxes;
    
    // motion subspace constants -- note type here is std::vector not std::array
    static const std::vector<std::vector<BScalar>> m_ZERO_6x3;
    static const std::vector<std::vector<BScalar>> m_ZERO_1x6;
    static const std::vector<std::vector<BScalar>> m_ZERO_6x6;
};

inline std::ostream&
operator<<( std::ostream &ostr, const BJoint &j )
{
    ostr << j.X_lambda() << '\n';
    ostr << j.X_T() << '\n';
    ostr << j.X_J() << '\n';
    ostr << j.v_J() << '\n';
    ostr << j.c_J() << '\n';
    ostr << j.S()   << '\n';
    ostr << j.DoFCount() << '\n';
    
    for (int i = 0; i < j.DoFCount(); ++i)
    {
        ostr << j.axis(i)  << '\n';
    }
    return ostr;
}

#endif




