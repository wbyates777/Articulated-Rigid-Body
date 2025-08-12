/* BModel 31/01/2024

 $$$$$$$$$$$$$$$$
 $   BModel.h   $
 $$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 Note: The following comments has been copied from the RBDL code.
 
 \brief Contains all information about the rigid body model

 This class contains all information required to perform the forward
 dynamics calculation. The variables in this class are also used for
 storage of temporary values. It is designed for use of the Articulated
 Rigid Body Algorithm (which is implemented in ForwardDynamics()) and
 follows the numbering as described in Featherstones book.

 Please note that body 0 is the root body and the moving bodies start at
 index 1. This numbering scheme is very beneficial in terms of
 readability of the code as the resulting code is very similar to the
 pseudo-code in the RBDA book. The generalized variables q, qdot, qddot
 and tau however start at 0 such that the first entry (e.g. q[0]) always
 specifies the value for the first moving body.

 \note To query the number of degrees of freedom use BModel::dof_count().


*/


#ifndef __BMODEL_H__
#define __BMODEL_H__

#ifndef __BFIXEDBODY_H__
#include "BFixedBody.h" 
#endif

#ifndef __BJOINT_H__
#include "BJoint.h"
#endif

#ifndef __BABINERTIA_H__
#include "BABInertia.h"
#endif


class BModel
{

public:


    BModel( int expected_dof = 1 );
    ~BModel( void )=default;

    void 
    init( void );
    
    void 
    clear( void );

    BJoint&
    joint( BJointId jid ) { return m_joint[jid]; }
    
    const BJoint&
    joint( BJointId jid ) const { return m_joint[jid]; } 
    
    size_t 
    joints( void ) const { return m_joint.size(); } // $N_J$
    
    BBody&
    body( BBodyId bid ) { return m_body[bid]; }

    const BBody&
    body( BBodyId bid ) const { return m_body[bid]; }
    
    BFixedBody&
    fixedBody( BBodyId bid ) { return m_fixed[bid - m_fbd]; }

    const BFixedBody&
    fixedBody( BBodyId bid ) const { return m_fixed[bid - m_fbd]; }
   
    size_t
    bodies( void ) const { return m_body.size(); }     // $N_B$
        
    

    /** \brief Connects a given body to the model
     *
     * The question "where is the new body to be added?" is split up in two
     * parts: first the parent (or successor) body to which it is added and
     * second the transformation to the origin of the joint that connects the
     * two bodies. With these two informations one specifies the relative
     * positions of the bodies when the joint is in neutral position.gk
     *
     * The last question "by what kind of joint should the body be added?" is
     * again simply contained in the Joint class.
     *
     * \param parent_id   id of the parent body
     * \param joint_frame the transformation from the parent frame to the origin
     *                    of the joint frame (used to set X_T in BJoint)
     *
     */
    BBodyId 
    addBody( BBodyId parent_id,
             const BSpatialTransform &joint_frame,
             const BJoint &joint, // used to set X_T in joint -- parent to this
             const BBody &body, // will set bodyId
             const std::string &body_name = "" );    


    
    // set a bodies parameters -- updates whole model
    void 
    setMass( BBodyId bid, BScalar mass );
    
    void 
    setInertia( BBodyId bid, const BMatrix3 &inertia );
    
    void 
    setCom( BBodyId bid, const BVector3 &com );
    
    void 
    setParameters( BBodyId bid, BScalar mass, const BMatrix3 &inertia, const BVector3 &com );
    //
    
    // set the joint frame transformtion, i.e. X_T, the second argument to  BModel::addBody()
    void 
    setJointFrame( BBodyId bid, const BSpatialTransform &transform );
    
    // joint frame transformtion, i.e. X_T, the second argument to BModel::addBody()
    BSpatialTransform 
    getJointFrame( BBodyId bid ) const;
    
    // return base coordinates of body_pos where body_pos is expressed in body $bid$ coordinates 
    BVector3
    toBasePos( BBodyId bid, const BVector3 &body_pos = B_ZERO_3 ) const;
    
    // return body $bid$ coordinates of base_pos where base_pos is expressed in base coordinates 
    BVector3 
    toBodyPos( BBodyId bid,  const BVector3 &base_pos ) const;

    // return orientation of body $bid$
    BMatrix3 
    orient( BBodyId bid ) const;
    //

    // total mass of body $bid$ and all children bodies (subtree)
    BScalar
    mass( BBodyId bid ) const; 
    
    // centre of mass of body $bid$ and all children bodies  (subtree)
    BVector3 
    com( BBodyId bid ) const; 
    
    // inertia of body $bid$ and all children bodies  (subtree)
    BRBInertia 
    inertia( BBodyId bid ) const;
    
    // size of the $\mathbf{q}$-vector.
    int  
    qsize( void ) const { return m_q_size; }
    
    // size of the ($\mathbf{\dot{q}}, \mathbf{\ddot{q}}$, and $\mathbf{\tau}$-vector.
    int  
    qdotsize( void ) const { return m_qdot_size; }
    
    // total of degrees of freedom for these articulated bodies
    int  
    dofCount( void ) const { return m_dof_count; }
    
    
    const BVector3& 
    gravity( void ) const { return m_gravity; }

    void 
    gravity( const BVector3& g ) { m_gravity = g; }
    
    //
    // dynamic variables
    //

    // IA are the articulated-body inertia $I_i^A$ for a body $Bi$ in $A_i$.
    const BABInertia&      // const BSpatialMatrix&
    IA( int i ) const { return m_IA[i]; }
    
    BABInertia&      //  BSpatialMatrix&
    IA( int i ) { return m_IA[i]; }
    
    // pA are the bias forces $p_i^A$ for a body $Bi$ in $A_i$.
    const BSpatialVector& 
    pA( int i ) const { return m_pA[i]; }
    
    BSpatialVector& 
    pA( int i ) { return m_pA[i]; }

    
    bool 
    isBodyId( BBodyId bid ) const;
    
    bool 
    isFixedBodyId( BBodyId bid ) const;
    
    BBodyId
    parentId( BBodyId bid )  const { return m_lambda[bid]; }
    
    BBodyId 
    getBodyId( const std::string &body_name ) const;

    std::string 
    getBodyName( BBodyId bid ) const;

    // returns the id of the actual non-virtual parent body.
    BBodyId 
    getParentBodyId( BBodyId bid ) const;

    
   // friend std::ostream&
  //  operator<<( std::ostream &ostr, const BModel &m );
    
private:

    BBodyId 
    addMultiDofJoint( BBodyId parent_id,
                      const BSpatialTransform &joint_frame,
                      const BJoint &joint,
                      const BBody &body,
                      const std::string &body_name = ""); 
    
    BBodyId
    addFixedJoint( BBodyId parent_id,
                   const BSpatialTransform &joint_frame,
                   const BJoint &joint,
                   const BBody &body,
                   const std::string &body_name = "" );   

    
    BBodyId
    addFloatingBaseJoint( BBodyId parent_id,
                          const BSpatialTransform &joint_frame,
                          const BJoint &joint,
                          const BBody &body,
                          const std::string &body_name = "" );
    
    void
    calcDoF( void );
    
    void
    addName( BBodyId bid, const std::string &body_name );
    
    // number of degrees of freedoms of the model
    // general (joint) state (q), velocity (qdot), acceleration (qddot)
    int m_dof_count; 
    int m_q_size; 
    int m_qdot_size;
    
    BBodyId m_fbd;                    // fixed_body_discriminator

    BVector3 m_gravity;

    // the id of the parent body
    std::vector<BBodyId> m_lambda;
    
    std::vector<BJoint>     m_joint;
    std::vector<BBody>      m_body;
    std::vector<BFixedBody> m_fixed;  //  bodies attached via a fixed joint

    std::vector<BABInertia> m_IA; // spatial articulated-body inertia $I_i^A$ (see RBDA, equation 7.37)
    //std::vector<BSpatialMatrix> m_IA; // spatial articulated-body inertia $I_i^A$ (see RBDA, equation 7.37)
    std::vector<BSpatialVector> m_pA; // spatial articulated-body bias force $p_i^A$ (see RBDA, equation 7.38)
    
    // human readable names for the bodies
    std::map<std::string, BBodyId> m_bodyNameMap;
};


inline std::ostream&
operator<<( std::ostream &ostr, const BModel &m )
{
    for (int i = 0; i < m.bodies(); ++i)
    {
        ostr << m.body(i) << '\n';
    }
    ostr << '\n';
    for (int i = 0; i < m.joints(); ++i)
    {
        ostr << m.joint(i)  << '\n';
    }

    return ostr;
}

#endif


