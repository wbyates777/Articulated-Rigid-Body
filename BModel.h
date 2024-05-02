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

class BModel
{

public:

    BModel( int expected_dof = 1 );
    ~BModel( void )=default;


    BJoint&
    joint( BJointID jid ) { return m_joints[jid]; }
    
    const BJoint&
    joint( BJointID jid ) const { return m_joints[jid]; } 
    
    size_t 
    joints( void ) const { return m_joints.size(); } // $N_J$
    
    BBody&
    body( BBodyID bid ) { return m_bodies[bid]; }
    
    const BBody&
    body( BBodyID bid ) const { return m_bodies[bid]; }
    
    size_t
    bodies( void ) const { return m_bodies.size(); }     // $N_B$
        
    

    /** \brief Connects a given body to the model
     *
     * When adding a body there are basically informations required:
     * - what kind of body will be added?
     * - where is the new body to be added?
     * - by what kind of joint should the body be added?
     *
     * The first information "what kind of body will be added" is contained
     * in the Body class that is given as a parameter.
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
     *                    of the joint frame (represents X_T in RBDA)
     * \param joint       specification for the joint that describes the
     *                    connection
     * \param body        specification of the body itself
     * \param body_name   human readable name for the body (can be used to
     *                    retrieve its id with GetBodyId())
     *
     * \returns id of the added body
     */
    BBodyID 
    addBody( BBodyID parent_id,
             const BSpatialTransform &joint_frame,
             const BJoint &joint,
             const BBody &body,
             const std::string &body_name = "" );    


    
    // set a bodies parameters -- updates whole model
    void 
    setMass( BBodyID bid, BScalar mass );
    
    void 
    setInertia( BBodyID bid, const BMatrix3 &inertia );
    
    void 
    setCOM( BBodyID bid, const BVector3 &com );
    
    void 
    setParameters( BBodyID bid, BScalar mass, const BMatrix3 &inertia, const BVector3 &com );
    //
    
    
    // size of the \f$\mathbf{q}\f$-vector.
    int  
    qsize( void ) const { return q_size; }
    
    // size of the (\f$\mathbf{\dot{q}}, \mathbf{\ddot{q}}\f$, and \f$\mathbf{\tau}\f$-vector.
    int  
    qdotsize( void ) const { return qdot_size; }
    
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

    // IA and pA are the articulated-body inertia $I_i^A$ and bias forces $p_i^A$ for a body $Bi$ in $A_i$.
    const BSpatialMatrix& 
    IA( int i ) const { return m_IA[i]; }
    
    BSpatialMatrix& 
    IA( int i ) { return m_IA[i]; }
    
    const BSpatialVector& 
    pA( int i ) const { return m_pA[i]; }
    
    BSpatialVector& 
    pA( int i ) { return m_pA[i]; }
    
    

    BBodyID
    parentID( BBodyID bid )  const { return m_lambda[bid]; }
    
    BBodyID 
    getBodyId( const std::string &body_name ) const;

    std::string 
    getBodyName( BBodyID bid ) const;

    bool 
    isBodyId( BBodyID bid ) const;

    /** Determines bid the actual parent body.
     *
     * When adding bodies using joints with multiple degrees of
     * freedom, additional virtual bodies are added for each degree of
     * freedom. This function returns the id of the actual
     * non-virtual parent body.
     */
    BBodyID 
    getParentBodyId( BBodyID bid ) const;

    // joint frame transformtion, i.e. the second argument to BModel::addBody()
    BSpatialTransform 
    getJointFrame( BBodyID bid ) const;

    // set the joint frame transformtion, i.e. the second argument to  BModel::addBody()
    void 
    setJointFrame( BBodyID bid, const BSpatialTransform &transform );
    
   // friend std::ostream&
  //  operator<<( std::ostream &ostr, const BModel &m );
    
private:

    BBodyID 
    addBodyMultiDofJoint( BBodyID parent_id,
                          const BSpatialTransform &joint_frame,
                          const BJoint &joint,
                          const BBody &body,
                          const std::string &body_name = ""); 
    
    BBodyID
    addBodyFixedJoint( BBodyID parent_id,
                       const BSpatialTransform &joint_frame,
                       const BJoint &joint,
                       const BBody &body,
                       const std::string &body_name = "" );   
    
    bool 
    isFixedBodyId( BBodyID bid ) const;
    

    void 
    updateInertia( BBodyID bid );

    // number of degrees of freedoms of the model
    // general (joint) state (q), velocity (qdot), acceleration (qddot)
    int m_dof_count; 
    int q_size; 
    int qdot_size;


    BVector3 m_gravity;

    // the id of the parent body
    std::vector<BBodyID> m_lambda;
    
    std::vector<BJoint> m_joints;
    std::vector<BBody>  m_bodies;
    std::vector<BFixedBody> m_fixedBodies;  //  bodies attached via a fixed joint
    BBodyID m_fbd; // fixed_body_discriminator

    std::vector<BSpatialMatrix> m_IA; // spatial inertia (see RBDA, equation 7.37)
    std::vector<BSpatialVector> m_pA; // spatial bias force (see RBDA, equation 7.38)
    
    // human readable names for the bodies
    std::map<std::string, BBodyID> m_bodyNameMap;
};


inline std::ostream&
operator<<( std::ostream& ostr, const BModel& m )
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


