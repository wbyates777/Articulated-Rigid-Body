/* BModelState 29/11/2025

 $$$$$$$$$$$$$$$$$$$$$
 $   BModelState.h   $
 $$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Holds the state of every joint in a model as vectors of scalars.

     std::vector<BScalar>  m_q;     // positions
     std::vector<BScalar>  m_qdot;  // velocities
     std::vector<BScalar>  m_qddot; // accelerations
     std::vector<BScalar>  m_tau;   // forces

 The q, qdot, qddot, tau notation is taken from Featherstone's RBDA.
 
 Notes 
 when using spherical joints (i.e FloatBase)
   initialising q to zero is safe because glm::mat3_cast(glm::quat(0,0,0,0)) -> B_IDENTITY_3x3
   initialising q to one is not safe as glm::mat3_cast(glm::quat(1,1,1,1)) is not 
   normalized and therefore not a rotation
 
*/


#ifndef __BMODELSTATE_H__
#define __BMODELSTATE_H__


#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif


#ifndef __BMODEL_H__
#include "BModel.h"
#endif

#ifndef __BJOINT_H__
#include "BJoint.h"
#endif



#include <vector>


class BModelState 
{
    
public:
    
    BModelState( void )=default;
    explicit BModelState( int expected_dof )
    {
        m_q.reserve(expected_dof);
        m_qdot.reserve(expected_dof);
        m_qddot.reserve(expected_dof);
        m_tau.reserve(expected_dof);
    }
    explicit BModelState( const BModel &model )
    {
        m_q.resize(model.qsize(), 0.0);
        m_qdot.resize(model.qdotsize(), 0.0);
        m_qddot.resize(model.qdotsize(), 0.0);
        m_tau.resize(model.qdotsize(), 0.0);
    }
    ~BModelState( void )=default;


    void
    set( const BModel &model )
    {
        m_q.resize(model.qsize(),0.0);
        m_qdot.resize(model.qdotsize(), 0.0);
        m_qddot.resize(model.qdotsize(), 0.0);
        m_tau.resize(model.qdotsize(), 0.0);
    }
    
    void
    reset( void )
    {
        m_q.assign(m_q.size(), 0.0); 
        m_qdot.assign(m_qdot.size(), 0.0);
        m_qddot.assign(m_qddot.size(), 0.0);
        m_tau.assign(m_tau.size(), 0.0);
    }

    //
    // joint state values
    //
    std::vector<BScalar>&
    q( void )  { return m_q; }               // position
    
    const std::vector<BScalar>&
    q( void ) const { return m_q; }         
    
    std::vector<BScalar>&
    qdot( void ) { return m_qdot; }          // velocity
    
    const std::vector<BScalar>&
    qdot( void ) const { return m_qdot; }   
    
    std::vector<BScalar>&
    qddot( void ) { return m_qddot; }        // acceleration
    
    const std::vector<BScalar>&
    qddot( void ) const { return m_qddot; }  
    
    std::vector<BScalar>&
    tau( void ) { return m_tau; }           // forces
    
    const std::vector<BScalar>&
    tau( void ) const { return m_tau; }    

    
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BModelState &m );
    
    friend std::istream& 
    operator>>( std::istream &istr, BModelState &m );
    
private:
    

    // joint state parameters for all joints in a BFloatBase
    std::vector<BScalar>  m_q;     // positions
    std::vector<BScalar>  m_qdot;  // velocities
    std::vector<BScalar>  m_qddot; // accelerations
    std::vector<BScalar>  m_tau;   // forces
};

inline std::ostream&
operator<<( std::ostream &ostr, const BModelState &j )
{
    ostr <<  j.m_q << '\n';
    ostr <<  j.m_qdot << '\n';
    ostr <<  j.m_qddot << '\n';
    ostr <<  j.m_tau << '\n';
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BModelState &j )
{
    istr >> j.m_q >> j.m_qdot >> j.m_qddot >> j.m_tau;
    return istr;
}

#endif


