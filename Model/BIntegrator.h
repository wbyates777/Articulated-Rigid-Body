/*! 

         $$$$$$$$$$$$$$$$$$$$$
         $   BIntegrator.h   $
         $$$$$$$$$$$$$$$$$$$$$

 @file BIntegrator.h
 @ingroup Model
 date 22/10/2026
 
 by W.B. Yates
 
 ---
 
 @brief Use numerical or Euler methods (depending on joints) to compute the joint
 positions and velocities after a time step assuming constant joint accelerations.
 
 Provides integration methods for single body (in world coords) and multibody models (in local/body coords).
 
 The process is split into two so that we can perform collision detection and resolution
 after velocity update but before position update.
 
 
 
*/


#ifndef BINTEGRATOR_H
#define BINTEGRATOR_H




class BBody;
class BJoint;
class BModel;
class BModelState;

class BIntegrator
{

public:

    BIntegrator( void ): m_limits(true) {}
    ~BIntegrator( void )=default;

    /// Use model joint parameter limits.
    void
    setLimits( bool on ) { m_limits = on; }
    
    /// Single body position.
    void
    updatePos( BBody &body, double dt );
    
    /// Single body velocity.
    void
    updateVel( BBody &body, double dt );
    

    /// Multibody model positions.
    void
    updatePos( BModel &model, BModelState &qstate, double dt );

    /// Multibody model velocities.
    void
    updateVel( BModel &model, BModelState &qstate, double dt );
  

private:

    void 
    updatePos( BJoint &joint, BModelState &qstate, double dt );
    
    bool m_limits;

};

#endif


