/*! 
 @file BBox.h 
 @date 27/04/2018

 $$$$$$$$$$$$$$
 $   BBox.h   $
 $$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 This sets up an axis aligned bounding box (AABB)

 Note we use double here - no spatial algebra/diffentiable types
 
 */
 
#ifndef BBOX_H
#define BBOX_H

#ifndef BGLM_H
#include "BGLM.h"
#endif

#ifndef BSTREAM_H
#include "BStream.h"
#endif

#include <cmath>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp> // for glm::length


class BBox
{
public:   
    BBox( void ) : m_top(0.0), m_bot(0.0), m_pos(0.0), m_extent(0.0), m_radius(0.0) {}
    
    explicit BBox( double r ): m_top(glm::dvec3(r)), m_bot(glm::dvec3(-r)), m_pos(0.0), m_extent(glm::dvec3(r)), m_radius(glm::length(m_extent)) {}
        
    BBox( const glm::dvec3 &top, const glm::dvec3 &bot ) : m_top(top), m_bot(bot)
    { 
        // useful for testing  but gets triggered by empty boxes close to zero
        //assert((m_top.x >= m_bot.x && m_top.y >= m_bot.y && m_top.z >= m_bot.z));
        m_pos    = 0.5 * (m_top + m_bot);
        m_extent = 0.5 * (m_top - m_bot);
        m_radius = glm::length(m_extent); 
    }
    
    ~BBox( void )=default;
    
    void
    clear( void )
    {
        m_top.x = m_top.y = m_top.z = m_bot.x = m_bot.y = m_bot.z = 
        m_pos.x = m_pos.y = m_pos.z =    
        m_extent.x = m_extent.y = m_extent.z = m_radius = 0.0;
    }
    
    void
    setSize( double r )
    {
        m_pos = glm::dvec3(0.0);
        m_top = m_extent = glm::dvec3(r);
        m_bot = -m_top;
        m_radius = glm::length(m_extent);
    }
    
    void
    setSize( const glm::dvec3 &r )
    {
        m_pos = glm::dvec3(0.0);
        m_top = m_extent = r;
        m_bot = -m_top;
        m_radius = glm::length(m_extent); 
    }
    
    // assumes BBox is axis aligned AABB
    void 
    setBox( const glm::dvec3 &top, const glm::dvec3 &bot )
    {
        // useful for testing  but gets triggered by empty boxes close to zero
        //assert((m_top.x >= m_bot.x && m_top.y >= m_bot.y && m_top.z >= m_bot.z));
        m_top = top;
        m_bot = bot;
        m_pos = 0.5 * (m_top + m_bot);
        m_extent = 0.5 * (m_top - m_bot);
        m_radius = glm::length(m_extent); 
    }
   
    
    const glm::dvec3&
    top( void ) const { return m_top; }
    
    const glm::dvec3&
    bot( void ) const { return m_bot; }
    
    // centre of box
    const glm::dvec3&
    pos( void ) const { return m_pos; }
    
    glm::dvec3&
    pos( void ) { return m_pos; }
    
    
    // box vertex
    glm::dvec3
    operator[](int i) const 
    {
        switch (i)
        {
            case 0: return glm::dvec3(m_bot.x, m_bot.y, m_top.z); break;
            case 1: return glm::dvec3(m_top.x, m_bot.y, m_top.z); break; 
            case 2: return m_top; break;
            case 3: return glm::dvec3(m_bot.x, m_top.y, m_top.z); break;
            case 4: return m_bot; break;
            case 5: return glm::dvec3(m_top.x, m_bot.y, m_bot.z); break;   
            case 6: return glm::dvec3(m_top.x, m_top.y, m_bot.z); break;
            case 7: return glm::dvec3(m_bot.x, m_top.y, m_bot.z); break;

            default: exit(EXIT_FAILURE); break;
        }
    }
  
    // radius of sphere that contains box
    double
    radius( void ) const { return m_radius; }
    
    double
    radius2( void ) const { return m_radius * m_radius; }
    
    const glm::dvec3&
    extent( void ) const { return m_extent; }
    

    double
    length( void ) const { return (m_top.z - m_bot.z); }
    
    double
    height( void ) const { return (m_top.y - m_bot.y); }
    
    double
    width( void ) const { return (m_top.x - m_bot.x); }

    
    
    bool
    contains( const glm::dvec3 &p ) const
    // assumes axis aligned bounding box (AABB) 
    {
        return (m_bot.x <= p.x && m_top.x >= p.x && m_bot.y <= p.y && m_top.y >= p.y && m_bot.z <= p.z && m_top.z >= p.z);
    }
    

    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BBox &b );
    
    friend std::istream& 
    operator>>( std::istream &istr, BBox &b ); 
    
private:

    glm::dvec3 m_top;       /// top (max) and bottom (min) opposite corners of box 
    glm::dvec3 m_bot;       
    glm::dvec3 m_pos;       /// centre of box; usually relative to some object
    glm::dvec3 m_extent;    /// distance from centre to face
    double m_radius;        /// of sphere that contains box

};


inline std::ostream&
operator<<( std::ostream &ostr, const BBox &b )
{
    ostr << b.m_top << ' ';
    ostr << b.m_bot << ' ';
    ostr << b.m_pos << ' ';
    ostr << b.m_extent << ' ';
    ostr << b.m_radius << ' ';

    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BBox &b )
{
    istr >> b.m_top;
    istr >> b.m_bot;
    istr >> b.m_pos;
    istr >> b.m_extent;
    istr >> b.m_radius;
    
    return istr;
}


namespace arb
{
    
    inline bool 
    intersect(const BBox &b1, const BBox &b2) 
    /// assumes axis aligned bounding boxes b1, b2 (AABB)
    { 
        return  b1.bot().x <= b2.top().x && b1.bot().y <= b2.top().y && b1.bot().z <= b2.top().z && 
        b2.bot().x <= b1.top().x && b2.bot().y <= b1.top().y && b2.bot().z <= b1.top().z;
    }
    
    inline double 
    intersectArea(const BBox& b1, const BBox& b2) 
    /// assumes axis aligned bounding boxes b1, b2 (AABB)
    { 
        double area = 0;
        
        if (intersect(b1, b2))
        {
            glm::dvec3 side(glm::min(b1.top(), b2.top()) - glm::max(b1.bot(), b2.bot()));
            
            // for 2D case
            if (side.z == 0.0) side.z = 1.0;
            
            area = side.x * side.y * side.z;
        }
        
        return area;
    }
    
    inline BBox 
    intersection(const BBox& b1, const BBox& b2) 
    /// assumes axis aligned bounding boxes b1, b2 (AABB)
    {
        if (!intersect(b1, b2))
        {
            return BBox(); 
        }
        
        const glm::dvec3 newBot = glm::max(b1.bot(), b2.bot());
        const glm::dvec3 newTop = glm::min(b1.top(), b2.top());
        
        return BBox(newTop, newBot);
    }

}

#endif
