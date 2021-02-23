#include "joint.hpp"

#include "GL/freeglut.h"

#include "rigidbody2D.hpp"
#include "util.hpp"

#include <iostream>

void SpringJoint::ApplyConstriant() const
{
    // TODO
    float elongation = length(m_body0->GetPosition() - m_body1->GetPosition()) - m_restLength; 
    float2 normalize_vector = normalize(m_body0->GetPosition() - m_body1->GetPosition());
   // m_body0->AddForce(-m_stiffness * elongation * normalize_vector);
    //m_body1->AddForce(m_stiffness * elongation * normalize_vector);

    float deltav = dot(m_body0->GetVelocity() - m_body1->GetVelocity(), normalize_vector);
    float m_DamperCoeff = m_stiffness / 30;
    
    m_body0->AddForce(-m_stiffness * elongation * normalize_vector -m_DamperCoeff * deltav * normalize_vector);
    m_body1->AddForce(m_stiffness * elongation * normalize_vector+m_DamperCoeff * deltav * normalize_vector);
}

void SpringJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // red for spring joint
        glColor3f(1, 0, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}

void DistanceJoint::ApplyConstriant() const
{
	// TODO


}

void DistanceJoint::Render() const
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        // green for distance joint
        glColor3f(0, 1, 0);
        glVertex2f( m_body0->GetPosition().x, m_body0->GetPosition().y );
        glVertex2f( m_body1->GetPosition().x, m_body1->GetPosition().y );
        glEnd();
    }
    glPopAttrib();
    glPopMatrix();
}