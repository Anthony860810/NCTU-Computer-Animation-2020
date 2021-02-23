#include "manifold.hpp"

#include <iostream>

Manifold::Manifold(
    std::shared_ptr<RigidBody2D> _body0, 
    std::shared_ptr<RigidBody2D> _body1,
    float2 _normal,
    float _penetration,
    bool _isHit)
    : m_body0(_body0), m_body1(_body1), m_normal(_normal),
      m_penetration(_penetration), m_isHit(_isHit)
    {}

void Manifold::Resolve() const
{
    float e = std::min(m_body0->m_restitution, m_body1->m_restitution);
	// TODO
    if (m_isHit) {
        //normal impulse
        float2 v_relative =  m_body0->GetVelocity() - m_body1->GetVelocity();
        float2 v_normal = dot(v_relative,m_normal)*m_normal; //法線速度
        float2 Impulse_normal = (1 + e) * v_normal / (m_body0->GetInvMass() + m_body1->GetInvMass());//法線衝量
        if (m_body0->GetMass() != 0) {
            m_body0->AddVelocity(-Impulse_normal * m_body0->GetInvMass());
            
        }
        if (m_body1->GetMass() != 0) {
            m_body1->AddVelocity(Impulse_normal * m_body1->GetInvMass());
            
        }
        v_relative = m_body0->GetVelocity() - m_body1->GetVelocity();
        v_normal = dot(v_relative, m_normal) * m_normal; //法線速度
        float2 v_tangent = v_relative - v_normal; //切線速度
        if (length(v_tangent) != 0) {
            float2 Impulse_tangent = (1 + e) * v_tangent / (m_body0->GetInvMass() + m_body1->GetInvMass());//切線衝量
            v_tangent = normalize(v_tangent);
            float2 MaxStaticfriction = sqrtf(m_body0->m_staticFriction*m_body1->m_staticFriction)*Impulse_normal;
            float Dynamicfriction =sqrtf(m_body0->m_dynamicFriction* m_body1->m_dynamicFriction);
            float2 Force_tangent;
            if (length(Impulse_tangent) <= length(MaxStaticfriction)) {
                Force_tangent = Impulse_tangent;
            }
            else {
                Force_tangent = Dynamicfriction * length(Impulse_normal) * v_tangent;
            }
            if (m_body0->GetMass() != 0) {
                m_body0->AddVelocity(-Force_tangent * m_body0->GetInvMass());
                //m_body0->AddForce(-Force_tangent);
            }
            if (m_body1->GetMass() != 0) {
                m_body1->AddVelocity(Force_tangent * m_body1->GetInvMass());
                //m_body1->AddForce(Force_tangent);
            }
        }
    }
    

}

void Manifold::PositionalCorrection() const
{
    const float percent = 0.4f; // usually 20% to 80%, when fps is 1/60
    const float slop = 0.01f;

	const float inv_mass_a = m_body0->GetInvMass();
	const float inv_mass_b = m_body1->GetInvMass();

    if(inv_mass_a == 0.0f && inv_mass_b == 0.0f)
        return;

    float2 correction = 
        (std::max( m_penetration - slop, 0.0f ) / (inv_mass_a + inv_mass_b))
        * percent * m_normal;

    m_body0->m_position -= inv_mass_a * correction;
    m_body1->m_position += inv_mass_b * correction;
}