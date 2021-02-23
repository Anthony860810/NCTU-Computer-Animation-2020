#include "circle.hpp"

#include "manifold.hpp"
#include "aabb.hpp"
#include "collision.hpp"

#include "GL/freeglut.h"

#include <cmath>

Manifold Circle::accept(std::shared_ptr<const ShapeVisitor<Manifold>> visitor) const
{
    return visitor->visitCircle(shared_from_this());
}

Manifold Circle::visitAABB(std::shared_ptr<const AABB> _shape) const
{
    auto manifold = CollisionHelper::GenerateManifold(
        _shape,
        shared_from_this()
    );

    return manifold;
}

Manifold Circle::visitCircle(std::shared_ptr<const Circle> _shape) const
{
    // TODO
    float dx = (_shape->m_body->GetPosition().x - m_body->GetPosition().x); //x�b�V�q
    float dy = (_shape->m_body->GetPosition().y - m_body->GetPosition().y); //y�b�V�q
    float TwoCircleDistance = hypotf(dx,dy); //���Z��
    float penetration =_shape->m_radius + m_radius - TwoCircleDistance; //�i�h�`��
    float2 normal = normalize(_shape->m_body->GetPosition() - m_body->GetPosition()) ;
    if (TwoCircleDistance > _shape->m_radius + m_radius) { //��ꥼ��Ĳ
        return Manifold(
            m_body,
            _shape->m_body,
            normal,
            0.0f,
            false
        );
    }
    else if (TwoCircleDistance == _shape->m_radius + m_radius) {//����n��Ĳ
        return Manifold(
            m_body,
            _shape->m_body,
            normal,
            0.0f,
            true
        );
    }
    else {
        return Manifold(//���w�I��
            m_body,
            _shape->m_body,
            normal,
           penetration,
            true
        );
    }

	//This is a template return object, you should remove it and return your own Manifold
    
}

void Circle::Render() const
{
    const size_t k_segments = 200;

    glPushMatrix();
    glBegin(GL_LINE_LOOP);
    {
        float theta = 0.0f;
        float inc = (float)M_PI * 2.0f / k_segments;
        for(size_t i = 0; i < k_segments; ++i)
        {
            theta += inc;
            float2 p( std::cos( theta ), std::sin( theta ) );
            p *= m_radius;
            p += m_body->GetPosition();
            glVertex2f( p.x, p.y );
        }
    }
    glEnd( );
    glPopMatrix();
}