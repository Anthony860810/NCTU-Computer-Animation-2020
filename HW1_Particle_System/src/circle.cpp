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
    float dx = (_shape->m_body->GetPosition().x - m_body->GetPosition().x); //x軸向量
    float dy = (_shape->m_body->GetPosition().y - m_body->GetPosition().y); //y軸向量
    float TwoCircleDistance = hypotf(dx,dy); //兩圓距離
    float penetration =_shape->m_radius + m_radius - TwoCircleDistance; //進去深度
    float2 normal = normalize(_shape->m_body->GetPosition() - m_body->GetPosition()) ;
    if (TwoCircleDistance > _shape->m_radius + m_radius) { //兩圓未接觸
        return Manifold(
            m_body,
            _shape->m_body,
            normal,
            0.0f,
            false
        );
    }
    else if (TwoCircleDistance == _shape->m_radius + m_radius) {//兩圓剛好接觸
        return Manifold(
            m_body,
            _shape->m_body,
            normal,
            0.0f,
            true
        );
    }
    else {
        return Manifold(//兩圓已碰撞
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