#include "aabb.hpp"

#include "manifold.hpp"
#include "circle.hpp"
#include "collision.hpp"

#include "GL/freeglut.h"

Manifold AABB::accept(std::shared_ptr<const ShapeVisitor<Manifold>> visitor) const
{
    return visitor->visitAABB(shared_from_this());
}

Manifold AABB::visitAABB(std::shared_ptr<const AABB> _shape) const
{
    // TODO
    float DistanceX = abs((_shape->m_body->GetPosition().x - m_body->GetPosition().x)); //x軸向量
    float DistanceY = abs((_shape->m_body->GetPosition().y - m_body->GetPosition().y)); //y軸向量
    float penetrationX = (m_extent.x+_shape->m_extent.x)/2 - DistanceX;
    float penetrationY = (m_extent.y + _shape->m_extent.y)/2 - DistanceY;
    float2 normalX = {  (_shape->m_body->GetPosition().x - m_body->GetPosition().x)/ DistanceX   , 0.0f };
    float2 normalY = { 0.0f , (_shape->m_body->GetPosition().y - m_body->GetPosition().y)/ DistanceY};
    if (float2{ DistanceX, DistanceY } > (m_extent+_shape->m_extent)/2) { //當兩AABB長寬皆未觸碰到
        return Manifold(
            m_body,
            _shape->m_body,
            float2(0.0f, 0.0f),
            0.0f,
            false
        );
    }
    else if ((DistanceX == (m_extent.x + _shape->m_extent.x) / 2 && DistanceY < (m_extent.y + _shape->m_extent.y) / 2) ||
        (DistanceX < (m_extent.x + _shape->m_extent.x) / 2 && DistanceY == (m_extent.y + _shape->m_extent.y) / 2)) {//其中一邊兩物體相連
        if(DistanceX == (m_extent.x + _shape->m_extent.x) / 2)
            return Manifold(
                m_body,
                _shape->m_body,
                float2(1.0f, 0.0f),
                0.0f,
                true
            );
        else
            return Manifold(
                m_body,
                _shape->m_body,
                float2(0.0f, 1.0f),
                0.0f,
                true
            );
    }
    else if (DistanceX <= (m_extent.x + _shape->m_extent.x) / 2 &&//完全進入
            DistanceY <= (m_extent.y + _shape->m_extent.y) / 2) {
        if (penetrationX < penetrationY) {
            return Manifold(
                m_body,
                _shape->m_body,
                normalX,
                penetrationX,
                true
            );
        }
        else {
            return Manifold(
                m_body,
                _shape->m_body,
                normalY,
                penetrationY,
                true
            );
        }
    }
        return Manifold(
            m_body,
            _shape->m_body,
            float2(0.0f, 0.0f),
            0.0f,
            false
        );
    
    
	//This is a template return object, you should remove it and return your own Manifold
    
}

Manifold AABB::visitCircle(std::shared_ptr<const Circle> _shape) const
{
    auto manifold = CollisionHelper::GenerateManifold(
        shared_from_this(),
        _shape
    );

    return manifold;
}

void AABB::Render() const
{
    glPushMatrix();

    glTranslatef(m_body->GetPosition().x, m_body->GetPosition().y, 0);

    glBegin(GL_LINE_LOOP);
    {
        float2 half_extent = m_extent / 2.0f;

        glVertex2f(0 - half_extent[0], 0 - half_extent[1]);
        glVertex2f(0 - half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 + half_extent[1]);
        glVertex2f(0 + half_extent[0], 0 - half_extent[1]);
    }
    glEnd();

    glBegin(GL_POINTS);
    {
        glPushMatrix();

        glVertex2f(0, 0);

        glPopMatrix();
    }
    glEnd();

    glPopMatrix();
}