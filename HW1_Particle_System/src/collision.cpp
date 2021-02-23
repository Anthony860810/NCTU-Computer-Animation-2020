#include "collision.hpp"

#include <algorithm>
#include <iostream>

#include "linalg.h"

Manifold CollisionHelper::GenerateManifold(std::shared_ptr<const AABB> _a, std::shared_ptr<const Circle> _b)
{
	float DistanceX = abs(_a->m_body->GetPosition().x - _b->m_body->GetPosition().x); // x 距離
	float DistanceY = abs(_a->m_body->GetPosition().y - _b->m_body->GetPosition().y); // y 距離
	linalg::aliases::float2 port1 = _a->m_body->GetPosition() + _a->m_extent / 2;//右上
	linalg::aliases::float2 port2 = _a->m_body->GetPosition() + linalg::aliases::float2(-_a->m_extent.x, _a->m_extent.y) / 2;//左上
	linalg::aliases::float2 port3 = _a->m_body->GetPosition() - _a->m_extent / 2;
	linalg::aliases::float2 port4 = _a->m_body->GetPosition() + linalg::aliases::float2(_a->m_extent.x, -_a->m_extent.y) / 2;//左上
	float case1 = linalg::min(hypotf(_b->m_body->GetPosition().x - port1.x, _b->m_body->GetPosition().y - port1.y)
		, hypotf(_b->m_body->GetPosition().x - port2.x, _b->m_body->GetPosition().y - port2.y));
	float case2 = linalg::min(hypotf(_b->m_body->GetPosition().x - port3.x, _b->m_body->GetPosition().y - port3.y)
		, hypotf(_b->m_body->GetPosition().x - port4.x, _b->m_body->GetPosition().y - port4.y));
	float shortport = linalg::min(case1, case2);
	if (DistanceX < _a->m_extent.x / 2 && DistanceY < _a->m_extent.y / 2) { //圓心在矩形內
		float penetrationX = _a->m_extent.x / 2 - DistanceX + _b->m_radius;
		float penetrationY = _a->m_extent.y / 2 - DistanceY + _b->m_radius;
		float normalX = (_b->m_body->GetPosition().x - _a->m_body->GetPosition().x) / DistanceX;
		float normalY = (_b->m_body->GetPosition().y - _a->m_body->GetPosition().y) / DistanceY;
		if (_a->m_extent.x / 2 - DistanceX < _a->m_extent.y / 2 - DistanceY) { // X軸深度較淺
			
			return Manifold(
				_a->m_body,
				_b->m_body,
				linalg::aliases::float2(normalX, 0.0f),
				penetrationX ,
				true
			);
		}
		else {	// Y軸深度較淺

			return Manifold(
				_a->m_body,
				_b->m_body,
				linalg::aliases::float2(0.0f, normalY),
				penetrationY,
				true
			);
		}
	}
	else if (DistanceX == _a->m_extent.x / 2 && DistanceY <= _a->m_extent.y / 2) {//圓心在邊界上
		float penetrationX = _a->m_extent.x / 2 - DistanceX + _b->m_radius;
		float normalX = (_b->m_body->GetPosition().x - _a->m_body->GetPosition().x) / DistanceX;
		return Manifold(
			_a->m_body,
			_b->m_body,
			linalg::aliases::float2(normalX, 0.0f),
			penetrationX,
			true
		);
	}
	else if (DistanceX < _a->m_extent.x / 2 && DistanceY == _a->m_extent.y / 2) {//圓心在邊界上
		float penetrationY = _a->m_extent.y / 2 - DistanceY + _b->m_radius;
		float normalY = (_b->m_body->GetPosition().y - _a->m_body->GetPosition().y) / DistanceY;
		return Manifold(
			_a->m_body,
			_b->m_body,
			linalg::aliases::float2(0.0f, normalY),
			penetrationY,
			true
		);
	}
	//四角落處理
	else if ((_b->m_body->GetPosition() > port1 && hypotf(_b->m_body->GetPosition().x-port1.x, _b->m_body->GetPosition().y-port1.y)<_b->m_radius)
			|| (_b->m_body->GetPosition().x < port2.x && _b->m_body->GetPosition().y > port2.y && hypotf(_b->m_body->GetPosition().x - port2.x, _b->m_body->GetPosition().y - port2.y) < _b->m_radius)
			|| (_b->m_body->GetPosition() < port3  && hypotf(_b->m_body->GetPosition().x - port3.x, _b->m_body->GetPosition().y - port3.y) < _b->m_radius)
			|| (_b->m_body->GetPosition().x > port4.x && _b->m_body->GetPosition().y < port4.y && hypotf(_b->m_body->GetPosition().x - port4.x, _b->m_body->GetPosition().y - port4.y) < _b->m_radius)) {
		float penetrationX = _a->m_extent.x/2+ _b->m_radius-DistanceX ;
		float penetrationY = _a->m_extent.y / 2+_b->m_radius - DistanceY;
		float normalX = (_b->m_body->GetPosition().x - _a->m_body->GetPosition().x) / DistanceX;
		float normalY = (_b->m_body->GetPosition().y - _a->m_body->GetPosition().y) / DistanceY;
		if (_a->m_extent.x / 2 - DistanceX < _a->m_extent.y / 2 - DistanceY) { // X軸深度較淺
			return Manifold(
				_a->m_body,
				_b->m_body,
				linalg::aliases::float2(normalX, 0.0f),
				penetrationX ,
				true
			);
		}
		else {	// Y軸深度較淺
			return Manifold(
				_a->m_body,
				_b->m_body,
				linalg::aliases::float2(0.0f, normalY),
				penetrationY,
				true
			);
		}
	}
	else if (_a->m_extent.x / 2 < DistanceX && DistanceX < _a->m_extent.x / 2 + _b->m_radius &&	 DistanceY < _a->m_extent.y / 2) {
		float penetrationX = _a->m_extent.x / 2 + _b->m_radius - DistanceX;
		float normalX = (_b->m_body->GetPosition().x - _a->m_body->GetPosition().x) / DistanceX;
			return Manifold(
				_a->m_body,
				_b->m_body,
				linalg::aliases::float2(normalX, 0.0f),
				penetrationX,
				true
			);
		
	}
	else if ( DistanceX < _a->m_extent.x / 2  && _a->m_extent.y / 2 < DistanceY && DistanceY < _a->m_extent.y / 2 + _b->m_radius) {
		
		float penetrationY = _a->m_extent.y / 2 + _b->m_radius - DistanceY;
		
		float normalY = (_b->m_body->GetPosition().y - _a->m_body->GetPosition().y) / DistanceY;
		// Y軸深度較淺
			return Manifold(
				_a->m_body,
				_b->m_body,
				linalg::aliases::float2(0.0f, normalY),
				penetrationY,
				true
			);
	}
	else if (DistanceX == _a->m_extent.x / 2 + _b->m_radius && _a->m_extent.y / 2 < DistanceY && DistanceY < _a->m_extent.y / 2 + _b->m_radius) {
	return Manifold(
				_a->m_body,
				_b->m_body,
				linalg::aliases::float2(1.0f, 0.0f),
				0,
				true
		   );
	}
	else if (DistanceY == _a->m_extent.y / 2 + _b->m_radius && _a->m_extent.x / 2 < DistanceX && DistanceX < _a->m_extent.x / 2 + _b->m_radius) {
	return Manifold(
		_a->m_body,
		_b->m_body,
		linalg::aliases::float2(0.0f, 1.0f),
		0,
		true
	);
	}
	return Manifold(
		_a->m_body,
		_b->m_body,
		linalg::aliases::float2(0.0f, 0.0f),
		0.0f,
		false
	);
}