#include "BoundedVector.hpp"
#include <cmath>
#include <sstream>
#include "MathUtils.hpp"

namespace model
{
	/**
	 *
	 */
	float BoundedVector::getMagnitude()
	{
		return std::sqrt( x * x + y * y);
	}
	/**
	 *
	 */
	void BoundedVector::normalise()
	{
		float magnitude = getMagnitude();

		if (magnitude <= Utils::ALMOST_ZERO)
			magnitude = 1;
		x /= magnitude;
		y /= magnitude;

		if (std::fabs( x) < Utils::ALMOST_ZERO)
			x = 0.0;
		if (std::fabs( y) < Utils::ALMOST_ZERO)
			y = 0.0;

	}
	/**
	 *
	 */
	BoundedVector BoundedVector::getNormalised()
	{
		BoundedVector polarCoord( *this);
		polarCoord.normalise();
		return polarCoord;
	}
	/**
	 *
	 */
	void BoundedVector::reverse()
	{
		x = -x;
		y = -y;
	}
	/**
	 *
	 */
	BoundedVector& BoundedVector::operator+=( const BoundedVector& aVector)
				{
		x += aVector.x;
		y += aVector.y;

		return *this;
				}
	/**
	 *
	 */
	BoundedVector& BoundedVector::operator-=( const BoundedVector& aVector)
				{
		x -= aVector.x;
		y -= aVector.y;

		return *this;
				}
	/**
	 *
	 */
	BoundedVector& BoundedVector::operator*=( float aScalar)
				{
		x *= aScalar;
		y *= aScalar;

		return *this;
				}
	/**
	 *
	 */
	BoundedVector& BoundedVector::operator/=( float aScalar)
				{
		x /= aScalar;
		y /= aScalar;

		return *this;
				}
	/**
	 *
	 */
	BoundedVector BoundedVector::operator-()
	{
		return BoundedVector( -x, -y);
	}
	/**
	 *
	 */
	std::string BoundedVector::asString() const
	{
		std::ostringstream os;

		os << "(" << x << "," << y << ")";

		return os.str();
	}
	/**
	 *
	 */
	std::string BoundedVector::asDebugString() const
	{
		return asString();
	}
} // namespace model
/**
 *
 */
model::BoundedVector operator+(	const model::BoundedVector& lhs,
								const model::BoundedVector& rhs)
{
	return model::BoundedVector( lhs.x + rhs.x, lhs.y + rhs.y);
}
/**
 *
 */
model::BoundedVector operator-(	const model::BoundedVector& lhs,
								const model::BoundedVector& rhs)
{
	return model::BoundedVector( lhs.x - rhs.x, lhs.y - rhs.y);
}
/**
 *
 */
float operator*(	const model::BoundedVector& lhs,
					const model::BoundedVector& rhs)
{
	return (lhs.x * rhs.x) + (lhs.y * rhs.y);
}
/**
 *
 */
model::BoundedVector operator*( 	float s,
								const model::BoundedVector& rhs)
{
	return rhs * s;
}
/**
 *
 */
model::BoundedVector operator*(	const model::BoundedVector& lhs,
								float s)
{
	return model::BoundedVector( lhs.x * s, lhs.y * s);
}
/**
 *
 */
model::BoundedVector operator/(	const model::BoundedVector& lhs,
								float s)
{
	return model::BoundedVector( lhs.x / s, lhs.y / s);
}
