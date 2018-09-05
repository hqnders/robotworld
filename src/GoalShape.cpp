#include "GoalShape.hpp"
#include "Goal.hpp"
#include "Logger.hpp"
#include <sstream>

namespace View
{
	/**
	 *
	 */
	GoalShape::GoalShape( model::GoalPtr aGoal) :
								WayPointShape( std::dynamic_pointer_cast<model::WayPoint>(aGoal))
	{
	}
	/**
	 *
	 */
	GoalShape::~GoalShape()
	{
	}
	/**
	 *
	 */
	model::GoalPtr GoalShape::getGoal() const
	{
		return std::dynamic_pointer_cast<model::Goal>(getModelObject());
	}
	/**
	 *
	 */
	void GoalShape::setGoal( model::GoalPtr aGoal)
	{
		setModelObject(std::dynamic_pointer_cast<model::ModelObject>(aGoal));
	}
	/**
	 *
	 */
	void GoalShape::handleNotification()
	{
	}
	/**
	 *
	 */
	std::string GoalShape::asString() const
	{
		std::ostringstream os;

		os << "GoalShape " << WayPointShape::asString();

		return os.str();
	}
	/**
	 *
	 */
	std::string GoalShape::asDebugString() const
	{
		std::ostringstream os;

		os << "GoalShape:\n";
		os << WayPointShape::asDebugString() << "\n";
		// Goal has not use in this version so we do not print it
		//	if(getGoal())
		//	{
		//		os << getGoal()->asDebugString();
		//	}

		return os.str();
	}
} // namespace View
