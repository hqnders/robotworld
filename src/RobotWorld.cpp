#include "RobotWorld.hpp"
#include "Logger.hpp"
#include "Robot.hpp"
#include "WayPoint.hpp"
#include "Goal.hpp"
#include "Wall.hpp"
#include <algorithm>

namespace model {

/**
 * Return the static instance of robotworld, singleton?
 * @return The robotworld instance
 */
/* static */RobotWorld &RobotWorld::RobotWorld::getRobotWorld() {
  static RobotWorld robotWorld;
  return robotWorld;
}

/**
 * Called to create and add a new robot to the world
 * @param aName Name of the robot
 * @param aPosition Position of the robot
 * @param aNotifyObservers True/false whether to notify the observers
 * @return Pointer to the created robot
 */
RobotPtr RobotWorld::newRobot(const std::string &aName /*= "New Robot"*/,
                              const Point &aPosition /*= Point(-1,-1)*/,
                              bool aNotifyObservers /*= true*/) {
  RobotPtr robot(new Robot(aName, aPosition));

  robots.push_back(robot);

  if (aNotifyObservers) {
    notifyObservers();
  }

  return robot;
}

/**
 * Called to create and add a new waypoint to the world
 * @param aName Name of the waypoint
 * @param aPosition Position of the waypoint
 * @param aNotifyObservers Whether to notify the observers
 * @return Pointer to the created waypoint
 */
WayPointPtr RobotWorld::newWayPoint(const std::string &aName /*= "new WayPoint"*/,
                                    const Point &aPosition /*= Point(-1,-1)*/,
                                    bool aNotifyObservers /*= true*/) {
  WayPointPtr wayPoint(new WayPoint(aName, aPosition));

  wayPoints.push_back(wayPoint);

  if (aNotifyObservers) {
    notifyObservers();
  }

  return wayPoint;
}

/**
 * Called to create and add a new goal to the world
 * @param aName Name of the goal
 * @param aPosition Position of the goal
 * @param aNotifyObservers Whether to notify the observers
 * @return Pointer to the created goal
 */
GoalPtr RobotWorld::newGoal(const std::string &aName /*= "New Goal"*/,
                            const Point &aPosition /*= Point(-1,-1)*/,
                            bool aNotifyObservers /*= true*/) {
  GoalPtr goal(new Goal(aName, aPosition));

  goals.push_back(goal);

  if (aNotifyObservers) {
    notifyObservers();
  }

  return goal;
}

/**
 * Called to create and add a new wall the the world
 * @param aPoint1 First point of the wall
 * @param aPoint2 Second point of the wall
 * @param aNotifyObservers Whether to notify observers
 * @return Pointer to the created wall
 */
WallPtr RobotWorld::newWall(const Point &aPoint1,
                            const Point &aPoint2,
                            bool aNotifyObservers /*= true*/) {
  WallPtr wall(new Wall(aPoint1, aPoint2));

  walls.push_back(wall);

  if (aNotifyObservers) {
    notifyObservers();
  }

  return wall;
}

/**
 * Called to delete a robot from the world
 * @param aRobot Pointer to the robot
 * @param aNotifyObservers Whether to notify observers
 */
void RobotWorld::deleteRobot(RobotPtr aRobot,
                             bool aNotifyObservers /*= true*/) {
  auto i = std::find_if(robots.begin(), robots.end(), [aRobot](RobotPtr r) {
    return aRobot->getName() == r->getName();
  });

  if (i != robots.end()) {
    robots.erase(i);
    if (aNotifyObservers) {
      notifyObservers();
    }
  }
}

/**
 * Called to delete a waypoint from the world
 * @param aWayPoint Pointer to the waypoint
 * @param aNotifyObservers Whether to notify observers
 */
void RobotWorld::deleteWayPoint(WayPointPtr aWayPoint,
                                bool aNotifyObservers /*= true*/) {
  auto i = std::find_if(wayPoints.begin(), wayPoints.end(), [aWayPoint](WayPointPtr w) {
    return aWayPoint->getName() == w->getName();
  });

  if (i != wayPoints.end()) {
    wayPoints.erase(i);
    if (aNotifyObservers) {
      notifyObservers();
    }
  }
}

/**
 * Called to delete a goal from the world
 * @param aGoal Pointer to the goal
 * @param aNotifyObservers Whether to notify observers
 */
void RobotWorld::deleteGoal(GoalPtr aGoal,
                            bool aNotifyObservers /*= true*/) {
  auto i = std::find_if(goals.begin(), goals.end(), [aGoal](GoalPtr g) {
    return aGoal->getName() == g->getName();
  });

  if (i != goals.end()) {
    goals.erase(i);

    if (aNotifyObservers) {
      notifyObservers();
    }
  }
}

/**
 * Called to delete a wall from the world
 * @param aWall Pointer to the wall
 * @param aNotifyObservers Whether to notify observers
 */
void RobotWorld::deleteWall(WallPtr aWall,
                            bool aNotifyObservers /*= true*/) {
  auto i = std::find_if(walls.begin(), walls.end(), [aWall](WallPtr w) {
    return
        aWall->getPoint1() == w->getPoint1() &&
            aWall->getPoint2() == w->getPoint2();
  });

  if (i != walls.end()) {
    walls.erase(i);

    if (aNotifyObservers) {
      notifyObservers();
    }
  }
}

/**
 * Get robot by name
 * @param aName The name
 * @return Pointer to the robot
 */
RobotPtr RobotWorld::getRobot(const std::string &aName) const {
  for (RobotPtr robot : robots) {
    if (robot->getName() == aName) {
      return robot;
    }
  }
  return nullptr;
}

/**
 * Get robot by objectId
 * @param anObjectId The objectId
 * @return Pointer to the robot
 */
RobotPtr RobotWorld::getRobot(const Base::ObjectId &anObjectId) const {
  for (RobotPtr robot : robots) {
    if (robot->getObjectId() == anObjectId) {
      return robot;
    }
  }
  return nullptr;
}

/**
 * Get waypoint by name
 * @param aName The name
 * @return Pointer to the waypoint
 */
WayPointPtr RobotWorld::getWayPoint(const std::string &aName) const {
  for (WayPointPtr wayPoint : wayPoints) {
    if (wayPoint->getName() == aName) {
      return wayPoint;
    }
  }
  return nullptr;
}

/**
 * Get waypoint by objectId
 * @param anObjectId The objectId
 * @return Pointer to the waypoint
 */
WayPointPtr RobotWorld::getWayPoint(const Base::ObjectId &anObjectId) const {
  for (WayPointPtr wayPoint : wayPoints) {
    if (wayPoint->getObjectId() == anObjectId) {
      return wayPoint;
    }
  }
  return nullptr;
}

/**
 * Get goal by name
 * @param aName The name
 * @return Pointer to the goal
 */
GoalPtr RobotWorld::getGoal(const std::string &aName) const {
  for (GoalPtr goal : goals) {
    if (goal->getName() == aName) {
      return goal;
    }
  }
  return nullptr;
}

/**
 * Get goal by objectId
 * @param anObjectId The objectId
 * @return The goal
 */
GoalPtr RobotWorld::getGoal(const Base::ObjectId &anObjectId) const {
  for (GoalPtr goal : goals) {
    if (goal->getObjectId() == anObjectId) {
      return goal;
    }
  }
  return nullptr;
}

/**
 * Get wall by objectId
 * @param anObjectId The objectId
 * @return Pointer to the wall
 */
WallPtr RobotWorld::getWall(const Base::ObjectId &anObjectId) const {
  for (WallPtr wall: walls) {
    if (wall->getObjectId() == anObjectId) {
      return wall;
    }
  }
  return nullptr;
}

const std::vector<RobotPtr> &RobotWorld::getRobots() const {
  return robots;
}

const std::vector<WayPointPtr> &RobotWorld::getWayPoints() const {
  return wayPoints;
}

const std::vector<GoalPtr> &RobotWorld::getGoals() const {
  return goals;
}

const std::vector<WallPtr> &RobotWorld::getWalls() const {
  return walls;
}

/**
 * Populate the world
 * @param aNumberOfWalls The number of walls
 */
void RobotWorld::populate(int aNumberOfWalls /*= 2*/) {
  // Add a new robot
  RobotWorld::getRobotWorld().newRobot("Robot", Point(163, 111), false);

  /*
  static Point coordinates[] = { Point( 100, 400), Point( 350, 300),
                                 Point( 300, 100),
                                 Point( 350, 200) };

  for (int i = 0; i < 2 * aNumberOfWalls; i += 2)
  {
      RobotWorld::getRobotWorld().newWall( coordinates[i], coordinates[i + 1],false);
  }
  */

  // Add a wall
  RobotWorld::getRobotWorld().newWall(Point(7, 234), Point(419, 234), false);

  // Add a goal
  RobotWorld::getRobotWorld().newGoal("Goal", Point(320, 285), false);

  getRobot("Robot")->setGoalToFind(getGoal("Goal"));

  // Notify observers
  notifyObservers();
}

/**
 * Remove all objects from world
 * @param aNotifyObservers Whether to notify observers
 */
void RobotWorld::unpopulate(bool aNotifyObservers /*= true*/) {
  robots.clear();
  wayPoints.clear();
  goals.clear();
  walls.clear();

  if (aNotifyObservers) {
    notifyObservers();
  }
}

/**
 * Remove all objects from the world except for the ones set to keep
 * @param aKeepObjects The objects to keep
 * @param aNotifyObservers Whether to notify observers
 */
void RobotWorld::unpopulate(const std::vector<Base::ObjectId> &aKeepObjects,
                            bool aNotifyObservers /*= true*/) {
  if (robots.size() > 0) {
    robots.erase(std::remove_if(robots.begin(),
                                robots.end(),
                                [&aKeepObjects](RobotPtr aRobot) {
                                  return std::find(aKeepObjects.begin(),
                                                   aKeepObjects.end(),
                                                   aRobot->getObjectId()) == aKeepObjects.end();
                                }),
                 robots.end());
  }

  if (wayPoints.size() > 0) {
    wayPoints.erase(std::remove_if(wayPoints.begin(),
                                   wayPoints.end(),
                                   [&aKeepObjects](WayPointPtr aWayPoint) {
                                     return std::find(aKeepObjects.begin(),
                                                      aKeepObjects.end(),
                                                      aWayPoint->getObjectId()) == aKeepObjects.end();
                                   }),
                    wayPoints.end());
  }

  if (goals.size() > 0) {
    goals.erase(std::remove_if(goals.begin(),
                               goals.end(),
                               [&aKeepObjects](GoalPtr aGoal) {
                                 return std::find(aKeepObjects.begin(),
                                                  aKeepObjects.end(),
                                                  aGoal->getObjectId()) == aKeepObjects.end();
                               }),
                goals.end());
  }

  if (walls.size() > 0) {
    walls.erase(std::remove_if(walls.begin(),
                               walls.end(),
                               [&aKeepObjects](WallPtr aWall) {
                                 return std::find(aKeepObjects.begin(),
                                                  aKeepObjects.end(),
                                                  aWall->getObjectId()) == aKeepObjects.end();
                               }),
                walls.end());
  }

  if (aNotifyObservers) {
    notifyObservers();
  }
}

std::string RobotWorld::asString() const {
  return ModelObject::asString();
}

std::string RobotWorld::asDebugString() const {
  std::ostringstream os;

  os << asString() << '\n';

  for (RobotPtr ptr : robots) {
    os << ptr->asDebugString() << '\n';
  }
  for (WayPointPtr ptr : wayPoints) {
    os << ptr->asDebugString() << '\n';
  }
  for (GoalPtr ptr : goals) {
    os << ptr->asDebugString() << '\n';
  }
  for (WallPtr ptr : walls) {
    os << ptr->asDebugString() << '\n';
  }

  return os.str();
}

RobotWorld::RobotWorld() {
}

RobotWorld::~RobotWorld() {
  // No notification while I am in the destruction mode!
  disableNotification();
  unpopulate();
}
} // namespace model
