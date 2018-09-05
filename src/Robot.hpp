#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "Config.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "AbstractAgent.hpp"
#include "AStar.hpp"
#include "BoundedVector.hpp"
#include "Message.hpp"
#include "MessageHandler.hpp"
#include "Observer.hpp"
#include "Point.hpp"
#include "Size.hpp"
#include "Region.hpp"
#include "Client.hpp"

namespace Messaging {
class Message;
}

namespace model {
class Robot;
typedef std::shared_ptr<Robot> RobotPtr;

class Goal;
typedef std::shared_ptr<Goal> GoalPtr;

class Robot : public AbstractAgent,
              public Messaging::MessageHandler,
              public Base::Observer {
 public:
  Robot();
  Robot(const std::string &aName);
  Robot(const std::string &aName, const Point &aPosition);
  virtual ~Robot();

  std::string getName() const {
    return name;
  }

  Size getSize() const {
    return size;
  };

  Point getPosition() const {
    return position;
  }

  BoundedVector getFront() const {
    return front;
  };

  float getSpeed() const {
    return speed;
  };

  void setPosition(const Point &aPosition, bool aNotifyObservers = true);
  void setSize(const Size &aSize, bool aNotifyObservers = true);
  void setName(const std::string &aName, bool aNotifyObservers = true);
  void setFront(const BoundedVector &aVector, bool aNotifyObservers = true);
  void setSpeed(float aNewSpeed, bool aNotifyObservers = true);

  /**
   * @return true if the robot is acting, i.e. either planning or driving
   */
  bool isActing() const {
    return acting;
  }

  /**
   * Make the robot act and drive
   */
  virtual void startActing();

  /**
   * Stop the robot from acting and driving
   */
  virtual void stopActing();

  /**
   * @return true if the robot is driving
   */
  bool isDriving() const {
    return driving;
  }

  /**
   * Calculate route to goal and drive
   */
  virtual void startDriving();

  /**
   * Set driving to false
   */
  virtual void stopDriving();

  /**
   * @return true if the robot is communicating, i.e. listens with an active ServerConnection
   */
  bool isCommunicating() const {
    return communicating;
  }

  /**
   * Starts a ServerConnection that listens at port 12345 unless given
   * an other port by specifying a command line argument -local_port=port
   */
  void startCommunicating();

  /**
   * Connects to the ServerConnection that listens at port 12345 unless given
   * an other port by specifying a command line argument -local_port=port
   * and sends a message with messageType "1" and a body with "stop"
   *
   */
  void stopCommunicating();

  /**
   * Return the region of the robot, calculated by it's 4 corner points
   */
  Region getRegion() const;

  /**
   * @return Whether the robot is colliding with the given region
   */
  bool intersects(const Region &aRegion) const;

  /**
   * @return Point of the robot's front left
   */
  Point getFrontLeft() const;

  /**
   * @return Point of the robot's front right
   */
  Point getFrontRight() const;

  /**
   * @return Point of the robot's back left
   */
  Point getBackLeft() const;

  /**
   * @return Point of the robot's back right
   */
  Point getBackRight() const;

  const std::shared_ptr<Messaging::Client> &getClient() const {
    return client;
  }


  /**
   * @name Observer functions
   */
  //@{
  /**
   * A Notifier will call this function if this Observer will handle the notifications of that
   * Notifier. It is the responsibility of the Observer to filter any events it is interested in.
   */
  virtual void handleNotification();
  //@}

  /**
   *
   */
  PathAlgorithm::OpenSet getOpenSet() const {
    return astar.getOpenSet();
  }

  /**
   *
   */
  PathAlgorithm::Path getPath() const {
    return path;
  }

  const GoalPtr &getGoalToFind() const {
    return goalToFind;
  }

  void setGoalToFind(const GoalPtr &goalToFind) {
    Robot::goalToFind = goalToFind;
  }

  /**
   * @name Messaging::MessageHandler functions
   */
  //@{
  /**
   * This function is called by a ServerSesssion whenever a message is received. If the request is handled,
   * any response *must* be set in the Message argument. The message argument is then echoed back to the
   * requester, probably a ClientSession.
   *
   * @see Messaging::RequestHandler::handleRequest( Messaging::Message& aMessage)
   */
  virtual void handleRequest(Messaging::Message &aMessage);

  /**
   * This function is called by a ClientSession whenever a response to a previous request is received.
   *
   * @see Messaging::ResponseHandler::handleResponse( const Messaging::Message& aMessage)
   */
  virtual void handleResponse(const Messaging::Message &aMessage);

  void onEchoRequest(Messaging::Message &aMessage);
  void onEchoResponse(const Messaging::Message &aMessage);
  void onSynchRequest(Messaging::Message &aMessage);
  void onSynchResponse(const Messaging::Message &aMessage);
  void onStartRobotRequest(Messaging::Message &aMessage);
  void onStartRobotResponse(const Messaging::Message &aMessage);
  //@}

  virtual std::string asString() const;
  virtual std::string asDebugString() const;
  //@}

  /**
   * @name The types of messages a Robot should understand
   */
  //@{
  enum MessageType {
    synchRequest,
    synchResponse,
    EchoRequest,
    EchoResponse,
    startRobotRequest,
    startRobotResponse
  };

  /**
   * util to split string
   */
  void splitStringToStringVec(const std::string &aString, char aDelim, std::vector<std::string> &aVec);

  /**
  * Sets up the client which the robot communicates over
  */
  void setupClient();

 protected:
  /**
   * Drive to the point without stopping
   */
  void drive();

  /**
   * Calculate the fastest route towards the goal
   */
  void calculateRoute(GoalPtr aGoal);

  /**
   * @return Whether or not the robot is at it's goal
   */
  bool arrived(GoalPtr aGoal);

  /**
   * @return Whether or not the robot is colliding with any walls
   */
  bool collision();

 private:
  GoalPtr goalToFind;
  std::string name;

  Size size;
  Point position;

  BoundedVector front;
  float speed;

  GoalPtr goal;
  PathAlgorithm::AStar astar;
  PathAlgorithm::Path path;

  bool acting;
  bool driving;
  bool communicating;

  std::thread robotThread;
  mutable std::recursive_mutex robotMutex;

  std::shared_ptr<Messaging::Client> client;
};
} // namespace model

#endif // ROBOT_HPP_
