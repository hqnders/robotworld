#include "Robot.hpp"
#include <sstream>
#include <ctime>
#include <chrono>
#include "Thread.hpp"
#include "MathUtils.hpp"
#include "Logger.hpp"
#include "Goal.hpp"
#include "WayPoint.hpp"
#include "Wall.hpp"
#include "RobotWorld.hpp"
#include "Shape2DUtils.hpp"
#include "CommunicationService.hpp"
#include "Client.hpp"
#include "Message.hpp"
#include "MainApplication.hpp"
#include "LaserDistanceSensor.hpp"
#include "MainApplication.hpp"

namespace model {
Robot::Robot() :
    name(""),
    size(DefaultSize),
    position(DefaultPosition),
    front(0, 0),
    speed(0.0),
    acting(false),
    driving(false),
    communicating(false) {
  std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
  attachSensor(laserSensor);
}

Robot::Robot(const std::string &aName) :
    name(aName),
    size(DefaultSize),
    position(DefaultPosition),
    front(0, 0),
    speed(0.0),
    acting(false),
    driving(false),
    communicating(false) {
  std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
  attachSensor(laserSensor);
}

Robot::Robot(const std::string &aName,
             const Point &aPosition) :
    name(aName),
    size(DefaultSize),
    position(aPosition),
    front(0, 0),
    speed(0.0),
    acting(false),
    driving(false),
    communicating(false) {
  std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
  attachSensor(laserSensor);
}

Robot::~Robot() {
  if (driving) {
    stopDriving();
  }
  if (acting) {
    stopActing();
  }
  if (communicating) {
    stopCommunicating();
  }
}

void Robot::setName(const std::string &aName,
                    bool aNotifyObservers /*= true*/) {
  name = aName;
  if (aNotifyObservers == true) {
    notifyObservers();
  }

}

void Robot::setSize(const Size &aSize,
                    bool aNotifyObservers /*= true*/) {
  size = aSize;
  if (aNotifyObservers == true) {
    notifyObservers();
  }
}

void Robot::setPosition(const Point &aPosition,
                        bool aNotifyObservers /*= true*/) {
  position = aPosition;
  if (aNotifyObservers == true) {
    notifyObservers();
  }
}

void Robot::setFront(const BoundedVector &aVector,
                     bool aNotifyObservers /*= true*/) {
  front = aVector;
  if (aNotifyObservers == true) {
    notifyObservers();
  }
}

void Robot::setSpeed(float aNewSpeed,
                     bool aNotifyObservers /*= true*/) {
  speed = aNewSpeed;
  if (aNotifyObservers == true) {
    notifyObservers();
  }
}

void Robot::startActing() {
  acting = true;
  std::thread newRobotThread([this] { startDriving(); });
  robotThread.swap(newRobotThread);
}

void Robot::stopActing() {
  acting = false;
  driving = false;
  robotThread.join();
}

void Robot::startDriving() {
  driving = true;

  // Calculate the fastest route towards the goal
  calculateRoute(goalToFind);

  drive();
}

void Robot::stopDriving() {
  driving = false;
}

void Robot::startCommunicating() {
  if (!communicating) {
    communicating = true;

    // Set the port to listen on
    std::string localPort = "12345";

    // Or set an alternative port supplied by commandline arguments
    if (Application::MainApplication::isArgGiven("-local_port")) {
      localPort = Application::MainApplication::getArg("-local_port").value;
    }

    // Attach it to the communication system
    Messaging::CommunicationService::getCommunicationService().runRequestHandler(toPtr<Robot>(), std::stoi(localPort));
  }
}

void Robot::stopCommunicating() {
  if (communicating) {
    communicating = false;

    // Set the port to listen on
    std::string localPort = "12345";

    // Or set an alternative port supplied by commandline arguments
    if (Application::MainApplication::isArgGiven("-local_port")) {
      localPort = Application::MainApplication::getArg("-local_port").value;
    }

    Messaging::Client c1ient("localhost", localPort, toPtr<Robot>());

    Messaging::Message message(1, "stop");

    c1ient.dispatchMessage(message);
  }
}

Region Robot::getRegion() const {
  Point translatedPoints[] = {getFrontRight(), getFrontLeft(), getBackLeft(), getBackRight()};
  return Region(4, translatedPoints);
}

bool Robot::intersects(const Region &aRegion) const {
  Region region = getRegion();
  region.Intersect(aRegion);
  return !region.IsEmpty();
}

Point Robot::getFrontLeft() const {
  // x and y are pointing to top left now
  int x = position.x - (size.x / 2);
  int y = position.y - (size.y / 2);

  Point originalFrontLeft(x, y);
  double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

  Point frontLeft
      ((originalFrontLeft.x - position.x) * std::cos(angle) - (originalFrontLeft.y - position.y) * std::sin(angle)
           + position.x, (originalFrontLeft.y - position.y) * std::cos(angle)
           + (originalFrontLeft.x - position.x) * std::sin(angle) + position.y);

  return frontLeft;
}

Point Robot::getFrontRight() const {
  // x and y are pointing to top left now
  int x = position.x - (size.x / 2);
  int y = position.y - (size.y / 2);

  Point originalFrontRight(x + size.x, y);
  double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

  Point frontRight
      ((originalFrontRight.x - position.x) * std::cos(angle) - (originalFrontRight.y - position.y) * std::sin(angle)
           + position.x, (originalFrontRight.y - position.y)
           * std::cos(angle) + (originalFrontRight.x - position.x) * std::sin(angle) + position.y);

  return frontRight;
}

Point Robot::getBackLeft() const {
  // x and y are pointing to top left now
  int x = position.x - (size.x / 2);
  int y = position.y - (size.y / 2);

  Point originalBackLeft(x, y + size.y);

  double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

  Point backLeft
      ((originalBackLeft.x - position.x) * std::cos(angle) - (originalBackLeft.y - position.y) * std::sin(angle)
           + position.x, (originalBackLeft.y - position.y) * std::cos(angle)
           + (originalBackLeft.x - position.x) * std::sin(angle) + position.y);

  return backLeft;

}

Point Robot::getBackRight() const {
  // x and y are pointing to top left now
  int x = position.x - (size.x / 2);
  int y = position.y - (size.y / 2);

  Point originalBackRight(x + size.x, y + size.y);

  double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

  Point backRight
      ((originalBackRight.x - position.x) * std::cos(angle) - (originalBackRight.y - position.y) * std::sin(angle)
           + position.x, (originalBackRight.y - position.y) * std::cos(angle)
           + (originalBackRight.x - position.x) * std::sin(angle) + position.y);

  return backRight;
}

void Robot::handleNotification() {
  //	std::unique_lock<std::recursive_mutex> lock(robotMutex);

  static int update = 0;
  if ((++update % 200) == 0) {
    notifyObservers();
  }
}

void Robot::handleRequest(Messaging::Message &aMessage) {
  switch (aMessage.getMessageType()) {
    case EchoRequest: {
      onEchoRequest(aMessage);
      break;
    }

    case synchRequest: {
      onSynchRequest(aMessage);
      break;
    }

    case startRobotRequest: {
      onStartRobotRequest(aMessage);
      break;
    }

    case posUpdateRequest: {
      onPosUpdateRequest(aMessage);
      break;
    }

    default: {
      Application::Logger::log(__PRETTY_FUNCTION__ + std::string(": default"));

      aMessage.setBody(" default  Goodbye cruel world!");
      break;
    }
  }
}

void Robot::onEchoRequest(Messaging::Message &aMessage) {
  Application::Logger::log(__PRETTY_FUNCTION__ + std::string(": EchoRequest"));

  aMessage.setMessageType(EchoResponse);
  aMessage.setBody(": case 1 " + aMessage.asString());
}

void Robot::onSynchRequest(Messaging::Message &aMessage) {
  Application::Logger::log("Attempting to Sync...");

  aMessage.setMessageType(synchResponse);

  Messaging::Message::MessageBody tempMessage = "";

  const std::vector<WallPtr> &wallsPtr = RobotWorld::getRobotWorld().getWalls();

  tempMessage += ";R ";
  tempMessage += std::to_string(getPosition().x);
  tempMessage += " ";
  tempMessage += std::to_string(getPosition().y);
  tempMessage += " ";
  tempMessage += std::to_string(RobotWorld::getRobotWorld().getGoal("Goal")->getPosition().x);
  tempMessage += " ";
  tempMessage += std::to_string(RobotWorld::getRobotWorld().getGoal("Goal")->getPosition().y);

  for (auto it : wallsPtr) {
    tempMessage += ";W ";
    tempMessage += std::to_string(it->getPoint1().x);
    tempMessage += " ";
    tempMessage += std::to_string(it->getPoint1().y);
    tempMessage += " ";
    tempMessage += std::to_string(it->getPoint2().x);
    tempMessage += " ";
    tempMessage += std::to_string(it->getPoint2().y);
  }

  aMessage.setBody(tempMessage);
}

void Robot::splitStringToStringVec(const std::string &aString, char aDelim, std::vector<std::string> &aVec) {
  std::stringstream stringStream(aString);
  std::string item;
  while (std::getline(stringStream, item, aDelim)) {
    if (item.length() > 0) {
      aVec.push_back(item);
    }
  }
}

void Robot::onStartRobotRequest(Messaging::Message &aMessage) {
  aMessage.setMessageType(startRobotResponse);
  aMessage.setBody("");
}

void Robot::onPosUpdateRequest(Messaging::Message &aMessage) {
  RobotPtr robob = RobotWorld::getRobotWorld().getRobot("Robob");
  if(robob){
    std::vector<std::string> body;
    splitStringToStringVec(aMessage.getBody(), ',', body);


    Point newPos(stoi(body.at(0)), stoi(body.at(1)));
    robob->setPosition(newPos);
    robob->notifyObservers();
    notifyObservers();
    std::cout << "Setting robot pos: " + std::to_string(robob->getPosition().x) + "," + std::to_string(robob->getPosition().y) << std::endl;
  }
}

void Robot::handleResponse(const Messaging::Message &aMessage) {
  switch (aMessage.getMessageType()) {
    case EchoResponse: {
      onEchoResponse(aMessage);
      break;
    }

    case synchResponse: {
      onSynchResponse(aMessage);
      break;
    }

    case startRobotResponse: {
      onStartRobotResponse(aMessage);
      break;
    }

    case posUpdateResponse: {
      onPosUpdateResponse(aMessage);
      break;
    }

    default: {
      Application::Logger::log(
          __PRETTY_FUNCTION__ + std::string(": default not implemented, ") + aMessage.asString());
      break;
    }
  }
}

void Robot::onEchoResponse(const Messaging::Message &aMessage) {
  Application::Logger::log(
      __PRETTY_FUNCTION__ + std::string(": case EchoResponse: not implemented, ") + aMessage.asString());
}

void Robot::onSynchResponse(const Messaging::Message &aMessage) {
  std::string body = aMessage.getBody();
  std::vector<std::string> objects;
  splitStringToStringVec(body, ';', objects);

  if (!objects.empty()) {
    for (std::string object : objects) {
      if (!object.empty()) {
        std::vector<std::string> arguments;
        Robot::splitStringToStringVec(object, ' ', arguments);

        switch (arguments.at(0).at(0)) {
          case 'R' : {
            RobotWorld::getRobotWorld().newRobot("Robob",
                                                 Point(std::stoi(arguments.at(1)), std::stoi(arguments.at(2))),
                                                 false);
            RobotPtr robob = RobotWorld::getRobotWorld().getRobot("Robob");

            RobotWorld::getRobotWorld().newGoal("Goat",
                                                Point(std::stoi(arguments.at(3)), std::stoi(arguments.at(4))),
                                                false);

            GoalPtr goalToFind = RobotWorld::getRobotWorld().getGoal("Goat");

            robob->setGoalToFind(goalToFind);
            break;
          }

          case 'W' :
            RobotWorld::getRobotWorld().newWall(Point(std::stoi(arguments.at(1)), std::stoi(arguments.at(2))),
                                                Point(std::stoi(arguments.at(3)), std::stoi(arguments.at(4))),
                                                false);
            break;
          default :
            Application::Logger::log(
                __PRETTY_FUNCTION__ + std::string(": unknown case syncResponse, ") + aMessage.asString());
            break;
        }
      }
    }
  }
  notifyObservers(); // notify observers for new objects
}

void Robot::onStartRobotResponse(const Messaging::Message &aMessage) {

  auto robot = RobotWorld::getRobotWorld().getRobot("Robot");

  robot->startActing();
  robot->notifyObservers();
  notifyObservers();
}

void Robot::onPosUpdateResponse(const Messaging::Message &aMessage) {
  std::cout << "Not setting robot pos: " + aMessage.getBody() << std::endl;
}

std::string Robot::asString() const {
  std::ostringstream os;

  os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

  return os.str();
}

std::string Robot::asDebugString() const {
  std::ostringstream os;

  os << "Robot:\n";
  os << AbstractAgent::asDebugString();
  os << "Robot " << name << " at (" << position.x << "," << position.y << ")\n";

  return os.str();
}

void Robot::drive() {
  try {
    // Turn on all sensors
    for (std::shared_ptr<AbstractSensor> sensor : sensors) {
      //sensor->setOn();
    }

    // Set the speed
    if (speed == 0.0) {
      speed = 10.0;
    }

    unsigned pathPoint = 0;

    // Keep the robot on screen
    while (position.x > 0 && position.x < 500 && position.y > 0 && position.y < 500 && pathPoint < path.size()) {
      // Destination vertex
      const PathAlgorithm::Vertex &vertex = path[pathPoint += speed];

      // Decide the front
      front = BoundedVector(vertex.asPoint(), position);

      // Move the robot
      position.x = vertex.x;
      position.y = vertex.y;

      if(client) {
        Messaging::Message message(posUpdateRequest);

        std::string body = std::to_string(position.x) + "," + std::to_string(position.y);

        message.setBody(body);
        getClient()->dispatchMessage(message);
      }

      // Check if the robot reached it's goal
      if (arrived(goal) || collision()) {
        Application::Logger::log(__PRETTY_FUNCTION__ + std::string(": arrived or collision"));
        notifyObservers();
        break;
      }

      notifyObservers();

      // Hold the thread for 100ms
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // this should be the last thing in the loop
      if (!driving) {
        return;
      }
    } // while

    for (std::shared_ptr<AbstractSensor> sensor : sensors) {
      //sensor->setOff();
    }
  }
  catch (std::exception &e) {
    std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
  }
  catch (...) {
    std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
  }
}

void Robot::calculateRoute(GoalPtr aGoal) {
  // Empty path
  path.clear();

  // If you have a goal
  if (aGoal) {
    // Turn off logging if not debugging AStar
    Application::Logger::setDisable();

    // Get the front
    front = BoundedVector(aGoal->getPosition(), position);

    // Handle notifications from astar
    handleNotificationsFor(astar);

    // Get the path from astar
    path = astar.search(position, aGoal->getPosition(), size);

    // Stop handling notifications
    stopHandlingNotificationsFor(astar);

    Application::Logger::setDisable(false);
  }
}

bool Robot::arrived(GoalPtr aGoal) {
  if (aGoal && intersects(aGoal->getRegion())) {
    return true;
  }
  return false;
}

bool Robot::collision() {
  // Get all corners of the robot
  Point frontLeft = getFrontLeft();
  Point frontRight = getFrontRight();
  Point backLeft = getBackLeft();
  Point backRight = getBackRight();

  // Get all walls from the world
  const std::vector<WallPtr> &walls = RobotWorld::getRobotWorld().getWalls();

  // For every wall
  for (WallPtr wall : walls) {
    // Call shape2dutils to check for collision
    if (Utils::Shape2DUtils::intersect(frontLeft, frontRight, wall->getPoint1(), wall->getPoint2()) ||
        Utils::Shape2DUtils::intersect(frontLeft, backLeft, wall->getPoint1(), wall->getPoint2()) ||
        Utils::Shape2DUtils::intersect(frontRight, backRight, wall->getPoint1(), wall->getPoint2())) {
      return true;
    }
  }
  return false;
}

void Robot::setupClient() {
  if (!client) {
    std::string remoteIpAdres = "localhost";
    std::string remotePort = "12345";

    if (Application::MainApplication::isArgGiven("-remote_ip")) {
      remoteIpAdres = Application::MainApplication::getArg("-remote_ip").value;
    }

    if (Application::MainApplication::isArgGiven("-remote_port")) {
      remotePort = Application::MainApplication::getArg("-remote_port").value;
    }

    client = std::make_shared<Messaging::Client>(remoteIpAdres, remotePort, (RobotPtr) this);
  }
}
} // namespace model
