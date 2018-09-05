#ifndef WAYPOINT_HPP_
#define WAYPOINT_HPP_

#include "Config.hpp"
#include "ModelObject.hpp"
#include "Point.hpp"
#include "Size.hpp"
#include "Region.hpp"

namespace model {

class WayPoint;
typedef std::shared_ptr<WayPoint> WayPointPtr;

class WayPoint : public ModelObject {
 public:
  WayPoint();
  WayPoint(const std::string &aName);
  WayPoint(const std::string &aName,
           const Point &aPosition);
  virtual ~WayPoint();


  void setName(const std::string &aName, bool aNotifyObservers = true);
  void setSize(const Size &aSize, bool aNotifyObservers = true);
  void setPosition(const Point &aPosition, bool aNotifyObservers = true);

  Region getRegion() const;
  Size getSize() const;
  std::string getName() const {
    return name;
  }
  Point getPosition() const {
    return position;
  }

  bool intersects(const Region &aRegion) const;

  virtual std::string asString() const;
  virtual std::string asDebugString() const;

 private:
  std::string name;
  Size size;
  Point position;
};
} // namespace model
#endif // WAYPOINT_HPP_
