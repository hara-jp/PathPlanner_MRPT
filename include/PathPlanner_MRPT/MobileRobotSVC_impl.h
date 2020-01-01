// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.h
 * @brief Service implementation header of MobileRobot.idl
 *
 */

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"
#include "InterfaceDataTypesSkel.h"

#include "MobileRobotSkel.h"

#ifndef MOBILEROBOTSVC_IMPL_H
#define MOBILEROBOTSVC_IMPL_H

#include <mrpt/slam/COccupancyGridMap2D.h>
#ifdef WIN32
#include <mrpt/nav/planners/CPathPlanningCircularRobot.h>
#else
#include <mrpt/nav/planners/PlannerSimple2D.h>
#endif
#include <mrpt/poses/CPose2D.h>

//#include "PathPlanner_MRPT.h"
using namespace mrpt;
//using namespace mrpt::slam;
using namespace std;

class PathPlanner_MRPT;
/*!
 * @class RTC_PathPlannerSVC_impl
 * Example class implementing IDL interface RTC::PathPlanner
 */
class RTC_PathPlannerSVC_impl
 : public virtual POA_RTC::PathPlanner,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RTC_PathPlannerSVC_impl();

   PathPlanner_MRPT *m_pRTC;

   mrpt::poses::CPose2D start;
   mrpt::poses::CPose2D goal;
   float RADIUS;
   float PATH_LENGTH;

 public:
  void setRTC(PathPlanner_MRPT* pRTC) { m_pRTC = pRTC; }
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /*!
   * @brief standard constructor
   */
   RTC_PathPlannerSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RTC_PathPlannerSVC_impl();

   // attributes and operations
   RTC::RETURN_VALUE planPath(const RTC::PathPlanParameter& param, RTC::Path2D_out outPath);

    mrpt::poses::CPose2D getStart(){return start;}
    mrpt::poses::CPose2D getGoal(){return goal;}

    void setConfig(float radius, float pathLength){
            RADIUS = radius;
            PATH_LENGTH = pathLength;
    }
    float getRadius(){return RADIUS;}
    float getPathLength(){return PATH_LENGTH;}

    void setStart(const RTC::Pose2D & tp, const RTC::OGMap & map);
    void setGoal(const RTC::Pose2D & tp, const RTC::OGMap & map);
    void OGMapToCOccupancyGridMap(RTC::OGMap ogmap, mrpt::maps::COccupancyGridMap2D *gridmap);


};


#endif // MOBILEROBOTSVC_IMPL_H


