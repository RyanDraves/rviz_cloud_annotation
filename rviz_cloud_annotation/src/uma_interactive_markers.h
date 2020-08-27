#ifndef UMA_INTERACTIVE_MARKERS_H
#define UMA_INTERACTIVE_MARKERS_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2/LinearMath/Vector3.h>

#include <math.h>
#include <map>


class UMAInteractiveMarkers
{
public:
    enum OBJECTS { SPHERE, CYLINDER, DOCK, UNKNOWN };

    UMAInteractiveMarkers();
    void init(const boost::shared_ptr<interactive_markers::InteractiveMarkerServer> &server);
    void make6DofMarker(size_t label, const geometry_msgs::Pose &position, OBJECTS object_type, bool apply);
    void removeMarker(size_t label);
    void removeAllMarkers();
    boost::shared_ptr<visualization_msgs::InteractiveMarker> getMarker(size_t label) const;
private:
    visualization_msgs::Marker makeSphere(const boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg);
    visualization_msgs::Marker makeCylinder(const boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg);
    visualization_msgs::Marker makeDock(const boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg);
    visualization_msgs::Marker makeUnknown(const boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg);
    visualization_msgs::InteractiveMarkerControl& makeCenterControl(
        boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg, OBJECTS object_type);
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> m_server;
    std::map<size_t, boost::shared_ptr<visualization_msgs::InteractiveMarker> > m_markers;
    std::map<std::string, size_t> m_marker_names_map;
};

#endif  /* UMA_INTERACTIVE_MARKERS_H */
