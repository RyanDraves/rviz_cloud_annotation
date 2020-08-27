#include "uma_interactive_markers.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


UMAInteractiveMarkers::UMAInteractiveMarkers()
{
}

void UMAInteractiveMarkers::init(const boost::shared_ptr<interactive_markers::InteractiveMarkerServer> &server)
{
  m_server = server;
}

/*
 * For the make[CourseObject] function, the specs come from
 * https://robonation.org/app/uploads/sites/3/2019/10/RoboBoat-2020-Rules-and-Task-Description-V1.pdf
 *
 * Sphere hard coded dimensions are from buoys A-0 and A-2
 * https://www.polyformus.com/buoys/a-series
 *
 * Cylinder hard coded dimensions are from Sur-Mark II buoys
 * https://www.taylormadeproducts.com/products/mooring-and-safety-products/sur-mark-marker-buoys/
 *
 * Dock hard coded dimensions are from 40'' Docks and 40'' "Baby" Docks
 * https://www.ez-dock.com/products/dock-sections/
 */

visualization_msgs::Marker UMAInteractiveMarkers::makeSphere(
  const boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.279;
  marker.scale.y = 0.279;
  marker.scale.z = 0.279;
  marker.color.r = 0.8;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker UMAInteractiveMarkers::makeCylinder(
  const boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = 0.127;
  marker.scale.y = 0.127;
  marker.scale.z = 1.5494;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.8;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker UMAInteractiveMarkers::makeDock(
  const boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 1.524;
  marker.scale.y = 1.016;
  marker.scale.z = 0.381;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker UMAInteractiveMarkers::makeUnknown(
  const boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  // TODO(dravesr@umich.edu): Pass in a min/max point to appropriately size the bounding box
  // (Bounding boxes are used to enclose unknown detections)
  marker.scale.x = msg->scale * 0.45;
  marker.scale.y = msg->scale * 0.45;
  marker.scale.z = msg->scale * 0.45;
  marker.color.r = 0.6;
  marker.color.g = 0.0;
  marker.color.b = 0.6;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::InteractiveMarkerControl& UMAInteractiveMarkers::makeCenterControl(
  boost::shared_ptr<visualization_msgs::InteractiveMarker> &msg, OBJECTS object_type)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  switch (object_type)
  {
    case OBJECTS::SPHERE:
      control.markers.push_back(makeSphere(msg));
      break;
    case OBJECTS::CYLINDER:
      control.markers.push_back(makeCylinder(msg));
      break;
    case OBJECTS::DOCK:
      control.markers.push_back(makeDock(msg));
      break;
    case OBJECTS::UNKNOWN:
      control.markers.push_back(makeUnknown(msg));
      break;
    default:
      perror("");
      std::exit(1);
  }
  msg->controls.push_back(control);

  return msg->controls.back();
}

void UMAInteractiveMarkers::processFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // std::ostringstream s;
  // s << "Feedback from marker '" << feedback->marker_name << "' "
  //     << " / control '" << feedback->control_name << "'";

  // std::ostringstream mouse_point_ss;
  // if( feedback->mouse_point_valid )
  // {
  //   mouse_point_ss << " at " << feedback->mouse_point.x
  //                  << ", " << feedback->mouse_point.y
  //                  << ", " << feedback->mouse_point.z
  //                  << " in frame " << feedback->header.frame_id;
  // }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      // ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      // ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      {
        // ROS_INFO_STREAM( s.str() << ": pose changed"
        //     << "\nposition = "
        //     << feedback->pose.position.x
        //     << ", " << feedback->pose.position.y
        //     << ", " << feedback->pose.position.z
        //     << "\norientation = "
        //     << feedback->pose.orientation.w
        //     << ", " << feedback->pose.orientation.x
        //     << ", " << feedback->pose.orientation.y
        //     << ", " << feedback->pose.orientation.z
        //     << "\nframe: " << feedback->header.frame_id
        //     << " time: " << feedback->header.stamp.sec << "sec, "
        //     << feedback->header.stamp.nsec << " nsec" );
        m_markers[m_marker_names_map[feedback->marker_name]]->pose = feedback->pose;
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      // ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      // ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  m_server->applyChanges();
}

void UMAInteractiveMarkers::make6DofMarker(size_t label, const geometry_msgs::Pose &position,
  OBJECTS object_type, bool apply)
{
  m_markers[label].reset(new visualization_msgs::InteractiveMarker());
  auto int_marker = m_markers[label];

  int_marker->header.frame_id = "base_link";
  int_marker->pose = position;
  int_marker->scale = 1;

  int_marker->name = "uma_marker_" + std::to_string(label);
  m_marker_names_map.insert({ int_marker->name, label });
  switch (object_type)
  {
    case OBJECTS::SPHERE:
      int_marker->description = "Sphere ";
      break;
    case OBJECTS::CYLINDER:
      int_marker->description = "Cylinder ";
      break;
    case OBJECTS::DOCK:
      int_marker->description = "Dock ";
      break;
    case OBJECTS::UNKNOWN:
      int_marker->description = "Unknown ";
      break;
    default:
      perror("");
      std::exit(1);
  }
  int_marker->description += std::to_string(label);

  // Make center marker (sphere/cylinder/dock/unknown)
  makeCenterControl(int_marker, object_type);
  int_marker->controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  bool is_sphere = object_type == OBJECTS::SPHERE;

  // Create X rotation and translation
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  if (!is_sphere)
  {
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker->controls.push_back(control);
  }
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker->controls.push_back(control);

  // Create Y rotation and translation
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  if (!is_sphere)
  {
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker->controls.push_back(control);
  }
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker->controls.push_back(control);

  // Create Z rotation and translation
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  if (!is_sphere)
  {
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker->controls.push_back(control);
  }
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker->controls.push_back(control);

  m_server->insert(*int_marker);
  m_server->setCallback(int_marker->name, boost::bind(&UMAInteractiveMarkers::processFeedback, this, _1));

  if (apply)
    m_server->applyChanges();
}

void UMAInteractiveMarkers::removeMarker(size_t label)
{
  if (m_markers.find(label) == m_markers.end()) return;

  m_server->erase(m_markers[label]->name);
  m_markers.erase(label);
}

void UMAInteractiveMarkers::removeAllMarkers()
{
  for (const auto &marker : m_markers)
  {
    m_server->erase(marker.second->name);
  }
  m_markers.clear();
}

boost::shared_ptr<visualization_msgs::InteractiveMarker> UMAInteractiveMarkers::getMarker(size_t label) const
{
  auto iter = m_markers.find(label);
  if (iter == m_markers.end()) return boost::shared_ptr<visualization_msgs::InteractiveMarker>();

  return iter->second;
}
