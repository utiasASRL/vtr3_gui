import logging
import threading

import rclpy
from geometry_msgs.msg import Pose2D
from vtr_messages.msg import GraphPin
from vtr_messages.srv import GraphRelaxation, GraphCalibration, GraphPinning

log = logging.getLogger()
log.setLevel(logging.INFO)

# A thread lock for ROS to avoid synchronization issues
ros_rlock = threading.RLock()


def ros_service_request(node, path, mtype, request):
  ros_service = node.create_client(mtype, path)
  while not ros_service.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')
  with ros_rlock:  # (yuchen) isn't this equivalent to call(request)?
    response = ros_service.call_async(request)
    rclpy.spin_until_future_complete(node, response)

  return response.result()


def get_graph(node, seq):
  """Get the relaxed pose graph from the map server"""
  log.info("Fetching graph with sequence number: {}".format(seq))

  request = GraphRelaxation.Request()
  request.seq = int(seq)
  request.update_graph = False  # TODO not used?
  request.project = True  # TODO always True?
  return ros_service_request(node, "relaxed_graph", GraphRelaxation, request)


def move_graph(node, x, y, theta, scale):
  """Update lat lng of the pose graph shown on map"""
  logging.info("Calling update_calib service.")

  request = GraphCalibration.Request()
  request.t_delta = Pose2D(x=x, y=y, theta=theta)
  request.scale_delta = scale
  return ros_service_request(node, "update_calib", GraphCalibration, request)


def pin_graph(node, pins):
  """Add vertex to latlng correspondence pins"""
  logging.info("Calling pin_graph service.")

  request = GraphPinning.Request()
  for pin in pins:
    pin_msg = GraphPin()
    pin_msg.id = int(pin["id"])
    pin_msg.lat = float(pin["latLng"]["lat"])
    pin_msg.lng = float(pin["latLng"]["lng"])
    pin_msg.weight = float(pin["weight"])
    request.pins.append(pin_msg)
  return ros_service_request(node, "pin_graph", GraphPinning, request)