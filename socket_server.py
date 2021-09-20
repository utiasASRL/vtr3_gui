#!/usr/bin/env python3

# Copyright 2021, Autonomous Space Robotics Lab (ASRL)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import flask
import flask_socketio

# Try importing vtr specific modules and variables, or set them to None
try:
  import rclpy

  from vtr_messages.action import Mission
  from vtr_messages.srv import MissionCmd
  from vtr_messages.msg import MissionStatus
  import vtr_mission_planning
  from . import graph_pb2
  from . import utils

  # ROS2 node
  rclpy.init()
  node = rclpy.create_node("socket_server")
  node.get_logger().info('Created node - socket_server')
except:
  graph_pb2 = None
  utils = None
  Mission = None
  MissionCmd = None
  MissionStatus = None
  vtr_mission_planning = None
  node = None

## Config the socket io server
# socket io server address and port
SOCKET_ADDRESS = 'localhost'
SOCKET_PORT = 5201

logger = logging.getLogger('SocketServer')

app = flask.Flask(__name__)
app.config['DEBUG'] = False
app.secret_key = 'asecretekey'

app.logger.setLevel(logging.ERROR)
logging.getLogger('werkzeug').setLevel(logging.ERROR)

socketio = flask_socketio.SocketIO(app,
                                   logger=False,
                                   engineio_logger=False,
                                   ping_interval=1,
                                   ping_timeout=2,
                                   cors_allowed_origins="*")


@app.route('/')
def main():
  return "This is a socket-only API server."


@socketio.on('connect')
def on_connect():
  logger.info('Client connected!')


@socketio.on('disconnect')
def on_disconnect():
  logger.info('Client disconnected!')


##### VTR specific calls #####


@socketio.on('goal/add')
def add_goal(json):
  """Handles SocketIO request to add a goal"""
  logger.info('Client requests to add a goal!')
  goal_str = json.get('type', None)
  if goal_str is None:
    return False, u"Goal type is a mandatory field"

  goal_type = {
      'IDLE': Mission.Goal.IDLE,
      'TEACH': Mission.Goal.TEACH,
      'REPEAT': Mission.Goal.REPEAT,
      'MERGE': Mission.Goal.MERGE,
      'LOCALIZE': Mission.Goal.LOCALIZE
  }.get(goal_str.upper(), None)

  if goal_type is None:
    return False, u"Invalid goal type"

  try:
    pause_before = float(json.get('pauseBefore', 0.0))
    pause_after = float(json.get('pauseAfter', 0.0))
  except Exception:
    return False, u"Non-numeric pause duration received"

  try:
    path = [int(x) for x in json.get('path', [])]
  except Exception:
    return False, u"Non-integer vertex id supplied in path"

  if goal_type == Mission.Goal.REPEAT and path == []:
    return False, u"Empty path supplied for repeat goal"

  rclient = vtr_mission_planning.remote_client()
  goal_id = rclient.add_goal(goal_type, path, pause_before, pause_after,
                             json.get('vertex', 2**64 - 1))
  logger.info("New goal: %s", goal_str)

  return True, goal_id


@socketio.on('goal/cancel')
def cancel_goal(json):
  """Handles SocketIO request to cancel a goal"""
  logger.info('Client requests to cancel a goal!')
  goal_id = json.get('id', 'all')
  logger.info("Cancelling goal %s", goal_id)

  if goal_id == 'all':
    result = vtr_mission_planning.remote_client().cancel_all()
  else:
    result = vtr_mission_planning.remote_client().cancel_goal(goal_id)

  return result


@socketio.on('goal/cancel/all')
def cancel_all():
  """Handles SocketIO request to cancel all goals"""
  logger.info('Client requests to cancel all goals!')
  return vtr_mission_planning.remote_client().cancel_all()


@socketio.on('goal/move')
def move_goal(json):
  """Handles SocketIO request to re-arrange goals"""
  logger.info('Client requests to re-arrange goals!')


@socketio.on('pause')
def pause(json):
  """Handle socketIO messages to pause the mission server"""
  logger.info('Client requests to pause the mission server!')
  paused = json.get('paused', None)
  if paused is None:
    return False, u"The pause state is a mandatory parameter"

  vtr_mission_planning.remote_client().set_pause(paused)
  return True


@socketio.on('graph/offset')
def move_graph(json):
  """Handles SocketIO request to update the graph offset"""
  logger.info('Client requests to update the graph offset!')
  utils.move_graph(node, float(json["x"]), float(json["y"]),
                   float(json["theta"]), json["scale"])


@socketio.on('graph/pins')
def pin_graph(json):
  """Handles SocketIO request to update the graph pins"""
  logger.info('Client requests to update the graph pins!')
  utils.pin_graph(node, json["pins"])


@socketio.on('graph/cmd')
def graph_cmd(req):
  """Handles SocketIO request of a graph command"""
  logger.info('Client sends a request of graph command!')

  ros_service = node.create_client(MissionCmd, "mission_cmd")
  while not ros_service.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')

  action = {
      'add_run': MissionCmd.Request.ADD_RUN,  # TODO unused, remove this
      'localize': MissionCmd.Request.LOCALIZE,
      'merge': MissionCmd.Request.START_MERGE,
      'closure': MissionCmd.Request.CONFIRM_MERGE,
      'continue_teach': MissionCmd.Request.CONTINUE_TEACH,
      'loc_search': MissionCmd.Request.LOC_SEARCH,
  }.get(req['action'], None)

  if action is None:
    raise RuntimeError("Invalid request string")

  request = MissionCmd.Request()
  request.action = action
  if 'vertex' in req.keys():
    request.vertex = req['vertex']
  if 'path' in req.keys():
    request.path = req['path']

  response = ros_service.call_async(request)
  rclpy.spin_until_future_complete(node, response)

  return response.result()


@socketio.on('message')
def handle_notifications(json):
  """Re-broadcasts incoming notifications from the mission client, every message
  from VTR2 goes to here.
  """
  # Match vtr_planning.mission_client.Notification
  callbacks = {
      'NewGoal': broadcast_new,
      'Error': broadcast_error,
      'Cancel': broadcast_cancel,
      'Complete': broadcast_success,
      'Started': broadcast_started,
      'Feedback': broadcast_feedback,
      'StatusChange': broadcast_status,
      'RobotChange': broadcast_robot,
      'PathChange': broadcast_path,
      'GraphChange': broadcast_graph,
  }

  try:
    callbacks[json['type']](*json['args'], **json['kwargs'])
  except KeyError as e:
    logger.error('A client disconnected ungracefully: {}.'.format(str(e)))


def broadcast_new(goal):
  """Broadcasts socketIO messages to all clients on new goal addition
  Args:
    goal: Dictionary representing the fields of the added goal
  """
  logger.info('Broadcast new goal: %s.', str(goal))
  socketio.emit(u"goal/new", goal, broadcast=True)


def broadcast_error(goal_id, goal_status):
  """Broadcasts socketIO messages to all clients on errors
  Args:
    goal_id The id of the failed goal
    goal_status Status enum representing the failure reason
  """
  logger.info('Broadcast error.')
  msg = "An unknown error occurred; check the console for more information"
  logger.warning(
      "An unexpected goal status (%d) occurred while broadcasting errors",
      goal_status)

  socketio.emit(u"goal/error", {'id': goal_id, 'msg': msg}, broadcast=True)


def broadcast_cancel(goal_id):
  """Broadcasts socketIO messages to all clients on goal cancellation
  Args:
    goal_id: The id of the cancelled goal
  """
  logger.info('Broadcast cancel.')
  socketio.emit(u"goal/cancelled", {'id': goal_id}, broadcast=True)


def broadcast_success(goal_id):
  """Broadcasts socketIO messages to all clients on goal completion
  Args:
    goal_id: The id of the finished goal
  """
  logger.info('Broadcast success.')
  socketio.emit(u"goal/success", {'id': goal_id}, broadcast=True)


def broadcast_started(goal_id):
  """Broadcasts socketIO messages to all clients when a goal becomes active
  Args:
    goal_id: The id of the goal that was started
  """
  logger.info('Broadcast started.')
  socketio.emit(u"goal/started", {'id': goal_id}, broadcast=True)


def broadcast_feedback(goal_id, feedback):
  """Broadcasts socketIO messages to all clients on new feedback for any goal
  Args:
    goal_id: The id of the goal receiving feedback
    feedback: Dictionary representing the feedback message
  """
  logger.info('Broadcast feedback.')
  data = feedback
  data['id'] = goal_id
  socketio.emit(u"goal/feedback", data, broadcast=True)


def broadcast_status(status, queue):
  """Broadcasts socketIO messages to all clients on status change in the mission
  server status
  Args:
    status: The current state of the mission server {EMPTY|PAUSED|PROCESSING|PENDING_PAUSE}
    queue: List of all current goal ids, ordered by execution priority
  """
  logger.info('Broadcast status.')
  text = {
      MissionStatus.PROCESSING: "PROCESSING",
      MissionStatus.PAUSED: "PAUSED",
      MissionStatus.PENDING_PAUSE: "PENDING_PAUSE",
      MissionStatus.EMPTY: "EMPTY"
  }.get(status, None)

  if text is None:
    text = "UNKNOWN"
  socketio.emit(u"status", {
      'state': str(text),
      'queue': [str(q) for q in queue]
  },
                broadcast=True)


def broadcast_robot(
    seq,
    vertex,
    lng_lat_theta,
    tf_leaf_trunk,
    cov_leaf_trunk,
    target_vertex,
    target_lng_lat_theta,
    tf_leaf_target,
    cov_leaf_target,
):
  """Broadcasts socketIO messages to all clients on position change of the robot
  Args:
    vertex: Current closest vertex ID to the robot
    tf_leaf_trunk: Transform from robot to trunk
    cov_leaf_trunk: Covariance (diagonal) of the robot to trunk transform
    target_vertex: The target vertex we are trying to merge into
    tf_leaf_target: Transform from robot to target
    cov_leaf_target: Covariance of the robot to target transform
  """
  logger.info('Broadcast robot')
  status = graph_pb2.RobotStatus()
  # status.seq = seq
  status.vertex = vertex
  (
      status.lng_lat_theta.x,
      status.lng_lat_theta.y,
      status.lng_lat_theta.theta,
  ) = lng_lat_theta
  (
      status.tf_leaf_trunk.x,
      status.tf_leaf_trunk.y,
      status.tf_leaf_trunk.theta,
  ) = tf_leaf_trunk
  for val in cov_leaf_trunk:
    status.cov_leaf_trunk.append(val)

  if 0 <= target_vertex < 2**64 - 1:
    status.target_vertex = target_vertex
    (
        status.target_lng_lat_theta.x,
        status.target_lng_lat_theta.y,
        status.target_lng_lat_theta.theta,
    ) = target_lng_lat_theta
    (
        status.tf_leaf_target.x,
        status.tf_leaf_target.y,
        status.tf_leaf_target.theta,
    ) = tf_leaf_target

    for val in cov_leaf_target:
      status.cov_leaf_target.append(val)

  socketio.emit(u"robot/loc", bytes(status.SerializeToString()), broadcast=True)


def broadcast_path(path):
  """Broadcasts socketIO messages to all clients on position change of the robot
  Args:
    path: List of vertices representing the current localization chain
  """
  logger.info("Broadcasting new path %s", str(path))
  socketio.emit(u"robot/path", {'path': path}, broadcast=True)


def broadcast_graph(msg):
  """Broadcasts socketIO messages to all clients on updates to the graph
  Args:
    msg: Update message from ROS
  """
  logger.info('Broadcast graph')
  update = graph_pb2.GraphUpdate()
  update.seq = msg['seq']
  update.stamp = msg['stamp']
  update.invalidate = msg['invalidate']

  for v in msg['vertices']:
    vertex = update.vertices.add()
    vertex.id = v['id']
    vertex.lat = v['T_projected'][1]
    vertex.lng = v['T_projected'][0]
    vertex.theta = v['T_projected'][2]
    vertex.neighbours.extend(v['neighbours'])

  socketio.emit(u"graph/update",
                bytes(update.SerializeToString()),
                broadcast=True)


def main():
  logger.setLevel(logging.INFO)
  hd = logging.StreamHandler()
  fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  hd.setFormatter(fm)
  logger.addHandler(hd)

  logger.info("Launching the socket server.")

  # TODO: Server runs on all interfaces.  Can we assume a trusted network?
  socketio.run(app, host=SOCKET_ADDRESS, port=SOCKET_PORT, use_reloader=False)


if __name__ == '__main__':
  main()