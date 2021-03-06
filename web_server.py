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

import io
import os
import os.path as osp
import logging
import numpy as np
import flask
import requests
from requests.exceptions import RequestException
from PIL import Image

## Try importing vtr specific modules and variables, or set them to None
try:
  import rclpy

  from vtr_messages.msg import GraphComponent
  import vtr_mission_planning
  from . import graph_pb2
  from . import utils

  # ROS2 node
  rclpy.init()
  node = rclpy.create_node("web_server")
  node.get_logger().info('Created node - web_server')
except:
  graph_pb2 = None
  utils = None
  GraphComponent = None
  vtr_mission_planning = None
  node = None

## Config the web server
# web server address and port
UI_ADDRESS = '0.0.0.0'
UI_PORT = 5200

logger = logging.getLogger('WebServer')

app = flask.Flask(__name__,
                  static_folder="vtr-ui/build",
                  template_folder="vtr-ui/build",
                  static_url_path="")
app.config['DEBUG'] = False
app.config['CACHE'] = True
app.config['CACHE_PATH'] = osp.abspath(osp.join(osp.dirname(__file__), 'cache'))
app.config['PROTO_PATH'] = osp.abspath(osp.dirname(__file__))
app.secret_key = 'asecretekey'

app.logger.setLevel(logging.ERROR)
logging.getLogger('werkzeug').setLevel(logging.ERROR)


@app.route("/")
def main_page():
  return flask.redirect("index.html")


@app.after_request
def set_response_headers(response):
  # TODO (yuchen) need to verify if this function is indeed effective
  response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
  response.headers['Pragma'] = 'no-cache'
  response.headers['Expires'] = '0'
  return response


@app.route('/proto/<path:proto_file>')
def proto_files(proto_file):
  return flask.send_from_directory(app.config['PROTO_PATH'], proto_file)


@app.route('/cache/tile/<s>/<x>/<y>/<z>')
def tile_cache(s, x, y, z):
  """Load google satellite map"""
  fname = x + '.jpg'
  fdir = osp.join(app.config['CACHE_PATH'], 'tile', z, y)
  fpath = osp.join(fdir, fname)

  if app.config['CACHE'] and osp.isfile(fpath):
    logger.debug("Using cached tile {%s,%s,%s}", x, y, z)
    return flask.send_from_directory(fdir, fname, max_age=60 * 60 * 24 * 30)

  headers = {
      'Accept': 'image/webp,image/*,*/*;q=0.8',
      'User-Agent': flask.request.user_agent.string
  }
  # url = 'https://khms' + s + '.googleapis.com/kh?v=199&hl=en-GB&x=' + x + '&y=' + y + '&z=' + z
  # Google Map service
  # url = 'http://mt1.google.com/vt/lyrs=y&x=' + x + '&y=' + y + '&z=' + z
  # Open Street Map (mapnik) service
  url = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'.format(
      z=z, y=y, x=x)

  try:
    res = requests.get(url, headers=headers, verify=False)
  except RequestException as e:
    logger.error('Error loading tile {%s,%s,%s}: %s', x, y, z, e)
    flask.abort(500)

  if res.ok:
    try:
      sio = io.BytesIO(res.content)
      if app.config['CACHE']:
        logger.debug("Caching tile: {%s,%s,%s}", x, y, z)
        os.makedirs(fdir, exist_ok=True)
        img = Image.open(sio)
        img.save(fpath)
      else:
        logger.debug("Proxying tile: {%s,%s,%s}", x, y, z)

      sio.seek(0)
      return flask.send_file(sio,
                             mimetype='image/jpeg',
                             max_age=60 * 60 * 24 * 30)
    except Exception as e:
      logger.error('Something went really sideways on tile {%s,%s,%s}: %s', x,
                   y, z, e)
      flask.abort(500)
  else:
    logger.warning("Tile {%s,%s,%s} did not exist on server", x, y, z)

  flask.abort(404)


##### VTR specific calls #####


@app.route('/api/map/<seq>')
def get_map(seq):
  """API endpoint to get the full map"""
  logger.info("Fetching graph with sequence number: {}".format(seq))
  graph = utils.get_graph(node, seq)

  proto_graph = graph_pb2.Graph()

  proto_graph.seq = graph.seq
  proto_graph.stamp = graph.stamp.sec + graph.stamp.nanosec * 1e-9
  proto_graph.root = graph.root_id

  if graph.seq < 0:
    return proto_graph.SerializeToString()

  if not graph.projected:
    raise RuntimeError(
        "Received an unprojected graph... What do I do with this?")

  for v in graph.vertices:
    vertex = proto_graph.vertices.add()
    vertex.id = v.id
    vertex.lat = v.t_projected.y
    vertex.lng = v.t_projected.x
    vertex.theta = v.t_projected.theta
    vertex.neighbours.extend(v.neighbours)

  for c in graph.components:
    if c.type == GraphComponent.PATH:
      component = proto_graph.paths.add()
    elif c.type == GraphComponent.CYCLE:
      component = proto_graph.cycles.add()
    else:
      raise RuntimeError(
          "Encountered unknown graph component type in UI Server")

    component.vertices.extend(c.vertices)

  proto_graph.branch.vertices.extend(graph.active_branch)
  proto_graph.junctions.extend(graph.junctions)

  for p in graph.pins:
    pin = proto_graph.pins.add()
    pin.id = p.id
    pin.latLng.lat = p.lat
    pin.latLng.lng = p.lng
    pin.weight = p.weight

  gps_coords = np.array(
      [[v.t_projected.y, v.t_projected.x] for v in graph.vertices])

  if len(gps_coords > 0):
    mn = list(np.min(np.array(gps_coords), axis=0))
    mx = list(np.max(np.array(gps_coords), axis=0))
  elif len(graph.center) == 2:
    mn = list(graph.center)
    mx = list(graph.center)
  else:
    mn = [43.781596, -79.467298]
    mx = [43.782806, -79.464608]

  proto_graph.min_bnd.lat = mn[0]
  proto_graph.min_bnd.lng = mn[1]
  proto_graph.max_bnd.lat = mx[0]
  proto_graph.max_bnd.lng = mx[1]
  proto_graph.map_center.lat = (mn[0] + mx[0]) / 2
  proto_graph.map_center.lng = (mn[1] + mx[1]) / 2

  return proto_graph.SerializeToString()


@app.route('/api/init')
def init_state():
  """API endpoint to get the initial state of the robot/localization chain"""
  rclient = vtr_mission_planning.remote_client()

  return flask.jsonify(seq=rclient.path_seq,
                       path=rclient.path,
                       vertex=rclient.trunk_vertex,
                       lngLatTheta=rclient.trunk_lng_lat_theta,
                       tfLeafTrunk=rclient.t_leaf_trunk,
                       covLeafTrunk=rclient.cov_leaf_trunk,
                       targetVertex=rclient.target_vertex,
                       targetLngLatTheta=rclient.target_lng_lat_theta,
                       tfLeafTarget=rclient.t_leaf_target,
                       covLeafTarget=rclient.cov_leaf_target)


@app.route('/api/goal/all')
def get_goals():
  """API endpoint to get all goals"""
  rclient = vtr_mission_planning.remote_client()
  return flask.jsonify(goals=rclient.goals, status=rclient.status)


def main():
  logger.setLevel(logging.INFO)
  hd = logging.StreamHandler()
  fm = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  hd.setFormatter(fm)
  logger.addHandler(hd)

  logger.info("Launching the web server.")

  # TODO: Server runs on all interfaces. Can we assume a trusted network?
  app.run(threaded=True, host=UI_ADDRESS, port=UI_PORT, use_reloader=False)


if __name__ == '__main__':
  main()