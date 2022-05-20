# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: graph.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='graph.proto',
  package='',
  syntax='proto2',
  serialized_pb=_b('\n\x0bgraph.proto\"!\n\tComponent\x12\x14\n\x08vertices\x18\x01 \x03(\x04\x42\x02\x10\x01\"\"\n\x06LatLng\x12\x0b\n\x03lat\x18\x01 \x02(\x01\x12\x0b\n\x03lng\x18\x02 \x02(\x01\"?\n\x08GraphPin\x12\n\n\x02id\x18\x01 \x02(\x04\x12\x17\n\x06latLng\x18\x02 \x02(\x0b\x32\x07.LatLng\x12\x0e\n\x06weight\x18\x03 \x02(\x01\"U\n\x06Vertex\x12\n\n\x02id\x18\x01 \x02(\x04\x12\x0b\n\x03lat\x18\x02 \x02(\x01\x12\x0b\n\x03lng\x18\x03 \x02(\x01\x12\r\n\x05theta\x18\x04 \x02(\x02\x12\x16\n\nneighbours\x18\x05 \x03(\x04\x42\x02\x10\x01\"\x9c\x02\n\x05Graph\x12\x0b\n\x03seq\x18\x01 \x02(\x03\x12\r\n\x05stamp\x18\x02 \x02(\x01\x12\x0c\n\x04root\x18\x03 \x02(\x04\x12\x1b\n\nmap_center\x18\x04 \x01(\x0b\x32\x07.LatLng\x12\x18\n\x07max_bnd\x18\x05 \x01(\x0b\x32\x07.LatLng\x12\x18\n\x07min_bnd\x18\x06 \x01(\x0b\x32\x07.LatLng\x12\x19\n\x05paths\x18\x07 \x03(\x0b\x32\n.Component\x12\x1a\n\x06\x63ycles\x18\x08 \x03(\x0b\x32\n.Component\x12\x1a\n\x06\x62ranch\x18\t \x01(\x0b\x32\n.Component\x12\x11\n\tjunctions\x18\n \x03(\x04\x12\x17\n\x04pins\x18\x0b \x03(\x0b\x32\t.GraphPin\x12\x19\n\x08vertices\x18\x0c \x03(\x0b\x32\x07.Vertex\"_\n\x0bGraphUpdate\x12\x0b\n\x03seq\x18\x01 \x02(\x04\x12\r\n\x05stamp\x18\x02 \x02(\x01\x12\x19\n\ninvalidate\x18\x03 \x02(\x08:\x05\x66\x61lse\x12\x19\n\x08vertices\x18\x04 \x03(\x0b\x32\x07.Vertex\";\n\x0cGraphOverlay\x12\x0b\n\x03ids\x18\x01 \x03(\x04\x12\x0e\n\x06to_ids\x18\x02 \x03(\x04\x12\x0e\n\x06values\x18\x03 \x03(\x01\"-\n\x06Pose2D\x12\t\n\x01x\x18\x01 \x02(\x01\x12\t\n\x01y\x18\x02 \x02(\x01\x12\r\n\x05theta\x18\x03 \x02(\x01\"\xfa\x01\n\x0bRobotStatus\x12\x0b\n\x03seq\x18\x01 \x01(\x04\x12\x0e\n\x06vertex\x18\x02 \x02(\x04\x12\x1e\n\rlng_lat_theta\x18\x03 \x02(\x0b\x32\x07.Pose2D\x12\x1e\n\rtf_leaf_trunk\x18\x04 \x02(\x0b\x32\x07.Pose2D\x12\x16\n\x0e\x63ov_leaf_trunk\x18\x05 \x03(\x01\x12\x15\n\rtarget_vertex\x18\x06 \x01(\x04\x12%\n\x14target_lng_lat_theta\x18\x07 \x01(\x0b\x32\x07.Pose2D\x12\x1f\n\x0etf_leaf_target\x18\x08 \x01(\x0b\x32\x07.Pose2D\x12\x17\n\x0f\x63ov_leaf_target\x18\t \x03(\x01\".\n\x0cSafetyStatus\x12\x0e\n\x06signal\x18\x01 \x02(\t\x12\x0e\n\x06\x61\x63tion\x18\x02 \x02(\r\"-\n\rMissionStatus\x12\r\n\x05state\x18\x01 \x02(\t\x12\r\n\x05queue\x18\x02 \x03(\t\"K\n\x0cGoalFeedback\x12\n\n\x02id\x18\x01 \x02(\t\x12\x17\n\x0fpercentComplete\x18\x02 \x02(\x01\x12\x16\n\x07waiting\x18\x03 \x02(\x08:\x05\x66\x61lse')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_COMPONENT = _descriptor.Descriptor(
  name='Component',
  full_name='Component',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='vertices', full_name='Component.vertices', index=0,
      number=1, type=4, cpp_type=4, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=15,
  serialized_end=48,
)


_LATLNG = _descriptor.Descriptor(
  name='LatLng',
  full_name='LatLng',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lat', full_name='LatLng.lat', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lng', full_name='LatLng.lng', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=50,
  serialized_end=84,
)


_GRAPHPIN = _descriptor.Descriptor(
  name='GraphPin',
  full_name='GraphPin',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='GraphPin.id', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='latLng', full_name='GraphPin.latLng', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='weight', full_name='GraphPin.weight', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=86,
  serialized_end=149,
)


_VERTEX = _descriptor.Descriptor(
  name='Vertex',
  full_name='Vertex',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='Vertex.id', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lat', full_name='Vertex.lat', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lng', full_name='Vertex.lng', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='theta', full_name='Vertex.theta', index=3,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='neighbours', full_name='Vertex.neighbours', index=4,
      number=5, type=4, cpp_type=4, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=151,
  serialized_end=236,
)


_GRAPH = _descriptor.Descriptor(
  name='Graph',
  full_name='Graph',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='seq', full_name='Graph.seq', index=0,
      number=1, type=3, cpp_type=2, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stamp', full_name='Graph.stamp', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='root', full_name='Graph.root', index=2,
      number=3, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='map_center', full_name='Graph.map_center', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_bnd', full_name='Graph.max_bnd', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='min_bnd', full_name='Graph.min_bnd', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='paths', full_name='Graph.paths', index=6,
      number=7, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cycles', full_name='Graph.cycles', index=7,
      number=8, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='branch', full_name='Graph.branch', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='junctions', full_name='Graph.junctions', index=9,
      number=10, type=4, cpp_type=4, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pins', full_name='Graph.pins', index=10,
      number=11, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vertices', full_name='Graph.vertices', index=11,
      number=12, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=239,
  serialized_end=523,
)


_GRAPHUPDATE = _descriptor.Descriptor(
  name='GraphUpdate',
  full_name='GraphUpdate',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='seq', full_name='GraphUpdate.seq', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stamp', full_name='GraphUpdate.stamp', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='invalidate', full_name='GraphUpdate.invalidate', index=2,
      number=3, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vertices', full_name='GraphUpdate.vertices', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=525,
  serialized_end=620,
)


_GRAPHOVERLAY = _descriptor.Descriptor(
  name='GraphOverlay',
  full_name='GraphOverlay',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ids', full_name='GraphOverlay.ids', index=0,
      number=1, type=4, cpp_type=4, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='to_ids', full_name='GraphOverlay.to_ids', index=1,
      number=2, type=4, cpp_type=4, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='values', full_name='GraphOverlay.values', index=2,
      number=3, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=622,
  serialized_end=681,
)


_POSE2D = _descriptor.Descriptor(
  name='Pose2D',
  full_name='Pose2D',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='Pose2D.x', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='Pose2D.y', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='theta', full_name='Pose2D.theta', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=683,
  serialized_end=728,
)


_ROBOTSTATUS = _descriptor.Descriptor(
  name='RobotStatus',
  full_name='RobotStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='seq', full_name='RobotStatus.seq', index=0,
      number=1, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vertex', full_name='RobotStatus.vertex', index=1,
      number=2, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lng_lat_theta', full_name='RobotStatus.lng_lat_theta', index=2,
      number=3, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='tf_leaf_trunk', full_name='RobotStatus.tf_leaf_trunk', index=3,
      number=4, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cov_leaf_trunk', full_name='RobotStatus.cov_leaf_trunk', index=4,
      number=5, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='target_vertex', full_name='RobotStatus.target_vertex', index=5,
      number=6, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='target_lng_lat_theta', full_name='RobotStatus.target_lng_lat_theta', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='tf_leaf_target', full_name='RobotStatus.tf_leaf_target', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cov_leaf_target', full_name='RobotStatus.cov_leaf_target', index=8,
      number=9, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=731,
  serialized_end=981,
)


_SAFETYSTATUS = _descriptor.Descriptor(
  name='SafetyStatus',
  full_name='SafetyStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='signal', full_name='SafetyStatus.signal', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='action', full_name='SafetyStatus.action', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=983,
  serialized_end=1029,
)


_MISSIONSTATUS = _descriptor.Descriptor(
  name='MissionStatus',
  full_name='MissionStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='state', full_name='MissionStatus.state', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='queue', full_name='MissionStatus.queue', index=1,
      number=2, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1031,
  serialized_end=1076,
)


_GOALFEEDBACK = _descriptor.Descriptor(
  name='GoalFeedback',
  full_name='GoalFeedback',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='GoalFeedback.id', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='percentComplete', full_name='GoalFeedback.percentComplete', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='waiting', full_name='GoalFeedback.waiting', index=2,
      number=3, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1078,
  serialized_end=1153,
)

_GRAPHPIN.fields_by_name['latLng'].message_type = _LATLNG
_GRAPH.fields_by_name['map_center'].message_type = _LATLNG
_GRAPH.fields_by_name['max_bnd'].message_type = _LATLNG
_GRAPH.fields_by_name['min_bnd'].message_type = _LATLNG
_GRAPH.fields_by_name['paths'].message_type = _COMPONENT
_GRAPH.fields_by_name['cycles'].message_type = _COMPONENT
_GRAPH.fields_by_name['branch'].message_type = _COMPONENT
_GRAPH.fields_by_name['pins'].message_type = _GRAPHPIN
_GRAPH.fields_by_name['vertices'].message_type = _VERTEX
_GRAPHUPDATE.fields_by_name['vertices'].message_type = _VERTEX
_ROBOTSTATUS.fields_by_name['lng_lat_theta'].message_type = _POSE2D
_ROBOTSTATUS.fields_by_name['tf_leaf_trunk'].message_type = _POSE2D
_ROBOTSTATUS.fields_by_name['target_lng_lat_theta'].message_type = _POSE2D
_ROBOTSTATUS.fields_by_name['tf_leaf_target'].message_type = _POSE2D
DESCRIPTOR.message_types_by_name['Component'] = _COMPONENT
DESCRIPTOR.message_types_by_name['LatLng'] = _LATLNG
DESCRIPTOR.message_types_by_name['GraphPin'] = _GRAPHPIN
DESCRIPTOR.message_types_by_name['Vertex'] = _VERTEX
DESCRIPTOR.message_types_by_name['Graph'] = _GRAPH
DESCRIPTOR.message_types_by_name['GraphUpdate'] = _GRAPHUPDATE
DESCRIPTOR.message_types_by_name['GraphOverlay'] = _GRAPHOVERLAY
DESCRIPTOR.message_types_by_name['Pose2D'] = _POSE2D
DESCRIPTOR.message_types_by_name['RobotStatus'] = _ROBOTSTATUS
DESCRIPTOR.message_types_by_name['SafetyStatus'] = _SAFETYSTATUS
DESCRIPTOR.message_types_by_name['MissionStatus'] = _MISSIONSTATUS
DESCRIPTOR.message_types_by_name['GoalFeedback'] = _GOALFEEDBACK

Component = _reflection.GeneratedProtocolMessageType('Component', (_message.Message,), dict(
  DESCRIPTOR = _COMPONENT,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:Component)
  ))
_sym_db.RegisterMessage(Component)

LatLng = _reflection.GeneratedProtocolMessageType('LatLng', (_message.Message,), dict(
  DESCRIPTOR = _LATLNG,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:LatLng)
  ))
_sym_db.RegisterMessage(LatLng)

GraphPin = _reflection.GeneratedProtocolMessageType('GraphPin', (_message.Message,), dict(
  DESCRIPTOR = _GRAPHPIN,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:GraphPin)
  ))
_sym_db.RegisterMessage(GraphPin)

Vertex = _reflection.GeneratedProtocolMessageType('Vertex', (_message.Message,), dict(
  DESCRIPTOR = _VERTEX,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:Vertex)
  ))
_sym_db.RegisterMessage(Vertex)

Graph = _reflection.GeneratedProtocolMessageType('Graph', (_message.Message,), dict(
  DESCRIPTOR = _GRAPH,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:Graph)
  ))
_sym_db.RegisterMessage(Graph)

GraphUpdate = _reflection.GeneratedProtocolMessageType('GraphUpdate', (_message.Message,), dict(
  DESCRIPTOR = _GRAPHUPDATE,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:GraphUpdate)
  ))
_sym_db.RegisterMessage(GraphUpdate)

GraphOverlay = _reflection.GeneratedProtocolMessageType('GraphOverlay', (_message.Message,), dict(
  DESCRIPTOR = _GRAPHOVERLAY,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:GraphOverlay)
  ))
_sym_db.RegisterMessage(GraphOverlay)

Pose2D = _reflection.GeneratedProtocolMessageType('Pose2D', (_message.Message,), dict(
  DESCRIPTOR = _POSE2D,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:Pose2D)
  ))
_sym_db.RegisterMessage(Pose2D)

RobotStatus = _reflection.GeneratedProtocolMessageType('RobotStatus', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTSTATUS,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:RobotStatus)
  ))
_sym_db.RegisterMessage(RobotStatus)

SafetyStatus = _reflection.GeneratedProtocolMessageType('SafetyStatus', (_message.Message,), dict(
  DESCRIPTOR = _SAFETYSTATUS,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:SafetyStatus)
  ))
_sym_db.RegisterMessage(SafetyStatus)

MissionStatus = _reflection.GeneratedProtocolMessageType('MissionStatus', (_message.Message,), dict(
  DESCRIPTOR = _MISSIONSTATUS,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:MissionStatus)
  ))
_sym_db.RegisterMessage(MissionStatus)

GoalFeedback = _reflection.GeneratedProtocolMessageType('GoalFeedback', (_message.Message,), dict(
  DESCRIPTOR = _GOALFEEDBACK,
  __module__ = 'graph_pb2'
  # @@protoc_insertion_point(class_scope:GoalFeedback)
  ))
_sym_db.RegisterMessage(GoalFeedback)


_COMPONENT.fields_by_name['vertices'].has_options = True
_COMPONENT.fields_by_name['vertices']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))
_VERTEX.fields_by_name['neighbours'].has_options = True
_VERTEX.fields_by_name['neighbours']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))
# @@protoc_insertion_point(module_scope)
