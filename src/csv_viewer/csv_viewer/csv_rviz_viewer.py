import csv
from typing import List, Tuple
from rclpy.node import Node
import rclpy

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

def _norm(s: str) -> str:
    return ''.join(ch for ch in s.lower() if ch.isalnum())

def _detect_delimiter(sample: str) -> str:
    # Try to sniff common delimiters; fallback to comma
    try:
        dialect = csv.Sniffer().sniff(sample, delimiters=",;\t ")
        return dialect.delimiter
    except Exception:
        return ','

def _read_points(csv_path: str, z_offset: float = 0.0, downsample_step: int = 1) -> List[Tuple[float, float, float]]:
    # Read small sample to sniff
    with open(csv_path, 'r', encoding='utf-8-sig') as f:
        sample = f.read(4096)
    delim = _detect_delimiter(sample)

    rows: List[List[str]] = []
    with open(csv_path, 'r', encoding='utf-8-sig', newline='') as f:
        reader = csv.reader(f, delimiter=delim)
        for row in reader:
            if len(row) == 0:
                continue
            rows.append(row)

    if not rows:
        return []

    # Header detection
    has_header = False
    try:
        has_header = csv.Sniffer().has_header(sample)
    except Exception:
        # Heuristic: if first row has any non-float -> header
        try:
            [float(x) for x in rows[0][:3]]
            has_header = False
        except Exception:
            has_header = True

    header = None
    data_start = 0
    if has_header:
        header = rows[0]
        data_start = 1
    else:
        header = [f'col{i}' for i in range(len(rows[0]))]

    # Map columns to x,y,z
    lower = [_norm(h) for h in header]
    # Preferred keys
    candidates = {
        'x': ['x', 'xe', 'xem', 'east', 'e', 'xm'],
        'y': ['y', 'yn', 'ynm', 'north', 'n', 'ym'],
        'z': ['z', 'za', 'zam', 'alt', 'a', 'height', 'zm'],
    }
    def find_idx(keys):
        for k in keys:
            if k in lower:
                return lower.index(k)
        # try startswith match (e.g., 'xem')
        for i, l in enumerate(lower):
            for k in keys:
                if l.startswith(k):
                    return i
        return None

    xi = find_idx(candidates['x'])
    yi = find_idx(candidates['y'])
    zi = find_idx(candidates['z'])

    # Fallback to first three columns
    if xi is None or yi is None:
        xi, yi = 0, 1
    if zi is None:
        zi = 2 if len(header) > 2 else None

    pts: List[Tuple[float, float, float]] = []
    for i, row in enumerate(rows[data_start:]):
        if downsample_step > 1 and (i % downsample_step != 0):
            continue
        try:
            x = float(row[xi])
            y = float(row[yi])
            if zi is not None and zi < len(row):
                z = float(row[zi]) + z_offset
            else:
                z = 0.0 + z_offset
            pts.append((x, y, z))
        except Exception:
            # skip malformed line
            continue

    return pts

class CsvViewerNode(Node):
    def __init__(self):
        super().__init__('csv_viewer_node')

        # Parameters
        self.declare_parameter('csv_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)          # Hz
        self.declare_parameter('z_offset', 0.0)               # meters
        self.declare_parameter('downsample_step', 1)          # keep every Nth point
        self.declare_parameter('marker_scale', 0.5)           # meters
        self.declare_parameter('line_width', 0.15)            # meters
        self.declare_parameter('publish_points', True)
        self.declare_parameter('publish_line', True)
        self.declare_parameter('publish_path', True)

        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value
        self.downsample_step = int(self.get_parameter('downsample_step').get_parameter_value().integer_value)
        self.marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value
        self.do_points = self.get_parameter('publish_points').get_parameter_value().bool_value
        self.do_line = self.get_parameter('publish_line').get_parameter_value().bool_value
        self.do_path = self.get_parameter('publish_path').get_parameter_value().bool_value

        if not self.csv_path:
            self.get_logger().error('csv_path parameter is empty. Please set it via launch/params.')
            self.points = []
        else:
            self.points = _read_points(self.csv_path, self.z_offset, max(1, self.downsample_step))
            self.get_logger().info(f'Loaded {len(self.points)} points from "{self.csv_path}"')

        # Publishers
        if self.do_path:
            self.path_pub = self.create_publisher(Path, 'csv_path', 10)
        else:
            self.path_pub = None

        self.marker_pub = self.create_publisher(MarkerArray, 'csv_markers', 10)

        # Timer
        period = 1.0 / max(1e-3, self.rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

    def _on_timer(self):
        now = self.get_clock().now().to_msg()

        # Publish Path
        if self.do_path and self.path_pub and self.points:
            path = Path()
            path.header = Header(stamp=now, frame_id=self.frame_id)
            for (x, y, z) in self.points:
                ps = PoseStamped()
                ps.header = Header(stamp=now, frame_id=self.frame_id)
                ps.pose = Pose()
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.position.z = z
                # orientation left as zeros (identity)
                path.poses.append(ps)
            self.path_pub.publish(path)

        # Publish Markers (points + line strip)
        ma = MarkerArray()
        idx = 0

        if self.do_points and self.points:
            m_points = Marker()
            m_points.header = Header(stamp=now, frame_id=self.frame_id)
            m_points.ns = 'csv_points'
            m_points.id = idx; idx += 1
            m_points.type = Marker.SPHERE_LIST
            m_points.action = Marker.ADD
            m_points.scale.x = self.marker_scale
            m_points.scale.y = self.marker_scale
            m_points.scale.z = self.marker_scale
            m_points.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.9)
            m_points.points = [Point(x=x, y=y, z=z) for (x, y, z) in self.points]
            ma.markers.append(m_points)

        if self.do_line and self.points:
            m_line = Marker()
            m_line.header = Header(stamp=now, frame_id=self.frame_id)
            m_line.ns = 'csv_line'
            m_line.id = idx; idx += 1
            m_line.type = Marker.LINE_STRIP
            m_line.action = Marker.ADD
            m_line.scale.x = self.line_width
            m_line.color = ColorRGBA(r=1.0, g=0.3, b=0.1, a=0.9)
            m_line.points = [Point(x=x, y=y, z=z) for (x, y, z) in self.points]
            ma.markers.append(m_line)

        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = CsvViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
