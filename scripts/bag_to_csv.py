import argparse
from pathlib import Path
import pandas as pd
import numpy as np

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, register_types


# 메시지를 dict로 변환
def extract_row(msg, timestamp_ns):
    row = {}
    row['stamp'] = timestamp_ns * 1e-9

    # imu orientation
    if hasattr(msg, 'orientation'):
        row['ori_x'] = msg.orientation.x
        row['ori_y'] = msg.orientation.y
        row['ori_z'] = msg.orientation.z
        row['ori_w'] = msg.orientation.w

    # gps 위치
    if hasattr(msg, 'latitude'):
        row['lat'] = msg.latitude
        row['lon'] = msg.longitude
        row['alt'] = msg.altitude

    return row


# 파일 이름 안전하게 변환
def sanitize_topic(topic: str) -> str:
    return topic.strip('/').replace('/', '_')


def main(bag_dir: Path, out_dir: Path):
    if not (bag_dir.exists() and (bag_dir / 'metadata.yaml').exists()):
        raise SystemExit(f"[ERROR] '{bag_dir}' 가 유효한 rosbag2 폴더가 아닙니다 (metadata.yaml 필요).")

    out_dir.mkdir(parents=True, exist_ok=True)
    topic_counts = {}

    with AnyReader([bag_dir]) as reader:
        # 1) 필요한 토픽만 선택
        include_topics = ['/gps/fix', '/imu/data']
        selected = [c for c in reader.connections if c.topic in include_topics]
        if not selected:
            # 이름이 다르면 키워드로 추려보기
            selected = [c for c in reader.connections
                        if any(k in c.topic.lower() for k in ('imu', 'gps', 'fix'))]

        # 2) 타입 등록 (모르는 타입은 건너뜀)
        for c in selected:
            if c.msgdef:
                try:
                    register_types(get_types_from_msg(c.msgdef, c.msgtype))
                except Exception:
                    pass

        # 3) 메시지 읽기
        buckets = {}
        for c, ts, raw in reader.messages(connections=selected):
            try:
                msg = reader.deserialize(raw, c.msgtype)
            except Exception:
                continue

            row = extract_row(msg, ts)
            if row:
                buckets.setdefault(c.topic, []).append(row)
                topic_counts[c.topic] = topic_counts.get(c.topic, 0) + 1

    # 4) CSV 저장
    for topic, rows in buckets.items():
        if not rows:
            continue
        df = pd.DataFrame(rows).sort_values('stamp')
        fname = sanitize_topic(topic) + '.csv'
        out_path = out_dir / fname
        df.to_csv(out_path, index=False)
        print(f"Saved: {out_path} ({len(df)} rows) topic='{topic}'")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="ROS2 bag → per-topic CSV")
    ap.add_argument("--bag", required=True, help="rosbag2 폴더 경로 (metadata.yaml이 있는 폴더)")
    ap.add_argument("--out", required=True, help="CSV 출력 폴더")
    args = ap.parse_args()

    main(Path(args.bag), Path(args.out))
