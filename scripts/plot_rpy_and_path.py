
import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import utm
import glob

def quat_to_euler_xyz(qx, qy, qz, qw):
    """Quaternion(x,y,z,w) -> roll, pitch, yaw [rad] (XYZ, ROS 기준)"""
    sinr_cosp = 2 * (qw*qx + qy*qz)
    cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw*qy - qz*qx)
    pitch = np.pi/2 * np.sign(sinp) if abs(sinp) >= 1 else np.arcsin(sinp)
    siny_cosp = 2 * (qw*qz + qx*qy)
    cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def latlon_df_to_utm(df, lat_col="latitude", lon_col="longitude"):
    e_list, n_list, zn_list, zl_list = [], [], [], []
    for lat, lon in zip(df[lat_col].values, df[lon_col].values):
        if pd.isna(lat) or pd.isna(lon):
            e_list.append(np.nan); n_list.append(np.nan); zn_list.append(None); zl_list.append(None)
            continue
        e, n, zn, zl = utm.from_latlon(lat, lon)
        e_list.append(e); n_list.append(n); zn_list.append(zn); zl_list.append(zl)
    out = df.copy()
    out["easting"] = e_list
    out["northing"] = n_list
    out["utm_zone_number"] = zn_list
    out["utm_zone_letter"] = zl_list
    return out

def topic_to_csv_name(topic: str) -> str:
    return (topic.strip('/').replace('/', '_') or 'root') + '.csv'

def auto_pick_csv(csv_dir: Path, need_cols):
    """csv_dir에서 첫 번째로 need_cols를 모두 포함하는 CSV를 골라줌."""
    for path in sorted(csv_dir.glob("*.csv")):
        try:
            df = pd.read_csv(path, nrows=10)
            if set(need_cols).issubset(df.columns):
                return path
        except Exception:
            continue
    return None

def main(csv_dir: Path, imu_topic: str, gps_topic: str):
    csv_dir = Path(csv_dir)
    plots_dir = csv_dir.parent / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    # ---------- IMU: RPY ----------
    imu_csv = None
    if imu_topic.lower() == "auto":
        imu_csv = auto_pick_csv(csv_dir, ["ori_x","ori_y","ori_z","ori_w","stamp"])
    else:
        imu_csv = csv_dir / topic_to_csv_name(imu_topic)

    if imu_csv and imu_csv.exists():
        imu = pd.read_csv(imu_csv)
        if set(['ori_x','ori_y','ori_z','ori_w']).issubset(imu.columns):
            r, p, y = [], [], []
            for qx,qy,qz,qw in zip(imu['ori_x'], imu['ori_y'], imu['ori_z'], imu['ori_w']):
                rr, pp, yy = quat_to_euler_xyz(qx,qy,qz,qw)
                r.append(rr); p.append(pp); y.append(yy)
            imu['roll'], imu['pitch'], imu['yaw'] = r, p, y
            imu.to_csv(imu_csv, index=False)

            plt.figure()
            plt.plot(imu['stamp'], imu['roll'], label='roll')
            plt.plot(imu['stamp'], imu['pitch'], label='pitch')
            plt.plot(imu['stamp'], imu['yaw'], label='yaw')
            plt.title('IMU RPY vs Time (rad)')
            plt.xlabel('stamp [s]'); plt.ylabel('rad')
            plt.legend(); plt.tight_layout()
            plt.savefig(plots_dir / "imu_rpy.png"); plt.close()
            print(f"Saved: {plots_dir / 'imu_rpy.png'}")
        else:
            print(f"[WARN] IMU CSV에 ori_* 칼럼 없음: {imu_csv}")
    else:
        print("[INFO] IMU CSV를 찾지 못했습니다 (auto 탐색 실패 or 파일 없음).")

    # ---------- GPS: UTM & Path ----------
    gps_csv = None
    if gps_topic.lower() == "auto":
        gps_csv = auto_pick_csv(csv_dir, ["latitude","longitude","stamp"])
    else:
        gps_csv = csv_dir / topic_to_csv_name(gps_topic)

    if gps_csv and gps_csv.exists():
        gps = pd.read_csv(gps_csv)
        if set(['latitude','longitude']).issubset(gps.columns):
            gps_utm = latlon_df_to_utm(gps, "latitude", "longitude")
            gps_utm.to_csv(gps_csv, index=False)

            if set(['easting','northing']).issubset(gps_utm.columns):
                plt.figure()
                plt.plot(gps_utm['easting'], gps_utm['northing'])
                plt.title('Path in UTM'); plt.xlabel('Easting [m]'); plt.ylabel('Northing [m]')
                plt.axis('equal'); plt.tight_layout()
                plt.savefig(plots_dir / "path_utm.png"); plt.close()
                print(f"Saved: {plots_dir / 'path_utm.png'}")
        else:
            print(f"[WARN] GPS CSV에 latitude/longitude 칼럼 없음: {gps_csv}")
    else:
        print("[INFO] GPS CSV를 찾지 못했습니다 (auto 탐색 실패 or 파일 없음).")

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="CSV에서 IMU RPY 및 GPS UTM 경로 시각화")
    ap.add_argument("--csv_dir", required=True, help="CSV 폴더 (예: outputs/csv)")
    ap.add_argument("--imu_topic", default="auto", help="IMU 토픽명 또는 'auto'")
    ap.add_argument("--gps_topic", default="auto", help="GPS 토픽명 또는 'auto'")
    args = ap.parse_args()
    main(Path(args.csv_dir), args.imu_topic, args.gps_topic)
