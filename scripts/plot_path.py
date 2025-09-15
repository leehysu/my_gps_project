#!/usr/bin/env python3
import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import utm

def find_gps_csv(csv_dir: Path):
    """csv_dir 안에서 'fix' 들어간 파일을 우선 선택"""
    candidates = sorted(csv_dir.glob("*.csv"))
    for p in candidates:
        if "fix" in p.name.lower():   # 이름에 fix 들어가면 바로 선택
            return p
    # fallback: 그냥 lat/lon 있는 파일 선택
    for p in candidates:
        try:
            df = pd.read_csv(p, nrows=5)
            cols = {c.lower() for c in df.columns}
            if {"lat","lon"}.issubset(cols) or {"latitude","longitude"}.issubset(cols):
                return p
        except Exception:
            continue
    return None


def load_gps(path: Path):
    df = pd.read_csv(path)
    # 컬럼명 통일
    cols = {c.lower(): c for c in df.columns}
    if "lat" in cols and "lon" in cols:
        lat_col, lon_col = cols["lat"], cols["lon"]
    elif "latitude" in cols and "longitude" in cols:
        lat_col, lon_col = cols["latitude"], cols["longitude"]
    else:
        raise SystemExit(f"[ERROR] {path} 에 latitude/longitude(or lat/lon) 컬럼이 없습니다.")
    # 시간 컬럼 정리
    tcol = None
    for c in ("stamp","time","timestamp","sec"):
        if c in cols:
            tcol = cols[c]; break
    if tcol is not None:
        df = df.sort_values(tcol)

    # 0 값/NaN 제거
    df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=[lat_col, lon_col])
    df = df[(df[lat_col] != 0) & (df[lon_col] != 0)]
    return df.rename(columns={lat_col:"lat", lon_col:"lon"})

def to_utm(df: pd.DataFrame):
    e_list, n_list = [], []
    for lat, lon in zip(df["lat"], df["lon"]):
        e, n, *_ = utm.from_latlon(lat, lon)
        e_list.append(e); n_list.append(n)
    out = df.copy()
    out["E"], out["N"] = e_list, n_list
    return out

def main(args):
    outdir = Path(args.out); outdir.mkdir(parents=True, exist_ok=True)

    # 입력 선택
    if args.gps_csv:
        gps_path = Path(args.gps_csv)
    else:
        gps_path = find_gps_csv(Path(args.csv_dir))
        if gps_path is None:
            raise SystemExit("[ERROR] lat/lon 컬럼 가진 CSV를 찾지 못했습니다. --gps_csv 로 직접 지정하세요.")
    print(f"[INFO] 사용 CSV: {gps_path}")

    df = load_gps(gps_path)
    print(f"[INFO] 유효 위치 샘플 수: {len(df)}")

    # 위경도 경로
    plt.figure()
    plt.plot(df["lon"], df["lat"], linewidth=1)
    plt.title("GPS Path (WGS84 lon/lat)")
    plt.xlabel("Longitude"); plt.ylabel("Latitude")
    plt.axis("equal"); plt.tight_layout()
    ll_png = outdir / "path_ll.png"
    plt.savefig(ll_png); plt.close()
    print(f"[SAVE] {ll_png}")

    # UTM 경로
    if args.utm:
        du = to_utm(df)
        plt.figure()
        plt.plot(du["E"], du["N"], linewidth=1)
        plt.title("GPS Path (UTM)")
        plt.xlabel("Easting [m]"); plt.ylabel("Northing [m]")
        plt.axis("equal"); plt.tight_layout()
        utm_png = outdir / "path_utm.png"
        plt.savefig(utm_png); plt.close()
        print(f"[SAVE] {utm_png}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="GPS 경로 시각화 (lon/lat 및 UTM)")
    ap.add_argument("--csv_dir", default="outputs/csv", help="GPS CSV가 들어있는 폴더")
    ap.add_argument("--gps_csv", help="특정 GPS CSV 파일 경로를 직접 지정 (선택)")
    ap.add_argument("--out", default="outputs/plots", help="이미지 저장 폴더")
    ap.add_argument("--utm", action="store_true", help="UTM 경로도 함께 저장")
    args = ap.parse_args()
    main(args)
