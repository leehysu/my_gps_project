import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import os

def visualize_gps_path(file_path):
    """
    CSV 파일의 GPS 경로를 direction 값에 따라 다른 색상으로 시각화하고,
    입력된 CSV 파일과 동일한 위치에 같은 이름의 PNG 파일로 저장합니다.
    """
    # f9r_to_csv.py에서 저장한 CSV 파일의 전체 경로를 입력하세요.
    # 예: "/home/user/data/my_gps_data.csv"
    # ---------------------------------------------
    
    try:
        # CSV 파일을 pandas DataFrame으로 읽어옵니다.
        df = pd.read_csv(file_path)

        # 필요한 열 이름
        x_col = 'X(E/m)'
        y_col = 'Y(N/m)'
        dir_col = 'direction'

        # 데이터의 열이 숫자가 아닐 경우를 대비해 숫자 형태로 변환합니다.
        df[x_col] = pd.to_numeric(df[x_col], errors='coerce')
        df[y_col] = pd.to_numeric(df[y_col], errors='coerce')
        df[dir_col] = pd.to_numeric(df[dir_col], errors='coerce')
        
        # 결측값 제거
        df.dropna(subset=[x_col, y_col, dir_col], inplace=True)

        # 그래프 생성
        plt.figure(figsize=(12, 12))
        
        # 색상 맵 정의
        color_map = {1: 'red', 0: 'black', -1: 'blue'}
        
        # direction 값이 변경되는 지점을 찾아 segment로 나누어 플로팅
        df['color_change'] = df[dir_col].ne(df[dir_col].shift())
        change_indices = df.index[df['color_change']].tolist()
        
        if 0 not in change_indices:
            change_indices.insert(0, 0)
        
        if df.index[-1] + 1 not in change_indices:
             change_indices.append(df.index[-1] + 1)

        # 각 세그먼트 플로팅
        for i in range(len(change_indices) - 1):
            start_idx = change_indices[i]
            end_idx = change_indices[i+1]
            
            segment_df = df.iloc[start_idx:end_idx]
            
            if start_idx > 0:
                segment_df = pd.concat([df.iloc[[start_idx - 1]], segment_df])
                
            direction = segment_df[dir_col].iloc[0]
            color = color_map.get(direction, 'gray')
            
            plt.plot(segment_df[x_col], segment_df[y_col], marker='.', linestyle='-', markersize=2, color=color, label=f'Direction {int(direction)}' if f'Direction {int(direction)}' not in plt.gca().get_legend_handles_labels()[1] else "")

        # 그래프 제목 및 축 레이블 설정
        plt.title('GPS Path Visualization by Direction', fontsize=16)
        plt.xlabel('Easting (meters)', fontsize=12)
        plt.ylabel('Northing (meters)', fontsize=12)
        
        # 범례 추가 (중복 제거)
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys())

        # 축의 가로 세로 비율을 동일하게 설정
        plt.axis('equal')
        plt.grid(True)
        
        # 축 서식 설정
        plt.gca().xaxis.set_major_formatter(ticker.FormatStrFormatter('%.0f'))
        plt.gca().yaxis.set_major_formatter(ticker.FormatStrFormatter('%.0f'))
        plt.xticks(rotation=45)

        # --- PNG 파일 저장 경로 및 이름 설정 (수정된 부분) ---
        # 1. 입력 파일의 디렉토리 경로를 가져옵니다.
        output_directory = os.path.dirname(file_path)
        # 2. 입력 파일의 이름에서 확장자만 .png로 변경합니다.
        base_name = os.path.basename(file_path)
        file_name_without_ext = os.path.splitext(base_name)[0]
        output_filename = file_name_without_ext + '.png'
        # 3. 디렉토리 경로와 파일 이름을 합쳐 최종 저장 경로를 만듭니다.
        output_image_path = os.path.join(output_directory, output_filename)
        # ---------------------------------------------------
        
        plt.savefig(output_image_path, dpi=300)
        print(f"Image saved to: {output_image_path}")
        # plt.show() # 로컬에서 실행할 때 그래프를 바로 보려면 주석 해제

    except FileNotFoundError:
        print(f"Error: The file was not found at {file_path}")
    except KeyError as e:
        print(f"Error: A required column is missing in the CSV file: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    # --- [사용자 설정] 시각화할 CSV 파일 경로 ---
    # f9r_to_csv.py에서 저장한 CSV 파일의 전체 경로를 입력하세요.
    csv_file_path = "/home/hannibal/Mandol_ws/data/processed/이전대회 GPS 데이터.csv"
    # ---------------------------------------------
    visualize_gps_path(csv_file_path)