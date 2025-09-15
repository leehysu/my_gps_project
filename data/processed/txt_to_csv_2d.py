import pandas as pd
import os

def convert_txt_to_csv(txt_file_path, csv_file_path):
    """
    3열짜리 TXT 파일을 읽어 앞의 2열(동쪽, 북쪽)만 CSV 파일로 저장합니다.
    """
    try:
        # TXT 파일을 공백 기준으로 DataFrame으로 읽어오기
        # usecols=[0, 1] : 첫 번째와 두 번째 열만 읽어오도록 지정
        # names : 읽어온 열들의 이름을 지정
        df = pd.read_csv(
            txt_file_path, 
            sep='\s+', 
            header=None, 
            usecols=[0, 1], 
            names=['X(E/m)', 'Y(N/m)']
        )
        
        # DataFrame을 CSV 파일로 저장
        # index=False는 DataFrame의 인덱스를 CSV에 포함하지 않도록 함
        df.to_csv(csv_file_path, index=False)
        
        print(f"'{txt_file_path}' 파일이 '{csv_file_path}' 파일로 성공적으로 변환되었습니다.")
        
    except FileNotFoundError:
        print(f"오류: '{txt_file_path}' 파일을 찾을 수 없습니다.")
    except Exception as e:
        print(f"오류가 발생했습니다: {e}")

if __name__ == '__main__':
    # 변환할 TXT 파일 경로
    txt_path = '/home/hannibal/Mandol_ws/data/preprocessed/이전대회 GPS 데이터.txt'
    
    # 저장될 CSV 파일 경로 (입력 파일과 동일한 이름으로 .csv 확장자만 변경)
    csv_path = os.path.splitext(txt_path)[0] + '.csv'
    
    # 함수 호출
    convert_txt_to_csv(txt_path, csv_path)