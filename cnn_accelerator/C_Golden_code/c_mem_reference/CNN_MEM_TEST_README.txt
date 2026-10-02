PC에서 C로 이미지 MEM 추론 결과 확인 (FPGA/Vitis 불필요)

구성 파일:
  cnn_int8.c           기존 사용자가 제공한 CPU 참조 추론 코드
  cnn_int8.h           해당 코드의 타입/함수 선언
  cnn_model_data.c/h   ROM model.mem에서 가중치와 bias 읽기
  cnn_mem_test.c       64x64 RRGGBB 이미지 MEM 입력 + 결과 출력
  model.mem            ROM + UART RTL 시험과 동일한 weight/bias 파일

1. dataset_4_rtl_64x64.zip 압축 해제: ram_hex/capture_08685.mem ~ 08729.mem
2. 이 패키지도 압축 해제하고 터미널에서 패키지 폴더로 이동합니다.
3. gcc 또는 clang으로 컴파일합니다 (Windows MinGW64 GCC 사용 가능):

   gcc -std=c11 -O2 -Wall -Wextra cnn_int8.c cnn_model_data.c cnn_mem_test.c -o cnn_mem_test.exe

4. 이미지 하나 실행:

   ./cnn_mem_test.exe model.mem ram_hex/capture_08685.mem

   이미지 ZIP이 다른 위치라면 ram_hex/... 대신 실제 경로를 적으세요.
   PowerShell에서는 .\cnn_mem_test.exe model.mem .\ram_hex\capture_08685.mem

5. 여러 이미지:

   PowerShell: .\cnn_mem_test.exe model.mem .\ram_hex\*.mem
   주의: PowerShell에서 외부 프로그램에 전달하는 와일드카드 동작은 환경에 따라 다릅니다.
   파일별 실행이 확실한 방식:
     Get-ChildItem .\ram_hex\*.mem | ForEach-Object { .\cnn_mem_test.exe model.mem $_.FullName }

확인된 출력 (capture_08685.mem):
  fc2_mac_before_bias = 13634
  fc2_after_bias      = 3995
  angle_deg           = 4

입력 처리: 이미지 MEM 한 줄은 RGB uint8의 RRGGBB입니다. 실제 RTL의
act_path와 동일하게 각 채널에 0x80 XOR (signed 값으로는 원본-128)을
적용한 뒤 R,G,B 순서의 HWC 배열로 추론합니다. 기존에 제안된
input_scale 나눗셈은 현재 RTL 경로와 맞지 않아 적용하지 않습니다.

위 결과는 C 소프트웨어 참조 모델의 계산 결과입니다. RTL 보드 실측 결과가
같다는 주장은 아니며, RTL과 비교할 기준값입니다. 원본 C 모델의 계산
가정과 실제 RTL 간 다른 부분이 있으면 별도 대조가 필요합니다.
