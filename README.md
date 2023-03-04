# ad-4-lane-keeping-drive-project
[4기] 자율주행 데브코스 차선인식 모의 경진대회 프로젝트 repository

# lane-keping-drive-project
## 차선 인식 방법
1. image stratching을 이용해 밝기 값을 늘린다.
2. image threshold를 이용해 일정 밝기 이상은 masking 처리한다.
3. 남아있는 이미지에 hough transform linep를 적용해 차선을 찾는다.
4. 관측된 차선을 좌우 차선으로 나누고 평균을 내어 차선의 중앙위치를 찾는다.

## 주행 알고리즘
1. 관측된 차선들의 중심과 이미지의 중심의 차이를 기준으로 error를 산출한다.
2. 산출된 error에 moving average를 적용한다.
3. 새롭게 계산된 error를 기반으로 steering angle을 조절한다.
4. PID기법을 통해 angle값을 조절한다.

주행 결과 이미지를 남기지 못했다...
