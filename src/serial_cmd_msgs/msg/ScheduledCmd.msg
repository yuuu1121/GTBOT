# 지정한 시각(exec_time, 시스템 wall-clock)에 cmds 를 순서대로 시리얼로 전송한다.
# 모든 수신 Pi 가 chrony 로 시계를 맞춰두면 네트워크 지연과 무관하게 동시에 실행된다.

builtin_interfaces/Time exec_time   # 실행 시각 (epoch, 시스템 시각 기준)
string[] cmds                       # 순서대로 전송할 명령 (예: ["x", "1001"])
float64 gap                         # cmds 사이 간격(초). 0 이면 연속 전송
string[] targets                    # 대상 robot_id. 비어 있으면 전체
string seq_id                       # ack 매칭용 식별자
