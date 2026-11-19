# 수신 Pi 가 ScheduledCmd 를 실행한 뒤 보내는 응답.

string robot_id
string host_addr                    # 이 Pi 의 IP (robot_id 가 겹쳐도 구분하기 위함)
string seq_id
builtin_interfaces/Time exec_time   # 요청된 실행 시각
float64 error_ms                    # 첫 명령 실제 전송 시각 - exec_time (ms, +면 늦음). 이 Pi 시계 기준
string[] cmds
bool ok
string message                      # 오류/드롭 사유 등

# 실행 직후 읽은 chrony 상태 (chronyc tracking). 이 Pi 시계가 기준 시계와 얼마나 다른지
bool clock_synced                   # chrony 가 동작하고 Leap Normal 이며 기준 서버를 따르는 중
float64 clock_offset_ms             # 이 Pi 시계 - 기준 시계 (ms, +면 이 Pi 시계가 빠름). synced 아니면 0
string clock_ref                    # 따르는 기준 서버 (IP 또는 이름) 와 stratum, 예: "192.168.10.30 s11"
