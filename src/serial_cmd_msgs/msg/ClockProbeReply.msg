# ClockProbe 에 대한 Pi 의 응답.
string robot_id
string host_addr
string seq_id
builtin_interfaces/Time t1          # 프로브에 들어 있던 송신 시각 (PC 시계)
builtin_interfaces/Time t2          # Pi 가 받은 시각 (Pi 시계)
builtin_interfaces/Time t3          # Pi 가 응답을 보낸 시각 (Pi 시계)
