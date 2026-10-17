# TÀI LIỆU ĐẶC TẢ TEST CASE KIỂM THỬ HÀNH VI THỰC TẾ XE (REAL-VEHICLE TEST SPECIFICATION)

Tài liệu này quy định toàn bộ các ca kiểm thử hành vi thực tế (real-world vehicle behavioral tests) nhằm đánh giá, nghiệm thu và đảm bảo an toàn tuyệt đối cho xe tự hành trước và trong khi vận hành thực địa.

---

## 1. QUY TẮC AN TOÀN TRƯỚC KHI THỬ NGHIỆM (SAFETY FIRST)

> [!CAUTION]
> **CẢNH BÁO AN TOÀN BẮT BUỘC:**
> 1. **Kê kích nâng bánh (Jack-Up)**: Đối với toàn bộ các bài test trong Nhóm 1, 2, 3, 4, 6 (E-Stop), 7 và 8, xe **BẮT BUỘC phải được kê kích nâng cao khỏi mặt đất** sao cho cả bánh dẫn động (traction drive) và bánh dẫn hướng (steering wheel) không chạm sàn.
> 2. **Người giám sát nút E-Stop cơ khí**: Trong suốt quá trình chạy bất kỳ test case nào, luôn phải có một kỹ sư túc trực tay cạnh nút **E-Stop cơ khí (DI7)** trên xe, sẵn sàng nhấn ngắt tức thì nếu có bất kỳ hiện tượng bất thường (rung lắc, kẹt cơ cấu, rồ ga ngoài tầm kiểm soát).
> 3. **Điện áp nguồn**: Kiểm tra điện áp pin/nguồn trước khi test (CAN1 ID `0x753` phải nằm trong dải danh định 40.0V - 50.0V).
> 4. **Không thử nghiệm trên đường đông người**: Khi chuyển sang giai đoạn chạy thực địa trên mặt đất (Track Test), khu vực thử nghiệm phải là bãi thử kín, bằng phẳng, không có vật cản và không có người trong bán kính 10 mét.

---

## 2. BẢNG TỔNG HỢP CÁC TEST CASES

| Nhóm | Test ID | Tên Ca Kiểm Thử (Test Case Name) | Mức Độ An Toàn | Thiết Bị / Phân Hệ |
| :--- | :--- | :--- | :---: | :--- |
| **Nhóm 1** | TC-01-01 | Kiểm tra phát hiện SocketCAN & Nodes CAN0 | Tiêu chuẩn | CAN0, ODrive 4/5, ESP32 |
| | TC-01-02 | Kiểm tra kết nối & tự động nhận diện RS485 Modbus | Tiêu chuẩn | RS485 USB-UART, Slave 1 & 2 |
| | TC-01-03 | Khởi tạo HSH FSM và xác nhận Safe State ban đầu | An toàn cao | HSH FSM, Relay, Ga, ODrive |
| | TC-01-04 | Kiểm tra toàn vẹn dữ liệu Telemetry Khung Gầm CAN1 | Tiêu chuẩn | CAN1 (Speed 0x751, Bat 0x753) |
| **Nhóm 2** | TC-02-01 | Tuần tự đóng Relay vào số Tiến (Drive Engagement) | An toàn cao | Relay 6, Relay 8, AO8 Ga |
| | TC-02-02 | Điều khiển điện áp ga tuyến tính 0% - 100% | Tiêu chuẩn | AO8 DAC Ga (0.0V .. 5.0V) |
| | TC-02-03 | Tuần tự đóng Relay vào số Lùi (Reverse Engagement) | An toàn cao | Relay 7, Relay 8, AO8 Ga |
| | TC-02-04 | Trả về số Mo (Neutral) và cắt động lực tức thì | Tiêu chuẩn | Relay 6/7/8, AO8 Ga |
| | TC-02-05 | Khóa liên động chống sốc đảo chiều khi xe lăn bánh | **Cực kỳ quan trọng**| Relay 6/7, Deadtime, Speed Interlock |
| **Nhóm 3** | TC-03-01 | Đánh lái danh định theo góc đặt (-1.0 đến +1.0 rad) | Tiêu chuẩn | ODrive Node 5 (-70 .. +70 vòng) |
| | TC-03-02 | Kiểm tra bộ băm xung gia tốc góc lái (Ramp Rate) | Tiêu chuẩn | Ramp Limiter (35 vòng/s) |
| | TC-03-03 | Trả thẳng lái về tâm 0 rad (Centering Test) | Tiêu chuẩn | ODrive Node 5, Biến trở 43.1% |
| | TC-03-04 | Chặn mềm 85% hành trình biến trở góc lái | **Cực kỳ quan trọng**| Biến trở AI8 (12.5% .. 73.7%) |
| | TC-03-05 | Chống giật vị trí khi lệnh lái vượt biên (> 1.0 rad) | Tiêu chuẩn | Controller Clamping Logic |
| **Nhóm 4** | TC-04-01 | Thả phanh tự do ở trạng thái IDLE (0% phanh) | Tiêu chuẩn | ODrive Node 4, Lò xo phanh |
| | TC-04-02 | Điều khiển lực phanh tỷ lệ (30%, 60%, 100%) | Tiêu chuẩn | ODrive Node 4 (0 .. +3.0 vòng) |
| | TC-04-03 | Giữ phanh tĩnh và nhả êm không kẹt | Tiêu chuẩn | ODrive ClosedLoop Position |
| **Nhóm 5** | TC-05-01 | Bật/Tắt độc lập từng Relay phụ trợ 1..5 | Tiêu chuẩn | ESP32 Relay 1..5 (Mask 0x1F) |
| | TC-05-02 | Tính độc lập giữa Relay phụ và Relay động lực 6..8 | An toàn cao | Relay Mask Logic |
| | TC-05-03 | Duy trì Relay phụ khi xe ở Safe Mode / Dừng chờ | Tiêu chuẩn | Controller Safe State Mask |
| **Nhóm 6** | TC-06-01 | Nhấn nút E-Stop cơ khí DI7 khi đang chạy | **Cực kỳ quan trọng**| DI7 ESP32, FSM STOP, Phanh 100% |
| | TC-06-02 | Kích hoạt E-Stop phần mềm từ lệnh ROS 2 | An toàn cao | `/vehicle/control/func_cmd` |
| | TC-06-03 | Khóa liên động phục hồi E-Stop ở chế độ AUTO | **Cực kỳ quan trọng**| DI6 Cycle Interlock (AUTO->MAN->AUTO) |
| | TC-06-04 | Phục hồi E-Stop trực tiếp ở chế độ MANUAL | An toàn cao | DI6 = 1 (MANUAL), FSM READY |
| **Nhóm 7** | TC-07-01 | Chuyển chế độ Auto <-> Manual khi đứng yên | Tiêu chuẩn | DI6 ESP32, FSM Mode Interlock |
| | TC-07-02 | Từ chối chuyển chế độ khi xe đang di chuyển | An toàn cao | Vận tốc > 0.5 km/h, Mode Lock |
| **Nhóm 8** | TC-08-01 | Watchdog mất lệnh điều khiển ROS 2 (Timeout 250ms)| An toàn cao | Controller Command Timeout |
| | TC-08-02 | Watchdog mất gói tin vận tốc CAN1 (> 500ms) | An toàn cao | CAN1 Rx Worker Watchdog |
| | TC-08-03 | Xử lý ngắt kết nối đường truyền RS485 Modbus | An toàn cao | Serial Watchdog & Auto-reprobe |
| | TC-08-04 | Giám sát tải bus CAN0 và cơ chế chống tràn ACK | Tiêu chuẩn | CAN0 Bandwidth, Relay CAN ACK |

---

## 3. CHI TIẾT CÁC BÀI TEST (TEST CASES SPECIFICATION)

---

### NHÓM 1: KHỞI ĐỘNG & NHẬN DIỆN PHẦN CỨNG (BOOTUP & DISCOVERY)

#### TC-01-01: Kiểm tra phát hiện SocketCAN & Nodes CAN0
- **Mục tiêu**: Xác nhận giao tiếp SocketCAN `can0` hoạt động bình thường ở tốc độ 500kbps, phát hiện đầy đủ ODrive Node 4, ODrive Node 5 và ESP32 Relay Board.
- **Tiền điều kiện**: Đã cắm cáp CAN vào cổng `can0` của máy tính Jetson/PC. Bật nguồn logic 24V/48V của xe.
- **Các bước thực hiện**:
  1. Mở terminal, kiểm tra giao diện can0: `ip -details link show can0`.
  2. Bật bringup hệ thống: `ros2 launch vehicle_controller vehicle_hardware_bringup.launch.py`.
  3. Mở terminal khác kiểm tra lưu lượng CAN: `candump can0`.
- **Hành vi phần cứng kỳ vọng**:
  - Đèn LED trạng thái trên 2 bộ ODrive nháy xanh bình thường (không báo đỏ lỗi).
  - Đèn LED trên mạch Waveshare ESP32 nháy báo nhận diện CAN.
- **Hành vi phần mềm / Telemetry kỳ vọng**:
  - `candump can0` thấy các bản tin từ ODrive: `0x084`, `0x0A4` (Node 4), `0x085`, `0x0A5` (Node 5) và bản tin định kỳ `0x580` từ ESP32 (chu kỳ 50ms).
  - Log ROS 2: `RelayCANSystem started`, `OdriveCANSystem` online.
  - Topic `/vehicle/state/system` có `hardware_online = true`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Tất cả các node CAN phản hồi ổn định trong 10 giây, không có drop frame hoặc bus-off.
  - **FAIL**: can0 không nhận diện được, hoặc thiếu bất kỳ node nào (Node 4, 5 hoặc 0x580).

#### TC-01-02: Kiểm tra kết nối & tự động nhận diện RS485 Modbus
- **Mục tiêu**: Xác nhận hệ thống tự động tìm đúng cổng USB-RS485 (qua `/dev/serial/by-id/` hoặc `/dev/ttyUSB*`) và thiết lập giao tiếp Modbus RTU 9600 baud với 2 module Input (ID 1) và Output (ID 2).
- **Tiền điều kiện**: Cắm cáp USB-RS485 vào máy tính, module Modbus được cấp nguồn.
- **Các bước thực hiện**:
  1. Quan sát log khởi động của node `ros2_control_node`.
  2. Kiểm tra giá trị đọc biến trở: `ros2 topic echo /vehicle/state/motion --field steering_angle_rad`.
- **Hành vi phần cứng kỳ vọng**:
  - Đèn TX/RX trên cổng chuyển đổi USB-RS485 nháy đều đặn với tần số ~20-30Hz.
  - Đèn RUN trên 2 module Modbus sáng bình thường.
- **Hành vi phần mềm kỳ vọng**:
  - Log xuất hiện: `Auto-detected and connected RS485 Modbus device on: /dev/...`.
  - Giá trị biến trở đọc về thanh ghi `0x0007` trả về trong khoảng hợp lệ (0% .. 100%).
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Modbus đọc và ghi dữ liệu thành công không báo lỗi timeout quá 3 lần liên tiếp.
  - **FAIL**: Không tìm thấy cổng serial, hoặc lỗi CRC liên tục.

#### TC-01-03: Khởi tạo HSH FSM và xác nhận Safe State ban đầu
- **Mục tiêu**: Đảm bảo khi khởi động xe, FSM tự động chuyển tuần tự từ `INIT` -> `READY` và giữ nguyên trạng thái an toàn tuyệt đối (không có bất kỳ chuyển động tự phát nào).
- **Tiền điều kiện**: Xe đã khởi động, không nhấn E-Stop.
- **Các bước thực hiện**:
  1. Theo dõi FSM state: `ros2 topic echo /hsh/fsm_state`.
  2. Đo điện áp ngõ ra AO8 (chân cấp ga cho biến tần/bộ điều khiển truyền động).
  3. Kiểm tra trạng thái rơle và trục ODrive.
- **Hành vi phần cứng kỳ vọng**:
  - Toàn bộ Relay 6, 7, 8 đều MỞ (ngắt điện).
  - Điện áp ga đo bằng đồng hồ VOM tại ngõ ra AO8 đúng bằng **0.00V ± 0.05V**.
  - Động cơ lái ODrive Node 5 và phanh Node 4 ở trạng thái IDLE (trục quay tự do nhẹ nhàng, không ăn dòng).
- **Hành vi phần mềm kỳ vọng**:
  - Topic `/hsh/fsm_state` báo `Ready`.
  - Topic `/vehicle/state/system` báo `safe_command_active = true`.
  - Topic `/vehicle/state/motion` báo `gear = 0 (NEUTRAL)`, `speed_percentage = 0.0`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: FSM đạt `Ready` trong vòng 3 giây sau khi bật bringup; xe bất động tuyệt đối, ga 0V.
  - **FAIL**: Xuất hiện điện áp ga > 0.1V, hoặc có rơle 6/7/8 đóng, hoặc động cơ ODrive bị giật.

#### TC-01-04: Kiểm tra toàn vẹn dữ liệu Telemetry Khung Gầm CAN1
- **Mục tiêu**: Xác nhận luồng đọc dữ liệu OEM từ CAN1 (Điện áp pin `0x753` và Vận tốc bánh xe `0x751`).
- **Tiền điều kiện**: Cắm cổng `can1` vào xe. Bật khóa điện khung gầm xe.
- **Các bước thực hiện**:
  1. Chạy script telemetry: `python3 tests/test_can1_telemetry.py`.
- **Hành vi phần cứng kỳ vọng**:
  - CAN1 nhận tín hiệu liên tục từ ECU/BMS của xe.
- **Hành vi phần mềm kỳ vọng**:
  - Giao diện hiển thị:
    - `Battery Voltage`: Nằm trong khoảng 40.00V đến 50.00V (ví dụ: `48.52 V`).
    - `Vehicle Speed`: Khi xe đứng yên, hiển thị `0.00 km/h` (raw `0x00000000`).
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Điện áp pin và tốc độ xe được giải mã chính xác, tần số nhận tin >= 10Hz.
  - **FAIL**: Giá trị pin nhảy loạn, hoặc không có frame nào trên CAN1.

---

### NHÓM 2: TRUYỀN ĐỘNG & TUẦN TỰ RELAY (DRIVE SEQUENCING & SHIFTING)

#### TC-02-01: Tuần tự đóng Relay vào số Tiến (Drive Engagement)
- **Mục tiêu**: Xác thực logic trễ tuần tự phần cứng: Relay 6 (Hướng Tiến) đóng trước -> Chờ 100ms -> Relay 8 (Nguồn chính) đóng -> Chờ 100ms -> Điện áp ga mới được phép nạp.
- **Tiền điều kiện**: Xe kê kích nâng bánh. FSM ở `READY`. Công tắc chế độ ở `AUTO`.
- **Các bước thực hiện**:
  1. Bật motion enable: gửi `/vehicle/control/func_cmd` với `motion_enable = true`.
  2. Gửi lệnh vào số Tiến với 30% ga: `/vehicle/control/motion_cmd` (`gear = 1 (DRIVE)`, `speed_percentage = 30.0`).
  3. Lắng nghe tiếng rơle tạch và quan sát máy hiện sóng / VOM trên chân Relay 6, Relay 8 và chân AO8 ga.
- **Hành vi phần cứng kỳ vọng**:
  - Nghe tiếng "tạch" thứ nhất: Relay 6 đóng (đèn LED 6 trên mạch ESP32 sáng).
  - Khoảng ~100ms sau, nghe tiếng "tạch" thứ hai: Relay 8 đóng (đèn LED 8 sáng).
  - Khoảng ~100ms tiếp theo: Động cơ truyền động bắt đầu quay êm về phía trước.
- **Hành vi phần mềm kỳ vọng**:
  - Mask relay chuyển tuần tự: `0x00` -> `0x20` (sau 0ms) -> `0xA0` (sau 100ms) -> Giữ nguyên `0xA0`.
  - Điện áp AO8: Giữ 0.0V trong 200ms đầu tiên, sau 200ms nạp lên `1.50V` (30% của 5V).
  - Topic `/vehicle/state/motion` báo `gear = 1`, `speed_percentage = 30.0`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Chuỗi đóng rơle xảy ra đúng thứ tự, tuyệt đối không có hiện tượng nạp ga trước khi Relay 8 đóng hoàn tất.
  - **FAIL**: Relay 8 đóng cùng lúc Relay 6, hoặc ga xuất hiện trước khi rơle đóng.

#### TC-02-02: Điều khiển điện áp ga tuyến tính 0% - 100%
- **Mục tiêu**: Kiểm tra độ chính xác và tính tuyến tính của điện áp ga tương tự qua cổng RS485 AO8.
- **Tiền điều kiện**: Xe ở trạng thái Tiến (DRIVE) ổn định.
- **Các bước thực hiện**:
  1. Lần lượt gửi lệnh `speed_percentage`: `0%`, `20%`, `40%`, `60%`, `80%`, `100%`.
  2. Dùng đồng hồ đo điện áp DC tại ngõ ra chân AO8.
- **Hành vi phần cứng kỳ vọng**:
  - Bánh xe tăng tốc đều đặn tương ứng với mức ga đặt.
  - Điện áp thực đo trên chân AO8:
    - 0%: 0.00V (± 0.05V)
    - 20%: 1.00V (± 0.05V)
    - 40%: 2.00V (± 0.05V)
    - 60%: 3.00V (± 0.05V)
    - 80%: 4.00V (± 0.05V)
    - 100%: 5.00V (± 0.05V)
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Điện áp sai lệch không quá ±2% toàn dải (±0.1V), động cơ đáp ứng tuyến tính không giật.
  - **FAIL**: Điện áp bị bão hòa sớm, hoặc sai lệch lớn hơn ±0.1V.

#### TC-02-03: Tuần tự đóng Relay vào số Lùi (Reverse Engagement)
- **Mục tiêu**: Xác thực tuần tự vào số Lùi: Relay 7 (Hướng Lùi - `0x40`) đóng trước -> Chờ 100ms -> Relay 8 đóng (`0xC0`) -> Chờ 100ms -> Nạp ga.
- **Tiền điều kiện**: Xe kê kích nâng bánh, đang ở số Mo (Neutral).
- **Các bước thực hiện**:
  1. Gửi lệnh: `gear = 2 (REVERSE)`, `speed_percentage = 25.0`.
- **Hành vi phần cứng kỳ vọng**:
  - Relay 7 đóng trước (LED 7 sáng), sau 100ms Relay 8 đóng (LED 8 sáng).
  - Bánh xe quay ngược chiều (lùi) mượt mà.
- **Hành vi phần mềm kỳ vọng**:
  - Relay mask: `0x00` -> `0x40` -> `0xC0`.
  - Điện áp AO8 đạt `1.25V` (25% của 5V) sau 200ms.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Bánh xe quay lùi êm ái, đúng trễ tuần tự.
  - **FAIL**: Relay 6 bị đóng nhầm thay vì Relay 7, hoặc chập rơle.

#### TC-02-04: Trả về số Mo (Neutral) và cắt động lực tức thì
- **Mục tiêu**: Đảm bảo khi trả số về Mo, ga lập tức bị ngắt về 0V và toàn bộ rơle truyền động ngắt điện.
- **Tiền điều kiện**: Xe đang chạy ở số Tiến (Drive 50%).
- **Các bước thực hiện**:
  1. Gửi lệnh: `gear = 0 (NEUTRAL)`.
- **Hành vi phần cứng kỳ vọng**:
  - Tiếng rơle nhả ngay lập tức.
  - Động cơ mất điện áp điều khiển ngay tức khắc và quay trôi tự do chậm dần đến khi dừng.
- **Hành vi phần mềm kỳ vọng**:
  - Điện áp AO8 rớt về 0.0V trong chu kỳ điều khiển tiếp theo (< 5ms).
  - Relay mask chuyển về `0x00` (hoặc chỉ giữ lại relay phụ nếu có).
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Động lực bị ngắt trong vòng dưới 10ms.
  - **FAIL**: Relay giữ trạng thái đóng hoặc ga duy trì điện áp > 0V sau khi chuyển Mo.

#### TC-02-05: Khóa liên động chống sốc đảo chiều khi xe lăn bánh (Speed Interlock)
- **Mục tiêu**: Xác thực cơ chế bảo vệ hộp số và động cơ: Tuyệt đối không cho phép đảo chiều rơle (Tiến -> Lùi hoặc Lùi -> Tiến) khi xe đang còn trôi/lăn bánh.
- **Tiền điều kiện**: Xe kê kích nâng bánh. Xe đang chạy số Tiến (Drive 40%).
- **Các bước thực hiện**:
  1. Đang ở số Tiến, bất ngờ gửi lệnh số Lùi (`gear = 2 (REVERSE)`, `speed = 40%`).
  2. Quan sát phản ứng rơle trong khi bánh xe vẫn còn đang quay theo đà quán tính (vận tốc góc > 0.1 rad/s).
  3. Để bánh xe quay chậm dần cho tới khi dừng hẳn.
- **Hành vi phần cứng kỳ vọng**:
  - Ngay khi nhận lệnh Lùi: Relay 6 và Relay 8 ngắt ngay lập tức, ga rớt về 0V.
  - **Trong khi bánh xe còn quay theo quán tính**: Relay 7 (Lùi) **BỊ CẤM HOÀN TOÀN** không được đóng!
  - **Sau khi bánh xe dừng yên hoàn toàn (dưới 0.1 rad/s) VÀ hết deadtime 100ms**: Relay 7 mới bắt đầu đóng -> Relay 8 đóng -> Ga lùi mới được nạp.
- **Hành vi phần mềm kỳ vọng**:
  - Trạng thái FSM của controller vào `DISENGAGING_FOR_REVERSE`.
  - Command interface giữ `cmd_relay_mask = 0` và `cmd_throttle = 0.0V` chừng nào `state_wheel_vel > 0.1 rad/s`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Relay lùi chỉ đóng sau khi bánh xe dừng quay hoàn toàn; không có tiếng giật cơ khí va đập hộp số.
  - **FAIL**: Relay lùi đóng trong khi bánh xe vẫn đang quay tiến.

---

### NHÓM 3: HỆ THỐNG LÁI ODRIVE & CẢM BIẾN BIẾN TRỞ (STEERING SYSTEM)

#### TC-03-01: Đánh lái danh định theo góc đặt (-1.0 đến +1.0 rad)
- **Mục tiêu**: Kiểm tra độ bám vị trí của động cơ ODrive Node 5 khi nhận lệnh góc lái.
- **Tiền điều kiện**: Xe kê kích bánh lái. FSM ở `READY` hoặc `RUNNING`.
- **Các bước thực hiện**:
  1. Gửi lệnh góc lái sang phải: `steering_angle_rad = +0.5 rad` (nửa hành trình phải).
  2. Quan sát số vòng quay thực tế trên ODrive và cơ cấu lái.
  3. Gửi lệnh góc lái kịch phải: `steering_angle_rad = +1.0 rad`.
  4. Lặp lại với góc lái sang trái: `-0.5 rad` và `-1.0 rad`.
- **Hành vi phần cứng kỳ vọng**:
  - Bánh lái xoay êm ái, chuyển hướng chính xác sang phải/trái theo lệnh.
- **Hành vi phần mềm kỳ vọng**:
  - Tại `+0.5 rad`: Động cơ ODrive quay tới `+35.0 vòng` (tương ứng `+35.0 * 2π rad`).
  - Tại `+1.0 rad`: Động cơ ODrive quay tới `+70.0 vòng`.
  - Tại `-0.5 rad`: Động cơ ODrive quay tới `-35.0 vòng`.
  - Tại `-1.0 rad`: Động cơ ODrive quay tới `-70.0 vòng`.
  - Topic `/vehicle/state/motion` báo góc lái thực tế bám theo góc lái đặt.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Sai lệch góc quay ODrive không quá ±0.5 vòng tại các điểm dừng.
  - **FAIL**: Động cơ quay sai hướng, hoặc dừng sai lệch vị trí > 2 vòng.

#### TC-03-02: Kiểm tra bộ băm xung gia tốc góc lái (Steering Ramp Rate)
- **Mục tiêu**: Đảm bảo vận tốc thay đổi góc lái được băm xung mượt mà theo `steering_ramp_rate_turns_s = 35.0 vòng/s`, không đánh lái giật cục làm hỏng cơ cấu cơ khí.
- **Tiền điều kiện**: Xe đang ở vị trí lái thẳng (0 vòng).
- **Các bước thực hiện**:
  1. Gửi một lệnh bước nhảy tức thời (step command) từ 0 rad sang +1.0 rad (tương ứng nhảy 70 vòng).
  2. Đo thời gian cơ cấu lái quay từ 0 vòng đến 70 vòng.
- **Hành vi phần cứng kỳ vọng**:
  - Bánh lái quay đều đặn, không có gia tốc giật mạnh đột ngột gây rung lắc khung gầm.
  - Thời gian quay hết 70 vòng xấp xỉ đúng **2.0 giây** (vì $70 / 35 = 2.0\text{ s}$).
- **Hành vi phần mềm kỳ vọng**:
  - Biến `current_steering_turns_` trong controller tăng tuyến tính với độ dốc $35.0 \times \Delta t$ trong mỗi chu kỳ 5ms.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Thời gian đáp ứng từ 0 đến 70 vòng nằm trong khoảng $2.0 \pm 0.2\text{ giây}$.
  - **FAIL**: Bánh lái bị giật tức thời hoặc mất hơn 3 giây mới đạt vị trí.

#### TC-03-03: Trả thẳng lái về tâm 0 rad (Centering Test)
- **Mục tiêu**: Kiểm tra khả năng tự động trả thẳng lái về vị trí 0 rad khi nhận lệnh.
- **Tiền điều kiện**: Bánh lái đang bẻ hết sang một bên (+70 vòng hoặc -70 vòng).
- **Các bước thực hiện**:
  1. Gửi lệnh `steering_angle_rad = 0.0`.
- **Hành vi phần cứng kỳ vọng**:
  - Cơ cấu lái xoay thẳng đều về vị trí trung tâm thẳng bánh.
- **Hành vi phần mềm kỳ vọng**:
  - Target ODrive về `0.0 vòng`.
  - Giá trị đọc biến trở hồi tiếp xấp xỉ mức tâm danh định `43.1% ± 2.0%`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Bánh xe thẳng hàng, biến trở hồi tiếp đạt giá trị tâm.
  - **FAIL**: Bánh xe bị lệch quá 5 độ so với trục dọc của xe.

#### TC-03-04: Chặn mềm 85% hành trình biến trở góc lái (Potentiometer Soft Limit)
- **Mục tiêu**: Đảm bảo cơ chế an toàn độc lập bằng biến trở: khi cơ cấu chạm mốc 85% hành trình cảm biến biến trở, hệ thống lập tức khóa không cho quay sâu thêm nhằm tránh kịch cơ khí phá hỏng thước lái.
- **Tiền điều kiện**: Biến trở hoạt động bình thường, đọc về qua RS485 AI8.
- **Các bước thực hiện**:
  1. Cho bánh lái quay từ từ sang biên phải: quan sát giá trị biến trở tăng dần qua 50%, 60%, 65%, chạm mốc **69.1%** (ngưỡng 85% của dải phải: $43.1 + 0.85 \times 30.6 = 69.11\%$).
  2. Tiếp tục giữ lệnh bẻ lái sâu thêm sang phải.
  3. Sau đó gửi lệnh bẻ lái ngược sang trái (về hướng trung tâm).
- **Hành vi phần cứng kỳ vọng**:
  - Khi biến trở chạm ngưỡng 69.1%, động cơ ODrive lập tức dừng quay thêm sang phải dù lệnh ROS vẫn yêu cầu tăng góc.
  - Khi người vận hành gửi lệnh bẻ ngược lại sang trái, động cơ ODrive phản hồi lập tức và xoay sang trái bình thường.
- **Hành vi phần mềm kỳ vọng**:
  - Controller so sánh `normalized_steer >= limit_ratio (0.85)` và khóa `requested_turns = cur_turns` đối với các lệnh tăng góc.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Cơ cấu dừng trước khi chạm cữ cơ khí kịch biên; cho phép quay ngược lại vùng an toàn.
  - **FAIL**: Động cơ tiếp tục quay ép cữ cơ khí gây quá dòng ODrive.

#### TC-03-05: Chống giật vị trí khi lệnh lái vượt biên (> 1.0 rad)
- **Mục tiêu**: Kiểm tra khả năng chống lỗi số học (cliff/overflow bug) khi nhận góc lái nằm ngoài dải thiết kế.
- **Tiền điều kiện**: Bánh lái đang ở +70 vòng (+1.0 rad).
- **Các bước thực hiện**:
  1. Gửi lệnh góc lái bất thường: `steering_angle_rad = +1.2 rad` rồi `+2.0 rad`.
- **Hành vi phần cứng kỳ vọng**:
  - Bánh lái giữ nguyên vị trí ổn định ở +70 vòng, tuyệt đối không bị giật lùi về tâm hay quay loạn.
- **Hành vi phần mềm kỳ vọng**:
  - Giá trị góc lái bị kẹp (clamped) chính xác tại `steering_max_turns_ = 70.0`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Cơ cấu đứng yên vững chắc tại 70 vòng.
  - **FAIL**: Động cơ giật ngược về 0 hoặc có hiện tượng rung lắc giật cục.

---

### NHÓM 4: HỆ THỐNG PHANH ĐIỆN ODRIVE (BRAKING SYSTEM)

#### TC-04-01: Nhả phanh tự do ở trạng thái IDLE (0% phanh)
- **Mục tiêu**: Xác nhận khi không có lệnh phanh, động cơ phanh ODrive Node 4 ở trạng thái IDLE hoàn toàn, cho phép lò xo cơ khí kéo nhả má phanh không bị bó kẹt.
- **Tiền điều kiện**: Xe nâng bánh, phanh đang nhả.
- **Các bước thực hiện**:
  1. Gửi lệnh `brake_percentage = 0.0`.
  2. Dùng tay quay thử bánh dẫn động.
- **Hành vi phần cứng kỳ vọng**:
  - Bánh xe quay nhẹ nhàng tự do, không có ma sát má phanh.
  - Trục động cơ ODrive Node 4 không bị ghì cứng.
- **Hành vi phần mềm kỳ vọng**:
  - Lệnh gửi xuống ODrive: `cmd_brake_idle = 1.0` (IDLE mode), `cmd_brake_pos = 0.0`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Bánh quay trơn tru, không sinh nhiệt do ma sát phanh.
  - **FAIL**: Động cơ ODrive bị khóa cứng (closed-loop) ở vị trí phanh.

#### TC-04-02: Điều khiển lực phanh tỷ lệ (30%, 60%, 100%)
- **Mục tiêu**: Xác nhận hành trình kéo dây phanh tỷ lệ theo mức phần trăm yêu cầu (tối đa +3.0 vòng).
- **Tiền điều kiện**: Xe nâng bánh.
- **Các bước thực hiện**:
  1. Gửi lệnh `brake_percentage = 30.0%`: Quan sát hành trình cơ cấu phanh. Thử dùng tay quay bánh xe.
  2. Gửi lệnh `brake_percentage = 60.0%`.
  3. Gửi lệnh `brake_percentage = 100.0%`.
- **Hành vi phần cứng kỳ vọng**:
  - Ở 30%: Má phanh bắt đầu tì nhẹ vào đĩa phanh.
  - Ở 60%: Má phanh kẹp chặt, khó dùng tay quay bánh.
  - Ở 100%: Phanh siết cứng hoàn toàn, không thể quay bánh bằng tay.
- **Hành vi phần mềm kỳ vọng**:
  - Vị trí target ODrive:
    - 30%: `reference_position + 0.9 vòng`.
    - 60%: `reference_position + 1.8 vòng`.
    - 100%: `reference_position + 3.0 vòng`.
  - ODrive chuyển sang chế độ `ClosedLoop` (`cmd_brake_idle = 0.0`).
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Hành trình kéo phanh tăng tịnh tiến theo tỷ lệ, siết cứng ở 100%.
  - **FAIL**: ODrive không kéo dây phanh, hoặc kéo quá 3.5 vòng làm đứt dây cáp phanh.

#### TC-04-03: Giữ phanh tĩnh và nhả êm không kẹt
- **Mục tiêu**: Kiểm tra độ tin cậy khi duy trì phanh trong thời gian dài (30 giây) và nhả phanh.
- **Tiền điều kiện**: Xe đang phanh ở mức 100%.
- **Các bước thực hiện**:
  1. Duy trì lệnh phanh 100% trong 30 giây. Theo dõi nhiệt độ và dòng điện ODrive.
  2. Giảm lệnh phanh về 0%.
- **Hành vi phần cứng kỳ vọng**:
  - ODrive giữ chặt không tuột phanh trong 30 giây.
  - Khi về 0%, phanh nhả nhanh và êm ái nhờ lò xo hồi vị.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Không có lỗi quá nhiệt hay quá dòng trên ODrive Node 4; nhả phanh hoàn toàn sau khi hủy lệnh.
  - **FAIL**: ODrive ngắt dòng do lỗi quá dòng trong lúc giữ phanh, hoặc phanh bị kẹt không nhả.

---

### NHÓM 5: HỆ THỐNG RELAY PHỤ TRỢ (AUXILIARY RELAYS)

#### TC-05-01: Bật/Tắt độc lập từng Relay phụ trợ 1..5
- **Mục tiêu**: Kiểm tra khả năng bật tắt các tải phụ (Đèn pha, Còi, Xi-nhan trái/phải, Đèn cảnh báo) thông qua 5 bit đầu của `io_output_mask`.
- **Tiền điều kiện**: Các thiết bị tải phụ đã được đấu nối vào Relay 1 đến Relay 5 trên board Waveshare.
- **Các bước thực hiện**:
  1. Lần lượt gửi lệnh `io_output_mask` với các giá trị:
     - `0x01` (Bit 0): Bật Relay 1.
     - `0x02` (Bit 1): Bật Relay 2.
     - `0x04` (Bit 2): Bật Relay 3.
     - `0x08` (Bit 3): Bật Relay 4.
     - `0x10` (Bit 4): Bật Relay 5.
     - `0x1F` (Bits 0..4): Bật đồng thời cả 5 relay phụ.
     - `0x00`: Tắt toàn bộ.
- **Hành vi phần cứng kỳ vọng**:
  - Đèn LED tương ứng trên board ESP32 sáng/tắt chính xác.
  - Tải ngoại vi tương ứng (đèn, còi...) hoạt động chuẩn xác.
- **Hành vi phần mềm kỳ vọng**:
  - Topic `/vehicle/state/system` có `io_output_mask` phản hồi đúng bitmask đang bật.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Các relay phụ đóng cắt chính xác 100%, không bị chập chờn.
  - **FAIL**: Có relay không đóng hoặc bật nhầm relay khác.

#### TC-05-02: Tính độc lập giữa Relay phụ và Relay động lực 6..8
- **Mục tiêu**: Đảm bảo việc bật/tắt liên tục các relay phụ không gây ảnh hưởng hay xung đột với trạng thái của Relay truyền động 6, 7, 8.
- **Tiền điều kiện**: Xe đang chạy số Tiến (Relay 6 + Relay 8 đang đóng, mask động lực = `0xA0`).
- **Các bước thực hiện**:
  1. Nhấp nháy liên tục còi hoặc xi-nhan (thay đổi `io_output_mask` giữa `0x00` và `0x05` với chu kỳ 500ms).
- **Hành vi phần cứng kỳ vọng**:
  - Xe vẫn duy trì chạy tiến ổn định với tốc độ không đổi.
  - Relay 6 và Relay 8 không hề bị ngắt hay rung rơle.
- **Hành vi phần mềm kỳ vọng**:
  - Mask tổng gửi xuống CAN luôn bảo toàn 3 bit cao: `0xA0 | 0x05 = 0xA5` và `0xA0 | 0x00 = 0xA0`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Xe di chuyển liên tục, không bị ngắt quãng động lực.
  - **FAIL**: Xe bị khựng hoặc rơle nguồn số 8 bị ngắt nhầm.

#### TC-05-03: Duy trì Relay phụ khi xe ở Safe Mode / Dừng chờ
- **Mục tiêu**: Đảm bảo các tải cảnh báo an toàn (như đèn hazard hoặc còi) vẫn hoạt động được ngay cả khi hệ thống truyền động bị khóa ở chế độ an toàn (Motion Disabled hoặc Safe Mode).
- **Tiền điều kiện**: Gửi lệnh `motion_enable = false`.
- **Các bước thực hiện**:
  1. Gửi lệnh `io_output_mask = 0x05` (Bật đèn hazard + đèn pha).
- **Hành vi phần cứng kỳ vọng**:
  - Đèn hazard và đèn pha vẫn sáng bình thường dù xe không thể di chuyển.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Tải phụ duy trì trạng thái theo mask yêu cầu.
  - **FAIL**: Toàn bộ relay phụ bị tắt ngấm khi vào Safe Mode.

---

### NHÓM 6: CƠ CHẾ DỪNG KHẨN CẤP (E-STOP & RECOVERY PROTOCOL)

#### TC-06-01: Nhấn nút E-Stop cơ khí DI7 khi đang chạy
- **Mục tiêu**: Kiểm tra thời gian đáp ứng và tính toàn diện của cơ chế dừng khẩn cấp cấp độ phần cứng cao nhất khi người dùng ấn nút E-Stop trên xe.
- **Tiền điều kiện**: Xe kê kích nâng bánh. Xe đang chạy số Tiến với ga 50% và đang đánh lái +30 vòng.
- **Các bước thực hiện**:
  1. Kỹ thuật viên ấn mạnh **Nút E-Stop cơ khí (DI7)** trên vỏ xe.
- **Hành vi phần cứng kỳ vọng**:
  - Ngay lập tức trong vòng < 10ms:
    - Tiếng rơle nảy ngắt toàn bộ Relay 6, 7, 8.
    - Động cơ kéo bánh xe mất điện hoàn toàn.
    - Động cơ phanh ODrive Node 4 gầm lên siết 100% (+3.0 vòng) ghì chặt bánh xe lại.
    - Động cơ lái ODrive Node 5 lập tức chuyển về IDLE (thả lỏng cơ cấu, không bị kẹt cưỡng bức).
- **Hành vi phần mềm kỳ vọng**:
  - FSM nhảy ngay sang trạng thái `STOP` (do nhận `EvCriticalError`).
  - Điện áp ga AO8 rớt về 0.0V.
  - Topic `/vehicle/state/system` báo `emergency_stop = true`, `hardware_motion_allowed = false`, `safe_command_active = true`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Toàn bộ động lực bị triệt tiêu, phanh khóa chặt trong vòng dưới 20ms.
  - **FAIL**: Xe tiếp tục quay bánh, hoặc phanh không ép, hoặc rơle không ngắt.

#### TC-06-02: Kích hoạt E-Stop phần mềm từ lệnh ROS 2
- **Mục tiêu**: Kiểm tra cơ chế dừng khẩn cấp kích hoạt qua mạng ROS 2 từ xa (Remote Stop).
- **Tiền điều kiện**: Xe đang chạy ở số Tiến (Drive 30%).
- **Các bước thực hiện**:
  1. Gửi lệnh ROS 2: `ros2 topic pub --once /vehicle/control/func_cmd vehicle_msgs/msg/VehicleFunctionCommand "{motion_enable: false, emergency_stop: true}"`.
- **Hành vi phần cứng & phần mềm kỳ vọng**:
  - Động lực ngắt ngay lập tức, phanh siết 100%, ga về 0V tương tự như nhấn E-Stop cứng.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Xe dừng khẩn cấp ngay khi nhận bản tin ROS.
  - **FAIL**: Xe tiếp tục di chuyển hoặc bỏ qua cờ `emergency_stop`.

#### TC-06-03: Khóa liên động phục hồi E-Stop ở chế độ AUTO (Strict Mode-Cycle Interlock)
- **Mục tiêu**: Xác thực cơ chế chống tai nạn rồ ga bất ngờ (Vehicle Runaway Prevention): Khi đang ở chế độ AUTO mà nhả nút E-Stop, xe **TUYỆT ĐỐI KHÔNG ĐƯỢC TỰ ĐỘNG CHẠY TIẾP**. Bắt buộc người vận hành phải gạt công tắc chế độ về MANUAL rồi gạt lại AUTO thì mới được cấp phép hoạt động lại.
- **Tiền điều kiện**: Xe vừa bị kích hoạt E-Stop (FSM ở `STOP`), công tắc chế độ trên xe đang ở vị trí **AUTO (DI6 = 0)**. Xe đã dừng yên hoàn toàn.
- **Các bước thực hiện**:
  1. Xoay nhả nút E-Stop cơ khí (DI7 chuyển từ 1 về 0).
  2. Quan sát trạng thái xe và log ROS 2 trong 5 giây (vẫn giữ nguyên công tắc ở AUTO).
  3. Sau đó, gạt công tắc chế độ trên xe sang **MANUAL (DI6 = 1)**: Quan sát.
  4. Tiếp tục gạt công tắc trở lại **AUTO (DI6 = 0)**: Quan sát.
- **Hành vi phần cứng & phần mềm kỳ vọng**:
  - **Tại bước 1 & 2**: Khi vừa nhả nút E-Stop, phanh nhả về IDLE. Tuy nhiên, FSM **VẪN BỊ KHÓA Ở TRẠNG THÁI `STOP`**. Log HSH cảnh báo:
    `"E-Stop đã nhả nhưng xe đang ở chế độ AUTO. Yêu cầu chuyển công tắc về MANUAL (DI6=0), sau đó chuyển ngược về AUTO (DI6=1) để tiếp tục."`
    Xe không nhận bất kỳ lệnh ga nào từ máy tính!
  - **Tại bước 3**: Hệ thống ghi nhận chu trình: `estop_cycle_saw_manual_ = true`.
  - **Tại bước 4**: Khi gạt lại AUTO, FSM lập tức kích hoạt `EvEStopReset`, chuyển trạng thái từ `STOP` -> `WAIT_HOMING`. Động cơ phanh ODrive tự động chạy chu trình homing chạm cảm biến tiệm cận để thiết lập lại điểm gốc nhả phanh 0.0 vòng. Khi hoàn tất homing, FSM chuyển tiếp sang `READY`/`RUNNING`. Xe sẵn sàng hoạt động an toàn trở lại.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Xe kiên quyết từ chối chạy khi chỉ nhả nút E-Stop mà không qua chu trình gạt công tắc; chỉ phục hồi khi hoàn tất chu trình MANUAL -> AUTO.
  - **FAIL**: Xe tự động chuyển sang READY và rồ ga ngay sau khi vừa xoay nhả nút E-Stop mà không cần thao tác gạt công tắc.

#### TC-06-04: Phục hồi E-Stop trực tiếp ở chế độ MANUAL
- **Mục tiêu**: Kiểm tra tính năng phục hồi nhanh khi điều khiển thủ công bằng tay.
- **Tiền điều kiện**: Công tắc chế độ xe ở vị trí **MANUAL (DI6 = 1)**. E-Stop đang nhấn. Xe dừng yên.
- **Các bước thực hiện**:
  1. Xoay nhả nút E-Stop cơ khí.
- **Hành vi phần cứng & phần mềm kỳ vọng**:
  - Do đang ở chế độ MANUAL, hệ thống không đòi hỏi chu trình xoay vòng công tắc.
  - FSM chuyển từ `STOP` -> `WAIT_HOMING`, động cơ phanh ODrive chạy chu trình homing về cảm biến tiệm cận, sau đó chuyển sang `READY`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: FSM chuyển sang `WAIT_HOMING`, thực hiện homing phanh và chuyển sang `READY` sau khi nhả nút E-Stop ở chế độ MANUAL.
  - **FAIL**: Hệ thống bị treo ở `STOP` không phục hồi được.

---

### NHÓM 7: CHUYỂN CHẾ ĐỘ LÁI (AUTO / MANUAL INTERLOCK)

#### TC-07-01: Chuyển chế độ Auto <-> Manual khi đứng yên
- **Mục tiêu**: Kiểm tra chuyển đổi mượt mà giữa chế độ tự hành (AUTO) và chế độ lái tay (MANUAL) khi xe dừng tĩnh.
- **Tiền điều kiện**: Xe đứng yên, vận tốc = 0.0 km/h.
- **Các bước thực hiện**:
  1. Gạt công tắc từ AUTO (DI6 = 0) sang MANUAL (DI6 = 1).
  2. Gửi lệnh ga từ ROS 2: `speed_percentage = 40.0%`.
  3. Gạt công tắc ngược lại từ MANUAL sang AUTO.
- **Hành vi phần cứng & phần mềm kỳ vọng**:
  - Tại bước 1: FSM ghi nhận `Drive mode transitioned to: MANUAL`.
  - Tại bước 2: Lệnh ga từ ROS 2 bị **KHÓA CHẶT Ở 0.0V**, xe không nhúc nhích.
  - Tại bước 3: FSM ghi nhận `Drive mode transitioned to: AUTO`, xe sẵn sàng nhận lệnh tự hành.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Chuyển chế độ thành công; ở MANUAL lệnh ga tự hành bị triệt tiêu hoàn toàn.
  - **FAIL**: Chế độ MANUAL vẫn cho phép ROS 2 nạp ga chạy xe.

#### TC-07-02: Từ chối chuyển chế độ khi xe đang di chuyển
- **Mục tiêu**: Xác thực tính năng bảo vệ an toàn: Tuyệt đối không cho phép đổi quyền điều khiển (cướp lái hoặc thả trôi) đột ngột khi xe đang chạy ở tốc độ cao (> 0.5 km/h).
- **Tiền điều kiện**: Xe kê kích nâng bánh, đang chạy ở chế độ AUTO với tốc độ quay bánh đo được > 0.5 km/h (hoặc mô phỏng vận tốc CAN1).
- **Các bước thực hiện**:
  1. Trong lúc bánh xe đang quay, gạt công tắc trên xe sang MANUAL.
- **Hành vi phần cứng & phần mềm kỳ vọng**:
  - Hệ thống từ chối lệnh chuyển chế độ (REJECT). Log HSH cảnh báo:
    `"Mode switch to MANUAL REJECTED: vehicle is moving (speed: ... km/h > threshold 0.50 km/h)"`.
  - Quyền điều khiển và trạng thái hiện tại được giữ nguyên cho tới khi người vận hành hãm dừng xe hoàn toàn.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Không xảy ra chuyển đổi chế độ khi xe đang chuyển động.
  - **FAIL**: Hệ thống cho phép chuyển chế độ làm ngắt ga hoặc đảo rơle đột ngột khi đang chạy.

---

### NHÓM 8: XỬ LÝ LỖI, WATCHDOG & MẤT TÍN HIỆU (FAULT & WATCHDOGS)

#### TC-08-01: Watchdog mất lệnh điều khiển ROS 2 (Timeout 250ms)
- **Mục tiêu**: Xác nhận nếu máy tính điều khiển cấp cao (Nav2 / Planner / Teleop) bị treo hoặc đứt cáp mạng không gửi lệnh trong vòng 250ms, bộ điều khiển cấp thấp phải tự động cắt ga và đưa xe về trạng thái an toàn.
- **Tiền điều kiện**: Xe đang chạy số Tiến (30% ga) nhận lệnh định kỳ 20Hz.
- **Các bước thực hiện**:
  1. Nhấn `Ctrl + C` tắt đột ngột tiến trình gửi lệnh `motion_cmd`.
  2. Bấm giờ quan sát thời điểm xe cắt ga.
- **Hành vi phần cứng kỳ vọng**:
  - Đúng **250ms** sau khi tắt tiến trình, rơle động lực ngắt, ga rớt về 0V, bánh xe dừng lại.
- **Hành vi phần mềm kỳ vọng**:
  - Topic `/vehicle/state/system` chuyển cờ `command_timed_out = true`, `safe_command_active = true`.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Xe ngắt ga và vào trạng thái an toàn trong vòng 250ms - 300ms.
  - **FAIL**: Xe tiếp tục chạy trớn hoặc giữ nguyên mức ga cũ (runaway).

#### TC-08-02: Watchdog mất gói tin vận tốc CAN1 (> 500ms)
- **Mục tiêu**: Đảm bảo an toàn khi mất cảm biến tốc độ khung gầm xe.
- **Tiền điều kiện**: Xe đang hoạt động, CAN1 truyền dữ liệu tốc độ định kỳ.
- **Các bước thực hiện**:
  1. Rút cáp đường `can1` ra khỏi Jetson/PC.
- **Hành vi phần mềm kỳ vọng**:
  - Sau 500ms không có frame `0x751`, HSH cảnh báo:
    `"CAN1 vehicle speed telemetry stale (> 500ms). Resetting speed to 0.0..."`.
  - Hệ thống xử lý an toàn, không để dữ liệu tốc độ cũ gây treo logic dừng xe.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Hệ thống phát hiện dữ liệu cũ đúng 500ms và ghi nhận cảnh báo.
  - **FAIL**: Treo tiến trình hoặc đọc dữ liệu rác.

#### TC-08-03: Xử lý ngắt kết nối đường truyền RS485 Modbus
- **Mục tiêu**: Kiểm tra hành vi khi đường truyền cáp RS485 bị đứt hoặc lỏng giắc trong quá trình xe vận hành.
- **Tiền điều kiện**: Xe đang chạy ở số Tiến (Ga 30% = 1.5V).
- **Các bước thực hiện**:
  1. Rút cáp USB-RS485 ra khỏi cổng USB của máy tính.
- **Hành vi phần cứng kỳ vọng**:
  - Mạch ngõ ra AO8 (nếu mất nguồn hoặc mất lệnh) lập tức mất điện áp, ga rớt về 0V.
- **Hành vi phần mềm kỳ vọng**:
  - Node `rs485_modbus_interface` đếm 10 lần lỗi liên tiếp (`consecutive_failures > 10`), ghi log:
    `"Lost communication on ... Re-probing serial ports..."`.
  - Hệ thống tự động chuyển sang vòng lặp quét tìm lại cổng mà không bị crash/segmentation fault.
  - Cắm lại cáp USB: Hệ thống tự động nhận diện lại và khôi phục kết nối.
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Không bị crash tiến trình ROS 2; ga bị ngắt an toàn; tự động kết nối lại khi cắm giắc.
  - **FAIL**: Tiến trình ros2_control bị văng (crash) hoặc rồ ga mất kiểm soát.

#### TC-08-04: Giám sát tải bus CAN0 và cơ chế chống tràn ACK
- **Mục tiêu**: Kiểm tra tính năng chống tràn băng thông CAN bus (Anti-flooding): Khi trạng thái rơle thực tế từ ESP32 đã khớp với lệnh điều khiển mong muốn, driver `relay_can_interface` phải ngừng gửi frame định kỳ lặp lại vô ích.
- **Tiền điều kiện**: Hệ thống đang chạy ổn định.
- **Các bước thực hiện**:
  1. Dùng lệnh `canbusload can0@500000` đo tải bus.
  2. Bật một relay bất kỳ và giữ nguyên trong 10 giây.
- **Hành vi phần mềm kỳ vọng**:
  - Driver gửi 1 frame lệnh `0x500`.
  - Khi nhận frame `0x580` phản hồi trùng khớp (`fb == mask`), driver ghi nhận ACK và **DỪNG GỬI** ngay lập tức.
  - Tải bus CAN0 giữ ở mức rất thấp (< 25% - 30%).
- **Tiêu chuẩn Pass/Fail**:
  - **PASS**: Không có bão frame CAN `0x500`, tải bus CAN luôn dưới 30%.
  - **FAIL**: Frame `0x500` bị bắn liên tục hàng nghìn gói mỗi giây làm nghẽn bus CAN ODrive.

---

## 4. BIÊN BẢN ĐÁNH GIÁ & TIÊU CHUẨN KÝ DUYỆT (SIGN-OFF CRITERIA)

Một phiên kiểm thử thực tế xe chỉ được coi là **ĐẠT ĐIỀU KIỆN VẬN HÀNH THỰC ĐỊA** khi thỏa mãn:
1. **100% các Test Case thuộc nhóm An Toàn Cao & Cực Kỳ Quan Trọng** (TC-01-03, TC-02-01, TC-02-05, TC-03-04, TC-06-01, TC-06-03, TC-07-02, TC-08-01) phải đạt kết quả **PASS**.
2. Không có bất kỳ lỗi rơle kẹt dính (welded contact) hoặc xung đột lệnh ga nào trong suốt quá trình thử nghiệm.
3. Toàn bộ các cơ chế phục hồi E-Stop và chuyển chế độ Auto/Manual tuân thủ nghiêm ngặt quy trình chốt an toàn liên động.

