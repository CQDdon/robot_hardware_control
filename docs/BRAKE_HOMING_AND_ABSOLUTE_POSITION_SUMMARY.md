# TỔNG KẾT TÍNH NĂNG: HOMING CẢM BIẾN TIỆM CẬN & ĐIỀU KHIỂN PHANH TUYỆT ĐỐI CHO ĐỘNG CƠ ODRIVE

Tài liệu này ghi lại toàn bộ nội dung thảo luận, yêu cầu kỹ thuật, quyết định thiết kế và mã nguồn đã triển khai trong phiên làm việc về tích hợp **Cảm biến tiệm cận cho phanh**, chu trình **Homing ODrive** và chuyển đổi sang **Điều khiển vị trí tuyệt đối (Absolute Position Control)**.

---

## 1. Yêu cầu & Bối cảnh kỹ thuật

1. **Sử dụng cảm biến tiệm cận cho phanh:**
   - Cài đặt cảm biến tiệm cận làm công tắc hành trình (endstop / homing sensor) tại vị trí nhả phanh hoàn toàn ($0\%$ phanh).
   - Tự động kích hoạt tính năng **Homing của ODrive** khi khởi động chương trình và **mỗi khi chạy trở lại sau khi dừng khẩn cấp (E-Stop recovery)**.

2. **Chuyển sang Điều khiển Vị trí Tuyệt đối (Absolute Position):**
   - **Vấn đề trước đây:** Chưa có cảm biến tiệm cận, mốc encoder phụ thuộc vào vị trí lúc bật nguồn (arbitrary zero). Hệ thống phải liên tục "chốt" vị trí hiện tại khi nhả phanh (`position_reference_rad_ = feedback`) và cộng dồn khoảng dời. Nếu lò xo hồi vị chưa kéo hết cơ cấu phanh về mốc cơ học, sai số sẽ bị tích lũy (drift). Khi đạp phanh sâu hoặc E-Stop, động cơ có thể quay vượt quá $3.5\text{ vòng}$ dẫn tới đứt cáp kéo phanh.
   - **Giải pháp:** Sử dụng cảm biến tiệm cận để xác định điểm $0.0\text{ vòng}$ ($0.0\text{ rad}$) cố định. Mọi lệnh điều khiển phanh từ ROS2 controller và HSH đều quy về **tọa độ tuyệt đối** tính từ mốc Homing này.

---

## 2. Kiến trúc & Thiết kế Trạng thái FSM

### 2.1. Luồng FSM khi Khởi động
```
[INIT] ---> [WAIT_SENSORS] ---> [WAIT_HOMING] ---> [READY] ---> [RUNNING]
```
- Tại `WAIT_HOMING`:
  - Trục phanh (Node 4 / Axis 0): chuyển sang `AxisDesiredState::Homing`.
  - Các trục khác (trục lái, bánh xe): giữ ở trạng thái `AxisDesiredState::Idle`.
  - Tay ga (RS485): khóa ở $0.0\text{V}$.
  - Rơ le (CAN Relay): ngắt toàn bộ (`relay_mask = 0x00`).
  - Khi ODrive chạm cảm biến tiệm cận và hoàn tất homing (về `IDLE`), FSM phát sự kiện `EvHomingDone` để chuyển sang `READY`.

### 2.2. Luồng FSM khi Phục hồi sau E-Stop
```
[RUNNING] ---> [STOP (E-Stop)] ---> [WAIT_HOMING] ---> [READY] ---> [RUNNING]
```
- Khi xảy ra sự cố khẩn cấp (nút E-Stop DI7 hoặc lỗi nghiêm trọng):
  - FSM chuyển sang `STOP`.
  - Động cơ phanh được kéo ngay lập tức đến **vị trí tuyệt đối $3.0\text{ vòng}$** ($100\%$ phanh).
  - Khóa ga $0.0\text{V}$, tắt rơ le, chuyển bánh xe sang `IDLE`.
- Khi nhả nút E-Stop và thực hiện quy trình an toàn (chuyển MANUAL hoặc xoay chu kỳ MANUAL $\rightarrow$ AUTO):
  - FSM **không** nhảy trực tiếp vào `READY`, mà chuyển sang `WAIT_HOMING`.
  - Trục phanh tự động homing lại về cảm biến tiệm cận để hiệu chuẩn lại điểm 0 chuẩn xác, loại bỏ hoàn toàn biến dạng hoặc trượt cáp nếu có trong quá trình phanh khẩn cấp.
  - Sau khi homing xong $\rightarrow$ FSM chuyển sang `READY` $\rightarrow$ `RUNNING`.

---

## 3. Chi tiết Mã nguồn Đã Thay đổi

### 3.1. `HardwareSystemHandler`
- [`state_stop.hpp`](file:///home/ddon/robot_hardware_ws/src/robot_hardware_control/src/HardwareSystemHandler/include/hardware_system_handler/state_stop.hpp):
  - Thay đổi sự kiện `react(EvEStopReset const&)` từ chuyển trực tiếp sang `READY` thành `transit<WAIT_HOMING>()`.
- [`state_wait_homing.hpp`](file:///home/ddon/robot_hardware_ws/src/robot_hardware_control/src/HardwareSystemHandler/include/hardware_system_handler/state_wait_homing.hpp):
  - Bổ sung `react(EvCriticalError const&) override { transit<STOP>(); }` bảo vệ an toàn nếu có E-Stop trong lúc đang homing.
- [`hsh_node.cpp`](file:///home/ddon/robot_hardware_ws/src/robot_hardware_control/src/HardwareSystemHandler/src/hsh_node.cpp):
  - Reset cờ homing (`homing_required_`, `homing_command_sent_`, `homing_started_`, `homing_done_`) mỗi khi vào trạng thái `FsmState::WaitHoming`.
  - `ControlCommand()`:
    - Nhánh `current_state == FsmState::WaitHoming`: Đảm bảo chỉ trục phanh nhận lệnh Homing, tay ga $0\text{V}$, relay $0\text{x00}$, không bị đè bởi lệnh chu kỳ `cmd_if`.
    - Lệnh phanh khẩn cấp (E-Stop): Chốt cố định `estop_brake_target_[0] = 3.0f` vòng (thay vì vị trí tương đối).
    - Loại bỏ biến `brake_reference_turns_` và mốc tương đối.
    - Thêm lớp bảo vệ giới hạn an toàn tuyệt đối (clamp) cho trục phanh:
      ```cpp
      if (is_brake_axis) {
          target = std::clamp(target, 0.0f, 3.0f);
      }
      ```
  - Khi Homing hoàn tất: Đảm bảo `ctrl->motion_enable = false` trong `READY`, chỉ kích hoạt `motion_enable = true` khi FSM thực sự bước vào `RUNNING`.

### 3.2. `vehicle_controller`
- [`vehicle_controller.cpp`](file:///home/ddon/robot_hardware_ws/src/robot_hardware_control/src/vehicle_controller/src/vehicle_controller.cpp):
  - Chuyển đổi phương thức `set_position_command()` sang tính vị trí tuyệt đối từ gốc $0.0\text{ rad}$:
    $$\text{target\_rad} = \text{position\_max\_offset\_turns\_} \times \frac{\text{requested\_percentage}}{100.0} \times 2\pi$$
  - Loại bỏ hoàn toàn việc đọc `state_interfaces_` để gán `position_reference_rad_`.
  - Phanh $0\%$ gửi lệnh $0.0\text{ rad}$ và chuyển trục sang `IDLE`.
  - Phanh $100\%$ gửi lệnh $3.0\text{ vòng} \times 2\pi = 6\pi\text{ rad} \approx 18.8495\text{ rad}$.

### 3.3. Cấu hình Launch
- [`vehicle_hardware_bringup.launch.py`](file:///home/ddon/robot_hardware_ws/src/robot_hardware_control/src/vehicle_controller/launch/vehicle_hardware_bringup.launch.py):
  - Chuyển tham số `bypass_position_homing` từ `True` sang `False` để luôn thực hiện chu trình Homing cảm biến tiệm cận khi khởi động thật trên xe.

---

## 4. Kiểm thử & Đánh giá (Verification)

### 4.1. Bộ test hành vi hệ thống (`test_system_behavior`)
- Đã chạy 8 kịch bản kiểm thử tích hợp phần cứng ảo qua Shared Memory:
  - Khởi động FSM & chuyển đổi chế độ Manual / Auto.
  - Kích hoạt E-Stop trên DI7 $\rightarrow$ Khóa phanh $3.0\text{ vòng}$, ngắt ga $0\text{V}$, tắt rơ le.
  - Nhả E-Stop khi đang ở AUTO $\rightarrow$ Khóa an toàn, yêu cầu gạt về MANUAL trước.
  - Phục hồi sau E-Stop trong MANUAL $\rightarrow$ Chuyển qua `WAIT_HOMING` $\rightarrow$ `READY`.
  - Watchdog kiểm tra timeout tốc độ CAN1 (500ms).
- **Kết quả:** `8/8 PASS (100%)`.

### 4.2. Bộ test homing (`test_homing_behavior`)
- Được xây dựng riêng tại [`test_homing_behavior.cpp`](file:///home/ddon/robot_hardware_ws/src/robot_hardware_control/src/HardwareSystemHandler/src/test_homing_behavior.cpp):
  - Mô phỏng chu trình Homing lúc khởi động (Startup Homing).
  - Mô phỏng chu trình dừng khẩn cấp E-Stop và Homing tự động khi phục hồi (Post-E-Stop Recovery Homing).
- **Kết quả:** `ALL PASS (100%)`.

### 4.3. Bộ test ROS2 Controller (`test_vehicle_controller_sequence`)
- Kiểm thử luồng điều khiển vị trí tuyệt đối và chuyển trạng thái của `vehicle_controller`.
- **Kết quả:** `9/9 PASS (100%)`.

