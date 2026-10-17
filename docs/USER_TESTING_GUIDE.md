# CẨM NANG HƯỚNG DẪN KIỂM THỬ THỰC TẾ XE DÀNH CHO NGƯỜI DÙNG (USER STEP-BY-STEP TESTING GUIDE)

Tài liệu này hướng dẫn chi tiết từng bước cho người vận hành/kỹ sư kiểm thử thực tế xe: từ khâu chuẩn bị phần cứng, các phím bấm điều khiển, thao tác gạt công tắc/ấn nút trên xe, cho đến những hiện tượng thực tế cần quan sát bằng mắt và nghe bằng tai.

---

## 🛑 BƯỚC 0: CHUẨN BỊ AN TOÀN & KHỞI ĐỘNG (BẮT BUỘC)

### 1. Chuẩn bị xe trên kích nâng (Jack-up)
- **Kê kích nâng cao xe**: Nâng cả **bánh dẫn động** và **bánh lái** khỏi mặt đất. Bánh xe phải quay tự do trong không khí để đảm bảo an toàn tuyệt đối nếu có sự cố rồ ga ngoài ý muốn.
- **Vị trí người kiểm thử**: 
  - 1 người ngồi trước máy tính (điều khiển bàn phím và xem log).
  - 1 người đứng cạnh xe, **tay đặt sẵn sát Nút Dừng Khẩn Cấp E-Stop (nút nấm màu đỏ)**.

### 2. Kiểm tra cắm cáp & cấp nguồn
- Cắm cổng USB-RS485 vào máy tính Jetson/PC.
- Cắm cáp CAN (`can0` nối ODrive + board ESP32; `can1` nối đường telemetry xe).
- Bật công tắc nguồn động lực và nguồn điều khiển 24V/48V.
- Kiểm tra công tắc chế độ trên xe đang ở vị trí **AUTO** (thông COM / DI6 = 0).
- Xoay nhả nút **E-Stop đỏ** (nút bật lên, DI7 = 0).

### 3. Bật hệ thống phần mềm
Mở **Terminal 1** và khởi chạy hệ thống điều khiển phần cứng:
```bash
source /opt/ros/humble/setup.bash
source /home/ddon/robot_hardware_ws/install/setup.bash
ros2 launch vehicle_controller vehicle_hardware_bringup.launch.py
```
> **Dấu hiệu bình thường**: Đèn trên 2 bộ ODrive nháy xanh. Log xuất hiện: `HSH node initialized`, `Drive mode transitioned to: AUTO`. FSM chuyển sang `WAIT_HOMING` -> Động cơ phanh ODrive tự động chạy chu trình homing chạm cảm biến tiệm cận để xác định điểm gốc 0.0 vòng -> Sau khi homing hoàn tất, FSM chuyển sang `Ready`.

Mở **Terminal 2** và bật công cụ điều khiển bàn phím (Keyboard Teleop):
```bash
source /home/ddon/robot_hardware_ws/install/setup.bash
ros2 run vehicle_keyboard_teleop vehicle_keyboard_teleop
```
> Màn hình sẽ hiện giao diện:
> ```text
> Gear=DRIVE   | Ga=  0.0% | Lái= +0.0 vòng | Phanh=+0.0 turns | Motion=OFF | Estop=OFF | AuxRelay=[0x00]
> ```

---

## 📋 BẢNG PHÍM BẮT ĐIỀU KHIỂN TELEOP

| Phím | Chức năng | Hành vi thực tế trên xe |
| :---: | :--- | :--- |
| **`E`** | **Bật / Tắt Cho Phép Chạy (Motion Enable)** | Bật/tắt cấp phép điều khiển động cơ. |
| **`W`** | **Tăng Ga (+5%)** | Tăng điện áp ra bộ điều khiển truyền động. |
| **`S`** | **Giảm Ga (-5%)** | Giảm điện áp ga. |
| **`R`** | **Vào số Tiến (DRIVE)** | Chuẩn bị chuỗi rơle chạy tiến. |
| **`F`** | **Vào số Lùi (REVERSE)** | Chuẩn bị chuỗi rơle chạy lùi. |
| **`N`** | **Về số Mo (NEUTRAL)** | Ngắt toàn bộ rơle truyền động, xe trôi tự do. |
| **`D`** | **Đánh lái sang Phải (+2 vòng)** | Động cơ ODrive quay bánh lái sang phải. |
| **`A`** | **Đánh lái sang Trái (-2 vòng)** | Động cơ ODrive quay bánh lái sang trái. |
| **`C`** | **Trả thẳng lái (0 vòng)** | Động cơ ODrive tự động quay thẳng bánh lái về tâm. |
| **`Space`** | **Đạp Phanh (3 mức: +1, +2, +3 vòng)** | Động cơ ODrive siết dây cáp kẹp má phanh. |
| **`Z`** | **Nhả Phanh (Về 0 vòng / IDLE)** | Thả lỏng cáp, lò xo kéo nhả má phanh. |
| **`1` .. `5`** | **Bật/Tắt Relay Phụ 1 đến 5** | Bật tắt Đèn, Còi, Xi-nhan, Đèn cảnh báo. |
| **`X`** | **E-Stop phần mềm** | Khóa dừng khẩn cấp toàn bộ hệ thống ngay trên phím. |
| **`Q`** | **Thoát an toàn** | Đưa toàn bộ hệ thống về 0, nhả rơle và tắt chương trình. |

---

## 🚗 QUY TRÌNH KIỂM THỬ THỰC TẾ TỪNG TÍNH NĂNG

---

### BÀI 1: KIỂM THỬ HỆ THỐNG LÁI (STEERING)

*Mục tiêu: Đảm bảo bánh lái quay đúng hướng, mượt mà và tự dừng an toàn khi chạm cữ biến trở.*

1. **Bật Motion**: Nhấn phím **`E`** (Màn hình hiện `Motion=ON`).
2. **Đánh lái sang phải**:
   - Nhấn phím **`D`** nhiều lần.
   - **Quan sát thực tế**: Bánh lái xoay đều đặn sang bên phải. Không có tiếng giật cục. Màn hình teleop hiển thị số vòng tăng dần: `+2.0`, `+4.0`, ..., `+35.0` (nửa hành trình).
3. **Thử giới hạn an toàn cữ phải**:
   - Tiếp tục nhấn **`D`** cho đến khi đạt mức kịch phải (khoảng `+70.0 vòng`).
   - **Quan sát**: Cơ cấu lái tự động dừng lại khi biến trở đạt ngưỡng 85% hành trình (~69%). Động cơ ODrive dừng quay êm, không gầm rú ép cữ cơ khí.
4. **Đánh lái sang trái**:
   - Nhấn phím **`A`** liên tục: Bánh lái đổi chiều, quay mượt mà sang bên trái qua tâm 0 vòng và sang mức âm (`-35.0`, `-70.0 vòng`).
5. **Trả thẳng lái**:
   - Nhấn phím **`C`**.
   - **Quan sát**: Bánh lái tự động quay đều về đúng vị trí thẳng bánh ban đầu (`Lái= +0.0 vòng`).

---

### BÀI 2: KIỂM THỬ HỆ THỐNG PHANH ĐIỆN (BRAKE)

*Mục tiêu: Đảm bảo cơ cấu kéo phanh siết chặt khi có lệnh và nhả hoàn toàn khi ở mức 0.*

1. **Kiểm tra trạng thái nhả phanh (0%)**:
   - Bấm phím **`Z`** (Màn hình báo `Phanh=+0.0 turns`).
   - Dùng tay thử quay bánh dẫn động: Bánh xe quay nhẹ nhàng, má phanh không chạm đĩa.
2. **Đạp phanh mức 1 (+1.0 vòng)**:
   - Bấm phím **`Space`** 1 lần (`Phanh=+1.0 turns`).
   - **Quan sát**: Động cơ ODrive kéo cáp phanh tì nhẹ vào đĩa.
3. **Đạp phanh mức 2 (+2.0 vòng)**:
   - Bấm phím **`Space`** lần 2 (`Phanh=+2.0 turns`).
   - Dùng tay xoay thử: Lực cản tăng rõ rệt.
4. **Phanh khẩn cấp / Đỗ (+3.0 vòng - 100%)**:
   - Bấm phím **`Space`** lần 3 (`Phanh=+3.0 turns`).
   - **Quan sát**: Cáp phanh căng hết cỡ, má phanh siết cứng đĩa, không thể dùng tay quay bánh xe.
5. **Nhả phanh**:
   - Bấm phím **`Z`**.
   - **Quan sát**: Động cơ phanh nhả về IDLE ngay tức khắc, lò xo kéo má phanh mở bung ra hoàn toàn. Bánh xe quay tự do trở lại.

---

### BÀI 3: KIỂM THỬ VẬN HÀNH TRUYỀN ĐỘNG (TIẾN / LÙI / MO)

*Mục tiêu: Đảm bảo chuỗi trễ bảo vệ rơle (Relay 6/7 -> Relay 8 -> Nạp ga) hoạt động đúng.*

#### 3.1. Chạy số Tiến (DRIVE)
1. Đảm bảo phanh đã nhả (phím **`Z`**).
2. Nhấn phím **`R`** để chọn số Tiến (`Gear=DRIVE`).
3. Nhấn phím **`W`** 3 lần để đặt ga lên 15% (`Ga= 15.0%`).
4. **Lắng nghe & Quan sát**:
   - Nghe tiếng "tạch" thứ nhất: Relay 6 đóng (đèn LED 6 trên mạch ESP32 sáng).
   - Đúng **100ms** sau, nghe tiếng "tạch" thứ hai: Relay 8 (nguồn chính) đóng (LED 8 sáng).
   - Đúng **100ms** sau nữa: Bánh xe bắt đầu quay êm về phía trước!
   - Nhấn **`W`** tăng lên 30%, 50%: Bánh xe tăng tốc mượt mà.

#### 3.2. Về số Mo (NEUTRAL)
1. Đang chạy, nhấn phím **`N`** (`Gear=NEUTRAL`).
2. **Quan sát**: Nghe tiếng rơle nhả ngay lập tức. Động cơ mất điện áp ga (về 0V) và bánh xe quay trôi tự do chậm dần đến khi dừng hẳn.

#### 3.3. Chạy số Lùi (REVERSE)
1. Bấm phím **`F`** để chọn số Lùi (`Gear=REVERSE`).
2. Bấm phím **`W`** đặt ga 15%.
3. **Lắng nghe & Quan sát**:
   - Relay 7 (Lùi) đóng trước (LED 7 sáng) $\rightarrow$ Trễ 100ms $\rightarrow$ Relay 8 đóng $\rightarrow$ Bánh xe quay theo chiều lùi.

#### 3.4. Thử nghiệm khóa an toàn đảo chiều (CỰC KỲ QUAN TRỌNG)
1. Cho bánh xe chạy số Tiến ở tốc độ khá nhanh (Ga 40%).
2. **Bất ngờ bấm phím `F` (chuyển sang Lùi) ngay khi bánh xe vẫn đang quay tít**:
3. **Hiện tượng bắt buộc phải xảy ra**:
   - Toàn bộ rơle 6, 8 ngắt ngay lập tức, ga rớt về 0V.
   - **Relay 7 (Lùi) TUYỆT ĐỐI KHÔNG ĐƯỢC PHÉP ĐÓNG** trong lúc bánh xe vẫn còn đang quay trớn!
   - Chờ bánh xe quay chậm dần cho đến khi dừng hẳn: Relay 7 mới bắt đầu tạch đóng $\rightarrow$ Relay 8 đóng $\rightarrow$ Bánh xe mới quay lùi.
   *(Tính năng này bảo vệ bánh răng hộp số không bị giật gãy).*

---

### BÀI 4: KIỂM THỬ RELAY PHỤ (ĐÈN, CÒI, XI-NHAN)

1. Bấm phím **`1`**: Relay 1 đóng $\rightarrow$ Đèn pha bật sáng.
2. Bấm phím **`2`**: Relay 2 đóng $\rightarrow$ Còi / Đèn xi-nhan bật.
3. Bấm phím **`3`**, **`4`**, **`5`**: Kiểm tra các phụ tải tương ứng.
4. Bấm phím **`0`**: Tắt toàn bộ relay phụ.
5. **Kiểm tra tính độc lập**: Trong lúc xe đang chạy tiến (Ga 20%), bấm nhấp nháy phím `1` liên tục. Xác nhận xe vẫn chạy đều, không bị ngắt rơle động lực.

---

### BÀI 5: KIỂM THỬ NÚT DỪNG KHẨN CẤP E-STOP (CỰC KỲ QUAN TRỌNG)

*Mục tiêu: Đảm bảo xe dừng đứng bánh ngay khi có sự cố và không bao giờ tự động phóng đi khi nhả nút E-Stop.*

#### 5.1. Nhấn E-Stop khi đang chạy
1. Bật số Tiến, nhấn ga cho bánh xe quay đều ở mức 40%.
2. Người đứng cạnh xe **ấn mạnh nút E-Stop cơ khí màu đỏ (DI7)** trên vỏ xe.
3. **Hiện tượng bắt buộc phải xảy ra ngay lập tức (< 10ms)**:
   - Toàn bộ Relay 6, 7, 8 nảy nhả ngay tức khắc.
   - Động cơ mất ga hoàn toàn (0V).
   - Động cơ phanh ODrive gầm lên siết cứng 100% (+3.0 vòng) ghì chặt đĩa phanh.
   - Động cơ lái ODrive thả lỏng về IDLE (tránh kẹt góc lái).
   - Màn hình Terminal 1 báo FSM chuyển sang `Stop`.

#### 5.2. Thử nghiệm cơ chế khóa an toàn phục hồi (Runaway Prevention)
1. Giữ nguyên công tắc chế độ ở vị trí **AUTO**.
2. **Xoay nhả nút E-Stop đỏ** (nút nảy lên trở lại).
3. **Quan sát hành vi**:
   - Phanh nhả về IDLE.
   - **XE TUYỆT ĐỐI KHÔNG ĐƯỢC PHÉP CHẠY LẠI** dù lệnh ga trên máy tính vẫn còn!
   - FSM vẫn kiên quyết nằm ở `Stop`. Log HSH báo:
     *"E-Stop đã nhả nhưng xe đang ở chế độ AUTO. Yêu cầu chuyển công tắc về MANUAL (DI6=0), sau đó chuyển ngược về AUTO (DI6=1) để tiếp tục."*
4. **Thao tác phục hồi chuẩn**:
   - Người đứng cạnh xe gạt công tắc sang **MANUAL** $\rightarrow$ Sau đó gạt ngược lại sang **AUTO**.
   - **Hiện tượng**: FSM chuyển sang `WAIT_HOMING` -> Động cơ phanh ODrive tự động thực hiện chu trình homing chạm cảm biến tiệm cận để hiệu chuẩn lại điểm gốc nhả phanh 0.0 vòng -> Khi homing hoàn tất, FSM chuyển sang `Ready`/`Running`! Xe sẵn sàng hoạt động an toàn trở lại.

---

### BÀI 6: KIỂM THỬ CÔNG TẮC AUTO / MANUAL

1. **Khi xe đang đứng yên**:
   - Gạt công tắc sang **MANUAL**: Thử bấm phím `W` tăng ga trên máy tính $\rightarrow$ Xe không nhúc nhích, ga bị khóa ở 0.0V.
   - Gạt công tắc về **AUTO**: Bấm ga $\rightarrow$ Xe chạy bình thường.
2. **Khi xe đang di chuyển**:
   - Bấm ga cho bánh xe quay (~2-3 km/h).
   - Gạt công tắc sang **MANUAL**.
   - **Quan sát log**: Hệ thống báo chối từ:
     *"Mode switch to MANUAL REJECTED: vehicle is moving"*.
   - Xe giữ nguyên trạng thái cho đến khi hãm dừng hẳn mới cho phép đổi quyền điều khiển.

---

### BÀI 7: KIỂM THỬ MẤT TÍN HIỆU ĐIỀU KHIỂN (WATCHDOG TIMEOUT)

1. Cho bánh xe chạy số Tiến ở mức 30% ga.
2. Tại **Terminal 2**, nhấn tổ hợp phím **`Ctrl + C`** để tắt đột ngột chương trình teleop (mô phỏng tình huống máy tính bị đơ hoặc đứt cáp mạng điều khiển).
3. **Quan sát**: Đúng **0.25 giây (250ms)** sau khi tắt chương trình, rơle động lực nảy cạch ngắt nguồn, điện áp ga rớt về 0V, bánh xe dừng lại an toàn!

---

## 🏁 BƯỚC CUỐI: THỬ NGHIỆM THỰC ĐỊA TRÊN ĐƯỜNG (TRACK TEST)

Sau khi **100% các bài test kê kích trên giá nâng đều ĐẠT (PASS)**:
1. Hạ xe xuống mặt đất tại khu vực bãi thử kín, bằng phẳng, không có chướng ngại vật.
2. Người cầm bàn phím/laptop đi bộ bám sát phía sau xe (khoảng cách 2 mét).
3. Người thứ hai luôn đi bộ song song sát nút E-Stop trên thân xe.
4. Bắt đầu chạy thử với tốc độ chậm (Ga 10% - 15%):
   - Chạy thẳng 10 mét $\rightarrow$ Bấm `N` dừng lại $\rightarrow$ Lùi 5 mét.
   - Bẻ lái tránh vật cản sang phải/trái.
   - Thử nghiệm đạp phanh dừng xe.
   - Thử nghiệm ấn nút E-Stop khi xe đang lăn bánh thực tế trên mặt đất.

---
*(Chúc mừng bạn đã nắm vững toàn bộ quy trình kiểm thử thực tế xe an toàn và chuẩn xác!)*

