# Giám sát encoder theo ý định chuyển động

Source: sketch `.ino` và `RobotTypes.h` cùng thư mục. Không đổi GPIO, PPR,
đường kính bánh, hình học né, bộ lọc sonar hay thuật toán PD line.

## Nguyên nhân báo kẹt sai

Watchdog cũ đã bỏ qua PWM <40; không đúng nếu nói PWM=0 luôn báo stall.
Tuy nhiên, PWM đầu ra 40–60 trên bánh trong cua chưa chắc tạo một xung trong
1.5 giây: cấu hình hiện tại cho khoảng 1.021 cm/xung. Hơn nữa, watchdog được
gọi trước khi tính lệnh mới nên có thể kiểm tra lệnh cũ ngay khi cần giảm tốc,
dừng chờ hoặc chuyển từ đi thẳng sang cua.

Bản PD hiện tại không nhất thiết ra lệnh dừng bánh trong cua: 100/001 thường
giảm mạnh bánh trái/phải. Không sửa PD thành dừng cứng chỉ để đáp ứng ví dụ.
Monitor hỗ trợ cả PWM mục tiêu bằng 0 và PWM thấp có chủ đích.

## Dữ liệu và thứ tự thực thi

- `targetLeftPWM/targetRightPWM`: lệnh trước ramp/bù encoder, đã áp dụng trần PWM.
- `currentLeft/currentRight`: PWM thực sự xuất ra sau ramp và bù.
- `applyMotorPWM`: chỉ xuất PWM, không ghi đè ý định. `drive` đặt ý định cho
  lệnh trực tiếp; `smoothDrive` lưu ý định rồi tính ramp/bù. Các chỗ giới hạn
  đầu ra line và dừng riêng bánh đã đủ đích dùng applyMotorPWM.
- `updateControl` tính toàn bộ điều kiện/đầu ra, kể cả các nhánh dừng sớm.
  `control` gọi updateControl trước, rồi mới checkEncoderStall. WAIT/STOP mới
  vì thế có hiệu lực trước khi kiểm tra khoảng không xung của lệnh cũ.
- Mỗi bánh có WheelMonitor: target/actual, delta xung, mốc nhận xung, mốc yêu
  cầu chạy, mốc lệnh đáng kể, thời gian không tiến triển, chu kỳ xung quan sát,
  timeout đang dùng, trạng thái và loại chuyển động.

## Chính sách

| Tình huống | Giám sát |
| --- | --- |
| Mục tiêu 0 | INTENTIONALLY_STOPPED; không kiểm tra stall, kể cả PWM còn dư lúc ramp-down |
| PWM mục tiêu hoặc thực tế <40 | LOW_SPEED; không kết luận kẹt chỉ vì không có xung |
| Bánh trong cua nhẹ/gắt, PWM hữu hiệu <65 | LOW_SPEED; cho phép xung thưa; vẫn giám sát bánh ngoài |
| Tăng từ thấp lên đáng kể / đổi chiều | Có cửa sổ tăng tốc mới; đảo chiều qua 0 |
| Đi thẳng 010 | Giám sát cả hai bánh; bắt đầu lại cửa sổ khi trở về thẳng |
| Né/quay tại chỗ | Cả bánh được yêu cầu quay đều được giám sát, timeout có trần chặt hơn |
| Bánh đạt đích encoderMove | Mục tiêu 0, dừng riêng bánh đó; bánh còn lại tiếp tục được giám sát |
| WAIT_CLEARANCE / motor settle | Lệnh 0, không phát sinh stall từ lệnh cũ |
| MANUAL / ABORTED | Không tạo lỗi stall tự động mới; ABORTED vẫn cần S rồi A |

Các trạng thái: IDLE, LOW_SPEED, ACCELERATING, MOVING, STALL_SUSPECTED,
STALL_CONFIRMED. MOVING là lệnh chạy đang được giám sát và chưa vượt cửa sổ
không xung; không phải khẳng định tốc độ vật lý đã được đo liên tục.

Với yêu cầu chuyển động đáng kể, timeout cơ sở phụ thuộc:

`1500 × hệ_số_cm_mỗi_xung × PWM_tham_chiếu / min(|target|, |actual|)`.

Hệ số độ phân giải dùng cm/xung so với 1.021, giới hạn 0.5–2.0. Tham chiếu
70 PWM khi dò line, 85 PWM khi né. Chu kỳ xung đã quan sát trong cửa sổ hiện
tại có thể tăng biên; timeout cuối giới hạn 1–3.5 giây cho FOLLOW_LINE và
1–2.5 giây cho AVOID. Không tăng đồng loạt một timeout cố định cho mọi bánh.

Sau thời gian tăng tốc tối thiểu 300 ms mới đánh giá. Hết cửa sổ không xung
chuyển STALL_SUSPECTED; thêm 200 ms chưa có xung mới xác nhận và ABORTED.
Có xung trong giai đoạn nghi kẹt sẽ xóa nghi ngờ. Bộ đếm xung tích lũy không
bị xóa; chỉ đặt lại mốc/cửa sổ quan sát khi cần.

Một bộ kiểm tra độc lập phát hiện cả xe không di chuyển: khi hai bánh đều có
lệnh khác 0 và PWM thực tế ≥40 mà cả hai không có xung liên tục 2.5 giây, dừng
với VEHICLE_NO_MOTION. Thay đổi mẫu line không reset bộ kiểm tra này; bất kỳ
xung mới hoặc dừng/giảm xuống dưới mức đáng kể sẽ điều chỉnh lại cửa sổ.
Do đó không bỏ watchdog hoàn toàn trong FOLLOW_LINE.

Reset cửa sổ khi về thẳng cũng loại chu kỳ xung chậm của cua khỏi ước lượng.
Chênh lệch xung tích lũy qua cua không bị coi là sai lệch góc. Giữ nguyên
checkTurnEncoderAgreement/turnReached và giới hạn quãng đường/thời gian né.

## Debug và lỗi

D/E in thêm MOTION, TARGET_PWM, ACTUAL_PWM, PULSE_DELTA, STATUS, REQUEST_MS,
NO_PROGRESS_MS và TIMEOUT_MS mỗi bánh. ENCODER MONITOR tự in khi trạng thái
hoặc loại chuyển động đổi, tối đa hai nhóm/giây. Mọi nhóm giữ separator đầu/cuối.
ENCODER STALL ghi snapshot trước khi dừng, lý do LEFT_WHEEL_STALL,
RIGHT_WHEEL_STALL hoặc VEHICLE_NO_MOTION. Sau xác nhận, motor dừng và không
tự chạy lại. Giữ trạng thái monitor đã lỗi để chẩn đoán khi ABORTED.

## Hiệu chỉnh

| Tham số | Mặc định |
| --- | --- |
| ENCODER_STALL_MS | 1500 ms, cơ sở để tính thích nghi |
| ENCODER_STALL_PWM | 40, ngưỡng chuyển động đáng kể |
| ENCODER_STRONG_PWM | 65, ngưỡng áp dụng watchdog riêng cho bánh trong cua |
| ENCODER_START_GRACE_MS | 300 ms |
| ENCODER_STALL_CONFIRM_MS | 200 ms |
| ENCODER_FOLLOW_MAX_MS | 3500 ms |
| ENCODER_AVOID_MAX_MS | 2500 ms |
| ENCODER_NO_MOTION_MS | 2500 ms |
| ENCODER_REFERENCE_PULSE_CM | 1.021 cm, tham chiếu độ phân giải |

Không thay PPR=20 hoặc đường kính=6.5 cm nếu chưa đo. PWM không là vận tốc;
các ngưỡng vẫn cần hiệu chỉnh từ chuyển động thực tế. Encoder D0 một kênh
không đo chiều quay; monitor suy ra hướng yêu cầu từ lệnh, không tuyên bố
phân biệt được trượt bánh hay mọi nguyên nhân không có xung.

## Kiểm thử

`tests/esp32_avoid/encoder_monitor.cpp` bao phủ đủ 15 trường hợp yêu cầu, gồm
bánh trong cua PWM45 > ngưỡng cũ nhưng xung cách 2 giây, bánh dừng theo lệnh,
ramp, giảm tốc, về thẳng, kẹt một bánh trong TURN_OUT, bánh đủ đích trong
FORWARD_CLEAR, cả xe không xung, WAIT trước watchdog, MANUAL/ABORTED và D.
Thử cả PD thật với xung chỉ ở bánh ngoài, sự nghi kẹt được xóa khi xung trở
lại, timeout theo độ phân giải và nhóm log đóng/mở đúng.

Chạy từ hardware (cần g++):

```powershell
g++ -std=c++11 -Wall -Wextra -pedantic -static -Itests/esp32_avoid tests/esp32_avoid/encoder_monitor.cpp -o "$env:TEMP/esp32-encoder-monitor-test.exe"
& "$env:TEMP/esp32-encoder-monitor-test.exe"
```

Ngoài bộ encoder, chạy lại test.cpp và prototype_order.cpp để kiểm tra logic
toàn xe và thứ tự kiểu khi Arduino tự sinh prototype. Đây là kiểm thử host,
không phải biên dịch/nạp ESP32 hay thử nghiệm xe thật. Phiên hiện tại không
có arduino-cli trong PATH và chưa có toolchain ESP32 truy cập được để xác nhận.
