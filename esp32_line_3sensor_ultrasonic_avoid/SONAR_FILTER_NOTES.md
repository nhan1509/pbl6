# Sonar: phép đo, khoảng trống và chính sách chuyển động

Chi tiết bản tối ưu tổng thể, lưu đồ, bảng kiểm thử và hiệu chỉnh:
[CONTROL_REVIEW.md](CONTROL_REVIEW.md). Source hoàn chỉnh nằm trong sketch `.ino`.

## Phân loại

`EchoMeasurement`: UNINITIALIZED, ECHO_VALID, NO_ECHO, SIGNAL_FAULT.
`ClearanceState`: NEAR, CLEAR, UNKNOWN. Thông tin nội bộ SonarStatus phân biệt
phục hồi, chờ và stale của luồng đo. Timeout được đo đều không tự thành LOST.
Không có khoảng cách giả định 200 cm; RAW=NAN khi không có echo hợp lệ.

- TIMEOUT: pulseIn không trả một xung hoàn chỉnh.
- TIMEOUT_WAITING_FALL: ECHO còn HIGH sau 12 ms; có thể là xung xa chưa kết thúc.
- ECHO_BUSY_RETRY: HIGH trước trigger, không phát trigger chồng; thử lại chu kỳ sau.
- ECHO_STUCK_HIGH: HIGH được quan sát kéo dài ít nhất 60 ms qua các vòng loop.
- PULSE_TOO_SHORT: xung dưới khoảng cách tối thiểu 2 cm.
- PULSE_EXCEEDS_TIMEOUT: độ dài bất nhất, vượt thời gian chờ đã cấu hình.
- OUTSIDE_USAGE_RANGE: echo thật >190 cm nhưng trong dải timeout, không phải fault.

Timeout 12000 µs tương ứng khoảng 205.8 cm; không đánh lỗi phần cứng chỉ vì
vượt 190 cm. Không khẳng định từ một timeout có thể biết chắc là vật quá xa.

## Lọc và cảnh báo

Chỉ echo hợp lệ đi vào median3 và EMA 0.55. Một lần timeout không xóa lịch sử
hoặc cập nhật bộ lọc. Giữ MED/EMA/LAST để tham khảo, HELD không xác nhận khoảng
trống. Sau ba timeout liên tiếp hoặc khoảng mất 800 ms, echo thật trở lại sẽ
khởi tạo lại cửa sổ. Phục hồi đòi ít nhất hai echo; raw mới quá gần dừng ngay.
Ba mẫu nhất quán trong 3 cm cho phép rebase EMA nếu còn lệch median hơn 12 cm.

`confirmedClear` cần hai raw mới liên tiếp và EMA đạt đúng ngưỡng đang xét.
Sequence chỉ được đếm khi có phép đo mới; validSequence chỉ tăng khi echo hợp lệ.
`nearLatched` và `cautionLatched` không bị NO_ECHO xóa; front nhả chốt bằng echo
mới ở 13 / 23 cm. Các chốt không được reset chỉ vì gửi lại A. `faultLatched`
giữ thông tin phục hồi, không còn mang nghĩa phải ABORT ngay khi đang dò line.

## FOLLOW_LINE và AVOID

FOLLOW_LINE không yêu cầu hai bên cùng trả echo. Side UNKNOWN không chặn xe;
fault bên gây giới hạn 60 PWM, không chuyển ABORT vì một vài phép đo lỗi. Front
fault dừng thử lại 240 ms; lỗi liên tục giữ dừng, hai echo mới cho tự phục hồi.
Front NO_ECHO không có cảnh báo gần cho phép thăm dò có cấu hình: 60 PWM từ
khởi động, 65 PWM sau lịch sử xa. Nếu đã có cảnh báo gần chưa giải quyết, giữ
dừng và đợi echo thật. Mất line hoặc bảo vệ encoder vẫn có thể dừng xe.

AVOID vẫn cần xác nhận thật. Xoay cần trước và cả hai bên trong vùng quét; pha
tịnh tiến cần trước và phía vật. Phía đối diện UNKNOWN không tạo deadlock khi
đi thẳng nhưng nguy hiểm đo được bên đó vẫn có hiệu lực. Khi thiếu dữ liệu,
dừng chờ và chỉ hủy xác nhận mép có thể đã cũ; giữ việc đã thấy vật. Không dùng
NO_ECHO để xác nhận mép, chọn hướng hoặc hoàn tất BYPASS. Giữ giới hạn timeout,
quãng đường và watchdog encoder.

## Lịch đo và debug

FRONT → LEFT → FRONT → RIGHT, cách lần bắt đầu đo tối thiểu 60 ms. FRONT danh
định 120 ms/lần, mỗi bên 240 ms/lần; stale luồng 800 ms có dư cho lịch bên.
pulseIn có thể chặn vòng chính khoảng 12 ms; encoder ISR vẫn nhận xung. Không
thêm delay dài. Xem CONTROL_REVIEW để biết parser Serial không chặn và TX buffer.

D in snapshot có separator đầu/cuối, tên và loại echo, clearance, REASON,
RAW/MED/EMA/LAST, HELD/FRESH/STALE, tuổi echo hợp lệ, số timeout liên tiếp,
tỷ lệ timeout, số mẫu phục hồi, chốt near/caution/fault, encoder và ACTION.
Các nhóm CONTROL chỉ in khi thay đổi hành động, tối đa hai lần/giây.

## Kiểm thử

Bộ host include chính sketch và kiểm tra đủ 15 tình huống tối ưu tổng thể,
các chuỗi lọc/NO_ECHO trước đó, bốn chuỗi né, các pha phục hồi, phân loại xung
xa/kẹt HIGH, line PD, giới hạn PWM và nhóm Serial. G++ C++11 với
-Wall -Wextra -pedantic đã chạy thành công. Chưa có kết quả toolchain ESP32
hoặc thử nghiệm trên phần cứng; mô phỏng không thay thế hiệu chỉnh xe thực.