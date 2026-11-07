# Xe dò line — phương án A, sonar xác nhận mép vật

Giữ `RobotTypes.h` cùng thư mục với file `.ino`. Các enum/struct được đặt trong
header này để khai báo hàm tự sinh của Arduino nhìn thấy kiểu trước khi dùng.
Không chỉ sao chép riêng nội dung `.ino` sang một sketch thiếu header.

**Bản tối ưu hiện tại:** xem [CONTROL_REVIEW.md](CONTROL_REVIEW.md) để đọc luồng
state machine, PD line, xử lý lỗi sonar thoáng qua và các nhóm Serial mới.
Giám sát encoder khi cua được cập nhật trong [ENCODER_MONITOR.md](ENCODER_MONITOR.md):
tách PWM mục tiêu/đầu ra, kiểm tra sau lệnh mới và cửa sổ thích nghi theo chuyển động.

Luồng né: chọn phía → xoay ra → dịch ngang quét mặt vật bằng sonar → xác nhận
mép và đưa hết thân qua mép → xoay song song → đi thẳng qua vật → xác nhận mép
cuối → đưa đuôi xe qua mép → xoay về line → tìm line → khôi phục hướng → dò line.

Không dùng điều chỉnh bám cạnh P trong BYPASS. Chỉ bù encoder để đi thẳng.

## Kích thước đã biết

- Khoảng cách mép ngoài hai bánh: **16 cm**.
- Mỗi bánh rộng **3 cm** → khoảng cách tâm hai bánh: **13 cm**, không phải 16 cm.
- Toàn xe dài **25 cm**; sonar bên đặt ở đầu xe.
- Chưa xác nhận: PPR và đường kính bánh. Hiện vẫn để tạm **20 xung/vòng bánh**,
  đường kính từng bánh **6.5 cm**. Phải đo và hiệu chỉnh trước khi tin vào góc quay.
- `TURN_LEFT_SCALE`, `TURN_RIGHT_SCALE`: hệ số hiệu chỉnh riêng, mặc định 1.
  Đo góc thật rồi nhân hệ số cũ với góc mục tiêu / góc thật. Sửa nguyên nhân
  mất xung trước; không dùng hệ số để che lỗi encoder chập chờn.

## Dịch ngang theo sonar, không đặt trước quãng né

Khi xe đã xoay ngang, cảm biến phía vật (trái nếu né phải) quét mặt trước của
vật trong lúc xe tiến ngang. Code yêu cầu:

1. Hai mẫu echo mới liên tiếp cho thấy mặt vật trong vùng hợp lý so với khoảng
   cách trước khi né và kích thước xe. Vùng này cộng chiều dài xe vì sau khi
   xoay, sonar đầu xe có thể nằm sau mặt phẳng cảm biến trước ban đầu.
2. Chuỗi echo hợp lệ mới liên tiếp cho thấy khoảng trống (không tính NO_ECHO);
   ít nhất 350 ms kể từ lần cuối còn thấy mặt vật. Mặt vật xuất hiện lại sẽ hủy
   xác nhận mép và tính lại từ đầu. Các lần gọi control không có mẫu mới không
   được tính thêm lần xác nhận. Cần hai echo hợp lệ liên tiếp vượt ngưỡng
   trước khi một mẫu được phép đóng góp vào xác nhận khoảng trống.
3. Kể từ mẫu khoảng trống đầu tiên, encoder đo phần thân còn phải vượt mép.
   Biên bảo thủ cho một lần xoay 90 độ là khoảng sonar đến đuôi + nửa bề rộng
   xe + khoảng hở tối thiểu: **25 + 8 + 4 = 37 cm**. Biên này không giả định
   trục bánh nằm chính giữa chiều dài xe. Cần kiểm tra thêm nếu thân/phụ kiện
   nhô ngoài hình bao 16 × 25 cm hoặc vị trí sonar thực khác mô tả.
4. Chỉ khi cả xác nhận sonar và phần bù thân đạt yêu cầu mới xoay song song.
   Tổng quãng ngang thay đổi theo vị trí mép vật, được lưu ở `measuredOffsetCm`.

37 cm là phần bù hình học SAU mép đo được, không phải tổng quãng dịch ngang.
Các giới hạn 80 cm / 8 giây chỉ gây dừng lỗi; không cho phép tự kết luận đã qua
vật. Trong thao tác né, NO_ECHO khiến xe dừng chờ phục hồi, không đoán mép và không tiếp tục quét
trong lúc khoảng trống cần thiết chưa được xác nhận.

Sau khi xoay song song, xe đi thẳng bằng encoder; sonar bên phải thấy vật rồi
xác nhận hết vật bằng ba mẫu khoảng trống mới. Chưa xác nhận mép thì không quay
về line theo quãng dự phòng cố định. Hết giới hạn 120 cm / 12 giây sẽ dừng lỗi.
Sau mép cuối cũng cần phần bù thân trước khi xoay về line.

Sonar có chùm đo giới hạn. Vật nghiêng, nhỏ, hấp thụ âm hoặc lắp sai hướng có
thể không xác nhận được mặt/mép. Xe sẽ dừng chờ hoặc dừng lỗi, không bảo đảm nhận
biết mọi hình dạng chỉ với ba sonar cố định. NO_ECHO không còn được gán khoảng
cách giả định. Khi đang FOLLOW_LINE, sân trống không có phản xạ có thể được chạy
chậm theo chính sách UNKNOWN; khi đang né vẫn dừng nếu thiếu xác nhận cần thiết.

## Encoder, dừng giữa bước và giới hạn an toàn

- D0 trái → GPIO21; D0 phải → GPIO4; GND chung.
- D0 phải tương thích 3.3 V. Nếu đầu ra lên 5 V, cần chuyển mức trước ESP32.
  Cấp VCC theo thông số module; ưu tiên 3.3 V nếu module hỗ trợ.
- Ngắt RISING đếm một cạnh lên/khe; bỏ cạnh cách nhau dưới 250 µs. PPR là xung
  trên một vòng BÁNH, kể cả encoder nằm trước hộp số.
- Chờ hai encoder không đổi 200 ms giữa các bước chuyển động rồi mới lấy mốc
  mới. Xung quán tính không được cộng sang bước tiếp. Nếu dừng mà xung vẫn
  tiếp tục trên 1.5 giây, dừng lỗi để kiểm tra nhiễu/quán tính.
- Khi xoay, nếu hai đường bánh chênh quá mức cho phép hoặc vượt đích quá nhiều,
  dừng lỗi thay vì giữ một bánh chạy để đuổi đủ xung. D0 một kênh không biết
  chiều quay thật; phải kiểm tra dây, cơ khí và encoder nếu báo lệch/mất xung.
- Trong AUTO, encoder được kiểm tra theo PWM mục tiêu, PWM đầu ra và loại chuyển
  động. Bánh dừng chủ đích/bánh trong cua quay chậm không dùng timeout 1.5 giây
  cố định. Bánh có lệnh đáng kể vẫn có watchdog thích nghi, giai đoạn nghi kẹt
  rồi xác nhận; xem ENCODER_MONITOR.md. Không bỏ bảo vệ kẹt bánh.
- Một bên sonar đo khoảng hở ≤ 4 cm (cộng phần thân nhô ngoài mặt cảm biến)
  hoặc phía trước ≤ 5 cm sẽ dừng khẩn. Phía trước ≤ 10 cm trong lúc đi vòng
  cũng dừng; điều kiện này vẫn áp dụng lúc xoay song song. Không tắt bảo vệ
  chỉ vì đầu xe đang quét qua vật trong lúc xoay.
- Trong thao tác né, dữ liệu sonar mất echo, lỗi hoặc cũ: dừng chờ, sau 1.5 giây chưa phục hồi thì lỗi.
  Giữ MED/EMA khi mất ngắn để tham khảo, không dùng giá trị giữ để xác nhận đường trống.
  Phục hồi cần ít nhất hai echo hợp lệ; echo mới quá gần vẫn dừng ngay.
- Trong FOLLOW_LINE, LEFT/RIGHT UNKNOWN không tự gây dừng. FRONT NO_ECHO sau
  lịch sử xa được chạy giới hạn 65 PWM; NO_ECHO từ đầu được thăm dò tối đa 60 PWM
  nếu bật cấu hình. Giới hạn áp dụng cho cả hai bánh, kể cả khi đánh lái/bù encoder.
  Không xóa cảnh báo gần hoặc lỗi tín hiệu đã biết vì NO_ECHO. Sau lịch sử gần,
  xe giữ dừng trong FOLLOW_LINE và đợi echo thật xác nhận lại; không tự ABORT
  chỉ do các timeout thông thường. Fault bên chỉ giảm tốc, fault trước dừng thử
  lại và tự phục hồi; không gọi timeout né cho lỗi bên trong FOLLOW_LINE.
  Các bảo vệ va chạm, kẹt bánh và giới hạn thao tác né vẫn có thể ABORT.
- Tìm lại line có giới hạn thời gian và quãng đường. Giới hạn quãng được mở
  theo quãng dịch ngang đã đo, không giữ 45 cm khi xe đã phải dịch ngang xa hơn.
- Toàn chuỗi né tối đa 45 giây. Các giới hạn thời gian không thay thế số xung
  hoặc xác nhận sonar để hoàn thành một bước.

## Hiệu chỉnh và lệnh Serial

`S`: dừng/MANUAL. `A`: AUTO. Sau ABORTED phải kiểm tra xe, gửi `S` rồi `A`.
`D`: line, sonar, encoder, tiến trình xác nhận mép. `E`: xung/quãng encoder.
`Z`: xóa bộ đếm khi đã dừng trong MANUAL. `F/B/L/R`: kiểm tra động cơ thủ công;
chế độ thủ công không áp dụng các bộ bảo vệ AUTO.

Kê bánh, gửi `S`, `Z`; quay mỗi bánh đúng một vòng rồi gửi `E` để đo PPR và
kiểm tra đúng thứ tự trái/phải. Đo đường kính lăn; thử đi thẳng và xoay trước
khi chạy né. Với bánh 6.5 cm và 20 xung/vòng, một xung khoảng 1.02 cm nên độ
phân giải góc vẫn thô. Encoder đo đường bánh, không phát hiện được bánh trượt.

Tham khảo [GPIO Arduino ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/gpio.html)
và [datasheet ESP32](https://documentation.espressif.com/esp32_datasheet_en.html).

## Kiểm thử host

Từ thư mục hardware, cần g++:

```powershell
g++ -std=c++11 -Wall -Wextra -pedantic -static -Itests/esp32_avoid tests/esp32_avoid/test.cpp -o "$env:TEMP/esp32-avoid-host-test.exe"
& "$env:TEMP/esp32-avoid-host-test.exe"
```

Mô phỏng hai bề rộng vật, hai hướng né, lịch đo luân phiên và bánh sinh xung.
Kiểm tra quãng dịch ngang thay đổi theo mép, không chấp nhận NO_ECHO ban đầu
là mép, hủy khe hở ngắn, chờ xung quán tính, dừng khi lệch encoder và giữ các
bảo vệ sonar. Không thay thế biên dịch ESP32 core 3.x hoặc thử cơ khí trên xe.

Chi tiết bộ lọc, trạng thái, lịch đo và tám tình huống kiểm thử:
[SONAR_FILTER_NOTES.md](SONAR_FILTER_NOTES.md).
